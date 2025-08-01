// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/motion_velocity_planner_common/planner_data.hpp"

#include "autoware/object_recognition_utils/predicted_path_utils.hpp"
#include "autoware_lanelet2_extension/utility/query.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_math/normalization.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
using autoware_perception_msgs::msg::PredictedPath;
namespace bg = boost::geometry;

namespace
{
std::optional<geometry_msgs::msg::Pose> get_predicted_object_pose_from_predicted_path(
  const PredictedPath & predicted_path, const rclcpp::Time & obj_stamp,
  const rclcpp::Time & current_stamp)
{
  const double rel_time = (current_stamp - obj_stamp).seconds();
  if (rel_time < 0.0) {
    return std::nullopt;
  }

  const auto pose =
    autoware::object_recognition_utils::calcInterpolatedPose(predicted_path, rel_time);
  if (!pose) {
    return std::nullopt;
  }
  return pose.get();
}

std::optional<geometry_msgs::msg::Pose> get_predicted_object_pose_from_predicted_paths(
  const std::vector<PredictedPath> & predicted_paths, const rclcpp::Time & obj_stamp,
  const rclcpp::Time & current_stamp)
{
  if (predicted_paths.empty()) {
    return std::nullopt;
  }

  // Get the most reliable path
  const auto predicted_path = std::max_element(
    predicted_paths.begin(), predicted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });

  return get_predicted_object_pose_from_predicted_path(*predicted_path, obj_stamp, current_stamp);
}
}  // namespace

PlannerData::PlannerData(rclcpp::Node & node)
: vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo())
{
  // nearest search
  ego_nearest_dist_threshold = get_or_declare_parameter<double>(node, "ego_nearest_dist_threshold");
  ego_nearest_yaw_threshold = get_or_declare_parameter<double>(node, "ego_nearest_yaw_threshold");

  trajectory_polygon_collision_check.decimate_trajectory_step_length =
    get_or_declare_parameter<double>(
      node, "trajectory_polygon_collision_check.decimate_trajectory_step_length");
  trajectory_polygon_collision_check.goal_extended_trajectory_length =
    get_or_declare_parameter<double>(
      node, "trajectory_polygon_collision_check.goal_extended_trajectory_length");
  trajectory_polygon_collision_check.enable_to_consider_current_pose =
    get_or_declare_parameter<bool>(
      node,
      "trajectory_polygon_collision_check.consider_current_pose.enable_to_consider_current_pose");
  trajectory_polygon_collision_check.time_to_convergence = get_or_declare_parameter<double>(
    node, "trajectory_polygon_collision_check.consider_current_pose.time_to_convergence");

  pointcloud_obstacle_filtering_param.pointcloud_voxel_grid_x =
    get_or_declare_parameter<double>(node, "pointcloud.pointcloud_voxel_grid_x");
  pointcloud_obstacle_filtering_param.pointcloud_voxel_grid_y =
    get_or_declare_parameter<double>(node, "pointcloud.pointcloud_voxel_grid_y");
  pointcloud_obstacle_filtering_param.pointcloud_voxel_grid_z =
    get_or_declare_parameter<double>(node, "pointcloud.pointcloud_voxel_grid_z");
  pointcloud_obstacle_filtering_param.pointcloud_cluster_tolerance =
    get_or_declare_parameter<double>(node, "pointcloud.pointcloud_cluster_tolerance");
  pointcloud_obstacle_filtering_param.pointcloud_min_cluster_size =
    get_or_declare_parameter<int>(node, "pointcloud.pointcloud_min_cluster_size");
  pointcloud_obstacle_filtering_param.pointcloud_max_cluster_size =
    get_or_declare_parameter<int>(node, "pointcloud.pointcloud_max_cluster_size");

  mask_lat_margin = get_or_declare_parameter<double>(node, "pointcloud.mask_lat_margin");

  no_ground_pointcloud = Pointcloud(pointcloud_obstacle_filtering_param, mask_lat_margin);
}
std::optional<TrafficSignalStamped> PlannerData::get_traffic_signal(
  const lanelet::Id id, const bool keep_last_observation) const
{
  const auto & traffic_light_id_map =
    keep_last_observation ? traffic_light_id_map_last_observed_ : traffic_light_id_map_raw_;
  if (traffic_light_id_map.count(id) == 0) {
    return std::nullopt;
  }
  return std::make_optional<TrafficSignalStamped>(traffic_light_id_map.at(id));
}

std::optional<double> PlannerData::calculate_min_deceleration_distance(
  const double target_velocity) const
{
  return motion_utils::calcDecelDistWithJerkAndAccConstraints(
    current_odometry.twist.twist.linear.x, target_velocity,
    current_acceleration.accel.accel.linear.x, velocity_smoother_->getMinDecel(),
    std::abs(velocity_smoother_->getMinJerk()), velocity_smoother_->getMinJerk());
}

double PlannerData::Object::get_dist_to_traj_poly(
  const std::vector<autoware_utils_geometry::Polygon2d> & decimated_traj_polys) const
{
  if (!dist_to_traj_poly) {
    const auto & obj_pose = predicted_object.kinematics.initial_pose_with_covariance.pose;
    const auto obj_poly = autoware_utils_geometry::to_polygon2d(obj_pose, predicted_object.shape);
    dist_to_traj_poly = std::numeric_limits<double>::max();
    for (const auto & traj_poly : decimated_traj_polys) {
      const double current_dist_to_traj_poly = bg::distance(traj_poly, obj_poly);
      dist_to_traj_poly = std::min(*dist_to_traj_poly, current_dist_to_traj_poly);
    }
  }
  return *dist_to_traj_poly;
}

double PlannerData::Object::get_dist_to_traj_lateral(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  if (!dist_to_traj_lateral) {
    const auto & obj_pos = predicted_object.kinematics.initial_pose_with_covariance.pose.position;
    dist_to_traj_lateral = autoware::motion_utils::calcLateralOffset(traj_points, obj_pos);
  }
  return *dist_to_traj_lateral;
}

double PlannerData::Object::get_dist_from_ego_longitudinal(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Point & ego_pos) const
{
  if (!dist_from_ego_longitudinal) {
    const auto & obj_pos = predicted_object.kinematics.initial_pose_with_covariance.pose.position;
    dist_from_ego_longitudinal =
      autoware::motion_utils::calcSignedArcLength(traj_points, ego_pos, obj_pos);
  }
  return *dist_from_ego_longitudinal;
}

double PlannerData::Object::get_lon_vel_relative_to_traj(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  if (!lon_vel_relative_to_traj) {
    calc_vel_relative_to_traj(traj_points);
  }
  return *lon_vel_relative_to_traj;
}

double PlannerData::Object::get_lat_vel_relative_to_traj(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  if (!lat_vel_relative_to_traj) {
    calc_vel_relative_to_traj(traj_points);
  }
  return *lat_vel_relative_to_traj;
}

void PlannerData::Object::calc_vel_relative_to_traj(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  const auto & obj_pose = predicted_object.kinematics.initial_pose_with_covariance.pose;
  const auto & obj_twist = predicted_object.kinematics.initial_twist_with_covariance.twist;

  const size_t object_idx =
    autoware::motion_utils::findNearestIndex(traj_points, obj_pose.position);
  const auto & nearest_traj_point = traj_points.at(object_idx);

  const double traj_yaw = tf2::getYaw(nearest_traj_point.pose.orientation);
  const double obj_yaw = tf2::getYaw(obj_pose.orientation);
  const Eigen::Rotation2Dd R_ego_to_obstacle(
    autoware_utils_math::normalize_radian(obj_yaw - traj_yaw));

  // Calculate the trajectory direction and the vector from the trajectory to the obstacle
  const Eigen::Vector2d traj_direction(std::cos(traj_yaw), std::sin(traj_yaw));
  const Eigen::Vector2d traj_to_obstacle(
    obj_pose.position.x - nearest_traj_point.pose.position.x,
    obj_pose.position.y - nearest_traj_point.pose.position.y);

  // Determine if the obstacle is to the left or right of the trajectory using the cross product
  const double cross_product =
    traj_direction.x() * traj_to_obstacle.y() - traj_direction.y() * traj_to_obstacle.x();
  const int sign = (cross_product > 0) ? -1 : 1;

  const Eigen::Vector2d obstacle_velocity(obj_twist.linear.x, obj_twist.linear.y);
  const Eigen::Vector2d projected_velocity = R_ego_to_obstacle * obstacle_velocity;

  lon_vel_relative_to_traj = projected_velocity[0];
  lat_vel_relative_to_traj = sign * projected_velocity[1];
}

geometry_msgs::msg::Pose PlannerData::Object::get_predicted_current_pose(
  const rclcpp::Time & current_stamp, const rclcpp::Time & predicted_objects_stamp) const
{
  if (!predicted_pose) {
    predicted_pose = calc_predicted_pose(current_stamp, predicted_objects_stamp);
  }
  return *predicted_pose;
}

geometry_msgs::msg::Pose PlannerData::Object::calc_predicted_pose(
  const rclcpp::Time & time, const rclcpp::Time & predicted_objects_stamp) const
{
  const auto predicted_pose_opt = get_predicted_object_pose_from_predicted_paths(
    predicted_object.kinematics.predicted_paths, predicted_objects_stamp, time);
  if (!predicted_pose_opt) {
    RCLCPP_WARN(
      rclcpp::get_logger("motion_velocity_planner_common"),
      "Failed to calculate the predicted object pose.");
    return predicted_object.kinematics.initial_pose_with_covariance.pose;
  }
  return *predicted_pose_opt;
}

void PlannerData::process_predicted_objects(
  const autoware_perception_msgs::msg::PredictedObjects & predicted_objects)
{
  predicted_objects_header = predicted_objects.header;

  objects.clear();
  for (const auto & predicted_object : predicted_objects.objects) {
    objects.push_back(std::make_shared<Object>(predicted_object));
  }
}

std::vector<StopPoint> PlannerData::calculate_map_stop_points(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory) const
{
  std::vector<StopPoint> stop_points;
  if (!route_handler) {
    return stop_points;
  }
  autoware_utils_geometry::LineString2d trajectory_ls;
  for (const auto & p : trajectory) {
    trajectory_ls.emplace_back(p.pose.position.x, p.pose.position.y);
  }
  const auto candidates = route_handler->getLaneletMapPtr()->laneletLayer.search(
    boost::geometry::return_envelope<lanelet::BoundingBox2d>(trajectory_ls));
  for (const auto & candidate : candidates) {
    const auto stop_lines = lanelet::utils::query::stopLinesLanelet(candidate);
    for (const auto & stop_line : stop_lines) {
      const auto stop_line_2d = lanelet::utils::to2D(stop_line).basicLineString();
      autoware_utils_geometry::MultiPoint2d intersections;
      boost::geometry::intersection(trajectory_ls, stop_line_2d, intersections);
      for (const auto & intersection : intersections) {
        const auto p =
          geometry_msgs::msg::Point().set__x(intersection.x()).set__y(intersection.y());
        const auto stop_line_arc_length = motion_utils::calcSignedArcLength(trajectory, 0UL, p);
        StopPoint sp;
        sp.ego_trajectory_arc_length =
          stop_line_arc_length - vehicle_info_.max_longitudinal_offset_m;
        if (sp.ego_trajectory_arc_length < 0.0) {
          continue;
        }
        sp.stop_line = stop_line_2d;
        sp.ego_stop_pose =
          motion_utils::calcInterpolatedPose(trajectory, sp.ego_trajectory_arc_length);
        stop_points.push_back(sp);
      }
    }
  }
  return stop_points;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr PlannerData::Pointcloud::get_filtered_pointcloud_ptr(
  const autoware::motion_velocity_planner::TrajectoryPoints & trajectory_points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info) const
{
  if (!filtered_pointcloud_ptr) {
    auto pair = filter_and_cluster_point_clouds(trajectory_points, vehicle_info);
    filtered_pointcloud_ptr = pair.first;
    cluster_indices = pair.second;
  }
  return *filtered_pointcloud_ptr;
}

const std::vector<pcl::PointIndices> PlannerData::Pointcloud::get_cluster_indices(
  const autoware::motion_velocity_planner::TrajectoryPoints & trajectory_points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info) const
{
  if (!cluster_indices) {
    auto pair = filter_and_cluster_point_clouds(trajectory_points, vehicle_info);
    filtered_pointcloud_ptr = pair.first;
    cluster_indices = pair.second;
  }
  return *cluster_indices;
}

void PlannerData::Pointcloud::search_pointcloud_near_trajectory(
  const std::vector<TrajectoryPoint> & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_points_ptr,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & output_points_ptr) const
{
  const double front_length = vehicle_info.max_longitudinal_offset_m;
  const double rear_length = vehicle_info.rear_overhang_m;
  const double vehicle_width = vehicle_info.vehicle_width_m;

  output_points_ptr->header = input_points_ptr->header;

  // Build footprints from trajectory
  std::vector<Polygon2d> footprints;
  footprints.reserve(trajectory.size());

  std::transform(
    trajectory.begin(), trajectory.end(), std::back_inserter(footprints),
    [&](const TrajectoryPoint & trajectory_point) {
      return autoware_utils_geometry::to_footprint(
        trajectory_point.pose, front_length, rear_length, vehicle_width + mask_lat_margin_ * 2.0);
    });

  // Define types for Boost.Geometry
  namespace bg = boost::geometry;
  namespace bgi = boost::geometry::index;
  using BoostPoint2D = bg::model::point<double, 2, bg::cs::cartesian>;
  using BoostValue = std::pair<BoostPoint2D, size_t>;  // point + index

  // Build R-tree from input points
  std::vector<BoostValue> rtree_data;
  rtree_data.reserve(input_points_ptr->points.size());

  {
    std::transform(
      input_points_ptr->points.begin(), input_points_ptr->points.end(),
      std::back_inserter(rtree_data), [i = 0](const pcl::PointXYZ & pt) mutable {
        return std::make_pair(BoostPoint2D(pt.x, pt.y), i++);
      });
  }

  bgi::rtree<BoostValue, bgi::quadratic<16>> rtree(rtree_data.begin(), rtree_data.end());

  std::unordered_set<size_t> selected_indices;

  std::for_each(footprints.begin(), footprints.end(), [&](const Polygon2d & footprint) {
    bg::model::box<BoostPoint2D> bbox;
    bg::envelope(footprint, bbox);

    std::vector<BoostValue> result_s;
    rtree.query(bgi::intersects(bbox), std::back_inserter(result_s));

    for (const auto & val : result_s) {
      const BoostPoint2D & pt = val.first;
      if (bg::within(pt, footprint)) {
        selected_indices.insert(val.second);
      }
    }
  });

  output_points_ptr->points.reserve(selected_indices.size());
  std::transform(
    selected_indices.begin(), selected_indices.end(), std::back_inserter(output_points_ptr->points),
    [&](const size_t idx) { return input_points_ptr->points[idx]; });
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<pcl::PointIndices>>
PlannerData::Pointcloud::filter_and_cluster_point_clouds(
  const autoware::motion_velocity_planner::TrajectoryPoints & trajectory_points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info) const
{
  if (pointcloud.empty()) {
    return {};
  }

  // 1. transform pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr =
    std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pointcloud);

  // 2. filter-out points far-away from trajectory
  pcl::PointCloud<pcl::PointXYZ>::Ptr far_away_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  search_pointcloud_near_trajectory(
    trajectory_points, vehicle_info, pointcloud_ptr, far_away_pointcloud_ptr);

  // 3. downsample & cluster pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> filter;

  filter.setInputCloud(far_away_pointcloud_ptr);
  filter.setLeafSize(
    pointcloud_obstacle_filtering_param_.pointcloud_voxel_grid_x,
    pointcloud_obstacle_filtering_param_.pointcloud_voxel_grid_y,
    pointcloud_obstacle_filtering_param_.pointcloud_voxel_grid_z);
  filter.filter(*filtered_points_ptr);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(filtered_points_ptr);
  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(pointcloud_obstacle_filtering_param_.pointcloud_cluster_tolerance);
  ec.setMinClusterSize(pointcloud_obstacle_filtering_param_.pointcloud_min_cluster_size);
  ec.setMaxClusterSize(pointcloud_obstacle_filtering_param_.pointcloud_max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(filtered_points_ptr);
  ec.extract(clusters);

  return std::make_pair(filtered_points_ptr, clusters);
}
}  // namespace autoware::motion_velocity_planner
