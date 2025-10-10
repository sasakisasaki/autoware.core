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

#ifndef AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__UTILS_HPP_
#define AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__UTILS_HPP_

#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "planner_data.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::utils
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::Polygon2d;
using nav_msgs::msg::Odometry;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

geometry_msgs::msg::Point to_geometry_point(const pcl::PointXYZ & point);
geometry_msgs::msg::Point to_geometry_point(const autoware_utils_geometry::Point2d & point);

/**
 * @brief compute the distance between `ego_idx` and `object_pos` along `traj_points`, only when
 * ego_idx is behind of `obstacle_pos`
 */
std::optional<double> calc_distance_to_front_object(
  const std::vector<TrajectoryPoint> & traj_points, const size_t ego_idx,
  const geometry_msgs::msg::Point & obstacle_pos);

template <class T>
std::vector<T> concat_vectors(std::vector<T> first_vector, std::vector<T> second_vector)
{
  first_vector.insert(
    first_vector.end(), std::make_move_iterator(second_vector.begin()),
    std::make_move_iterator(second_vector.end()));
  return first_vector;
}

/**
 * @brief crop part of the `traj_points` from `current_pose`, resample it by
 * `decimate_trajectory_step_length`, and extend the end by `goal_extended_trajectory_length`
 */
std::vector<TrajectoryPoint> decimate_trajectory_points_from_ego(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & current_pose,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold,
  const double decimate_trajectory_step_length, const double goal_extended_trajectory_length);

template <typename T, typename U>
std::optional<T> get_obstacle_from_uuid(const std::vector<T> & obstacles, const U & target_uuid)
{
  const auto itr = std::find_if(obstacles.begin(), obstacles.end(), [&](const auto & obstacle) {
    return obstacle.uuid == target_uuid;
  });

  if (itr == obstacles.end()) {
    return std::nullopt;
  }
  return *itr;
}

std::vector<uint8_t> get_target_object_type(rclcpp::Node & node, const std::string & param_prefix);

/**
 * @brief compute the half of the diagonal length of the shape
 */
double calc_object_possible_max_dist_from_center(const Shape & shape);

Marker get_object_marker(
  const geometry_msgs::msg::Pose & obj_pose, size_t idx, const std::string & ns, const double r,
  const double g, const double b);

/**
 * @brief compute the index of the point which overpasses `longitudinal_offset` for the 1st time,
 * and return that index or the index next to it which is closer to `longitudinal_offset`
 */
template <class T>
size_t get_index_with_longitudinal_offset(
  const T & points, const double longitudinal_offset, std::optional<size_t> start_idx)
{
  if (points.empty()) {
    throw std::logic_error("points is empty.");
  }

  if (start_idx) {
    if (/*start_idx.get() < 0 || */ points.size() <= *start_idx) {
      throw std::out_of_range("start_idx is out of range.");
    }
  } else {
    if (longitudinal_offset > 0) {
      start_idx = 0;
    } else {
      start_idx = points.size() - 1;
    }
  }

  double sum_length = 0.0;
  if (longitudinal_offset > 0) {
    for (size_t i = *start_idx; i < points.size() - 1; ++i) {
      const double segment_length =
        autoware_utils_geometry::calc_distance2d(points.at(i), points.at(i + 1));
      sum_length += segment_length;
      if (sum_length >= longitudinal_offset) {
        const double back_length = sum_length - longitudinal_offset;
        const double front_length = segment_length - back_length;
        if (front_length < back_length) {
          return i;
        } else {
          return i + 1;
        }
      }
    }
    return points.size() - 1;
  }

  for (size_t i = *start_idx; 0 < i; --i) {
    const double segment_length =
      autoware_utils_geometry::calc_distance2d(points.at(i - 1), points.at(i));
    sum_length += segment_length;
    if (sum_length >= -longitudinal_offset) {
      const double back_length = sum_length + longitudinal_offset;
      const double front_length = segment_length - back_length;
      if (front_length < back_length) {
        return i;
      } else {
        return i - 1;
      }
    }
  }
  return 0;
}

/**
 * @brief subtract `object`'s diagonal length + `vehicle_info`'s diagonal length from the minimal
 * distance of `object` to `traj_points`
 * @note this sets the cache in `object`
 */
double calc_possible_min_dist_from_obj_to_traj_poly(
  const std::shared_ptr<PlannerData::Object> object,
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info);

/**
 * @brief return the minimum distance from `point` to each polygon in `decimated_traj_polys`
 */
double get_dist_to_traj_poly(
  const geometry_msgs::msg::Point & point,
  const std::vector<autoware_utils::Polygon2d> & decimated_traj_polys);

/*
 * @brief return the  distance from `predicted_object` to `decimated_traj_polys`
 */
double calc_dist_to_traj_poly(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  const std::vector<autoware_utils_geometry::Polygon2d> & decimated_traj_polys);

/**
 * @brief append the `input_points` up to `extend_length` every `step_length`, in the direction of
 * the last point of `input_points`, keeping its vel/acc
 */
std::vector<TrajectoryPoint> get_extended_trajectory_points(
  const std::vector<TrajectoryPoint> & input_points, const double extend_distance,
  const double step_length);
}  // namespace autoware::motion_velocity_planner::utils
#endif  // AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__UTILS_HPP_
