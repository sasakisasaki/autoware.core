// Copyright 2024 TIER IV, Inc.
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

#include "autoware/path_generator/node.hpp"

#include "autoware/path_generator/utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include "autoware/trajectory/utils/reference_path.hpp"

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
template <typename LaneletT, typename PointT>
double get_arc_length_along_centerline(const LaneletT & lanelet, const PointT & point)
{
  return lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), lanelet::utils::to2D(point))
    .length;
}
}  // namespace

namespace autoware::path_generator
{
PathGenerator::PathGenerator(const rclcpp::NodeOptions & node_options)
: Node("path_generator", node_options)
{
  param_listener_ =
    std::make_shared<::path_generator::ParamListener>(this->get_node_parameters_interface());

  path_publisher_ = create_publisher<PathWithLaneId>("~/output/path", 1);

  turn_signal_publisher_ =
    create_publisher<TurnIndicatorsCommand>("~/output/turn_indicators_cmd", 1);

  hazard_signal_publisher_ = create_publisher<HazardLightsCommand>("~/output/hazard_lights_cmd", 1);

  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  const auto params = param_listener_->get_params();

  // Ensure that the refine_goal_search_radius_range and search_radius_decrement must be positive
  if (params.refine_goal_search_radius_range <= 0 || params.search_radius_decrement <= 0) {
    throw std::runtime_error(
      "refine_goal_search_radius_range and search_radius_decrement must be positive");
  }

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params.planning_hz).period(),
    std::bind(&PathGenerator::run, this));
}

void PathGenerator::run()
{
  const auto input_data = take_data();
  set_planner_data(input_data);
  if (!is_data_ready(input_data)) {
    return;
  }

  const auto param = param_listener_->get_params();
  const auto path = plan_path(input_data, param);
  if (!path) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is invalid");
    return;
  }

  auto turn_signal = utils::get_turn_signal(
    *path, planner_data_, input_data.odometry_ptr->pose.pose,
    input_data.odometry_ptr->twist.twist.linear.x, param.turn_signal.search_distance,
    param.turn_signal.search_time, param.turn_signal.angle_threshold_deg,
    vehicle_info_.max_longitudinal_offset_m);
  turn_signal.stamp = now();
  turn_signal_publisher_->publish(turn_signal);

  HazardLightsCommand hazard_signal;
  hazard_signal.command = HazardLightsCommand::NO_COMMAND;
  hazard_signal.stamp = now();
  hazard_signal_publisher_->publish(hazard_signal);

  path_publisher_->publish(*path);
}

PathGenerator::InputData PathGenerator::take_data()
{
  InputData input_data;

  // route
  if (const auto msg = route_subscriber_.take_data()) {
    if (msg->segments.empty()) {
      RCLCPP_ERROR(get_logger(), "input route is empty, ignoring...");
    } else {
      input_data.route_ptr = msg;
    }
  }

  // map
  if (const auto msg = vector_map_subscriber_.take_data()) {
    input_data.lanelet_map_bin_ptr = msg;
  }

  // velocity
  if (const auto msg = odometry_subscriber_.take_data()) {
    input_data.odometry_ptr = msg;
  }

  return input_data;
}

void PathGenerator::set_planner_data(const InputData & input_data)
{
  if (input_data.lanelet_map_bin_ptr) {
    planner_data_.lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(
      *input_data.lanelet_map_bin_ptr, planner_data_.lanelet_map_ptr,
      &planner_data_.traffic_rules_ptr, &planner_data_.routing_graph_ptr);
  }

  if (input_data.route_ptr) {
    set_route(input_data.route_ptr);
  }
}

void PathGenerator::set_route(const LaneletRoute::ConstSharedPtr & route_ptr)
{
  planner_data_.route_frame_id = route_ptr->header.frame_id;
  planner_data_.goal_pose = route_ptr->goal_pose;

  planner_data_.route_lanelets.clear();
  planner_data_.preferred_lanelets.clear();
  planner_data_.start_lanelets.clear();
  planner_data_.goal_lanelets.clear();

  size_t primitives_num = 0;
  for (const auto & route_section : route_ptr->segments) {
    primitives_num += route_section.primitives.size();
  }
  planner_data_.route_lanelets.reserve(primitives_num);

  for (const auto & route_section : route_ptr->segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(id);
      planner_data_.route_lanelets.push_back(lanelet);
      if (id == route_section.preferred_primitive.id) {
        planner_data_.preferred_lanelets.push_back(lanelet);
      }
    }
  }

  const auto set_lanelets_from_segment =
    [&](
      const autoware_planning_msgs::msg::LaneletSegment & segment,
      lanelet::ConstLanelets & lanelets) {
      lanelets.reserve(segment.primitives.size());
      for (const auto & primitive : segment.primitives) {
        const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(primitive.id);
        lanelets.push_back(lanelet);
      }
    };
  set_lanelets_from_segment(route_ptr->segments.front(), planner_data_.start_lanelets);
  set_lanelets_from_segment(route_ptr->segments.back(), planner_data_.goal_lanelets);
}

bool PathGenerator::is_data_ready(const InputData & input_data)
{
  const auto notify_waiting = [this](const std::string & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for %s", name.c_str());
  };

  if (!planner_data_.lanelet_map_ptr) {
    notify_waiting("map");
    return false;
  }

  if (planner_data_.route_lanelets.empty()) {
    notify_waiting("route");
    return false;
  }

  if (!input_data.odometry_ptr) {
    notify_waiting("odometry");
    return false;
  }

  return true;
}

std::optional<PathWithLaneId> PathGenerator::plan_path(
  const InputData & input_data, const Params & params)
{
  const auto path = generate_path(input_data.odometry_ptr->pose.pose, params);

  if (!path) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is invalid");
    return std::nullopt;
  }
  if (path->points.empty()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is empty");
    return std::nullopt;
  }

  return path;
}

std::optional<PathWithLaneId> PathGenerator::generate_path(
  const geometry_msgs::msg::Pose & current_pose, const Params & params)
{
  if (!update_current_lanelet(current_pose, params)) {
    return std::nullopt;
  }

  const auto reference_path_opt = autoware::experimental::trajectory::build_reference_path(
      {*current_lanelet_}, *current_lanelet_, current_pose, planner_data_.lanelet_map_ptr,
      planner_data_.routing_graph_ptr, planner_data_.traffic_rules_ptr,
      params.path_length.forward, params.path_length.backward);
  if (!reference_path_opt.has_value()) {
    return std::nullopt;
  }
  auto reference_path = reference_path_opt.value();

  // Check if the goal point is in the search range
  // Note: We only see if the goal is approaching the tail of the path.
  const auto distance_to_goal = autoware_utils::calc_distance2d(
    reference_path.compute(reference_path.length()), planner_data_.goal_pose);

  if (distance_to_goal < params.refine_goal_search_radius_range) {
    auto refined_path = utils::modify_path_for_smooth_goal_connection(
      reference_path, planner_data_, params.refine_goal_search_radius_range);

    if (refined_path) {
      refined_path->align_orientation_with_trajectory_direction();
      reference_path = *refined_path;
    }
  }

  // Compose the polished path
  PathWithLaneId finalized_path_with_lane_id{};

  finalized_path_with_lane_id.points = reference_path.restore();

  if (finalized_path_with_lane_id.points.empty()) {
    return std::nullopt;
  }

  // Set header which is needed to engage
  finalized_path_with_lane_id.header.frame_id = planner_data_.route_frame_id;
  finalized_path_with_lane_id.header.stamp = now();

  //const auto [left_bound, right_bound] = utils::get_path_bounds(
  //  extended_lanelet_sequence, std::max(0., s_offset + s_bound_start),
  //  std::max(0., s_offset + s_bound_end));
  lanelet::LaneletSequence lanelet_sequence({*current_lanelet_});
  // TODO:                                                           !! Do not hardcode s_start and s_end !!
  const auto [left_bound, right_bound] = utils::get_path_bounds(lanelet_sequence, 0.0, 100.0);
  finalized_path_with_lane_id.left_bound = left_bound;
  finalized_path_with_lane_id.right_bound = right_bound;

  return finalized_path_with_lane_id;
}

bool PathGenerator::update_current_lanelet(
  const geometry_msgs::msg::Pose & current_pose, const Params & params)
{
  if (!current_lanelet_) {
    lanelet::ConstLanelet current_lanelet;
    if (lanelet::utils::query::getClosestLanelet(
          planner_data_.route_lanelets, current_pose, &current_lanelet)) {
      current_lanelet_ = current_lanelet;
      return true;
    }
    return false;
  }

  lanelet::ConstLanelets candidates;
  if (
    const auto previous_lanelet =
      utils::get_previous_lanelet_within_route(*current_lanelet_, planner_data_)) {
    candidates.push_back(*previous_lanelet);
  }
  candidates.push_back(*current_lanelet_);
  if (
    const auto next_lanelet =
      utils::get_next_lanelet_within_route(*current_lanelet_, planner_data_)) {
    candidates.push_back(*next_lanelet);
  }

  if (lanelet::utils::query::getClosestLaneletWithConstrains(
        candidates, current_pose, &*current_lanelet_, params.ego_nearest_dist_threshold,
        params.ego_nearest_yaw_threshold)) {
    return true;
  }

  if (lanelet::utils::query::getClosestLanelet(
        planner_data_.route_lanelets, current_pose, &*current_lanelet_)) {
    return true;
  }

  return false;
}
}  // namespace autoware::path_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::path_generator::PathGenerator)
