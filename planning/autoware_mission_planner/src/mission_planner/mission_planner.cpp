// Copyright 2019 Autoware Foundation
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

#include "mission_planner.hpp"

#include "service_utils.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/route_checker.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::mission_planner
{

MissionPlanner::MissionPlanner(const rclcpp::NodeOptions & options)
: Node("mission_planner", options),
  arrival_checker_(this),
  plugin_loader_("autoware_mission_planner", "autoware::mission_planner::PlannerPlugin"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  odometry_(nullptr),
  map_ptr_(nullptr),
  is_mission_planner_ready_(false)
{
  initialize_parameters();
  initialize_plugin();
  initialize_publishers_and_subscribers();
  initialize_services();
  initialize_timers();
}

void MissionPlanner::initialize_parameters()
{
  map_frame_ = declare_parameter<std::string>("map_frame");
  reroute_time_threshold_ = declare_parameter<double>("reroute_time_threshold");
  minimum_reroute_length_ = declare_parameter<double>("minimum_reroute_length");
  allow_reroute_in_autonomous_mode_ = declare_parameter<bool>("allow_reroute_in_autonomous_mode");
}

void MissionPlanner::initialize_plugin()
{
  planner_ =
    plugin_loader_.createSharedInstance("autoware::mission_planner::lanelet2::DefaultPlanner");
  planner_->initialize(this);
}

void MissionPlanner::initialize_publishers_and_subscribers()
{
  using std::placeholders::_1;
  const auto durable_qos = rclcpp::QoS(1).transient_local();

  // Subscribers
  sub_odometry_ = create_subscription<Odometry>(
    "~/input/odometry", rclcpp::QoS(1), std::bind(&MissionPlanner::on_odometry, this, _1));
  sub_operation_mode_state_ = create_subscription<OperationModeState>(
    "~/input/operation_mode_state", rclcpp::QoS(1),
    std::bind(&MissionPlanner::on_operation_mode_state, this, _1));
  sub_vector_map_ = create_subscription<LaneletMapBin>(
    "~/input/vector_map", durable_qos, std::bind(&MissionPlanner::on_map, this, _1));

  // Publishers
  pub_marker_ = create_publisher<MarkerArray>("~/debug/route_marker", durable_qos);
  pub_route_ = create_publisher<LaneletRoute>("~/route", durable_qos);
  pub_state_ = create_publisher<RouteState>("~/state", durable_qos);
  pub_processing_time_ = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);

  logger_configure_ = std::make_unique<autoware_utils_logging::LoggerLevelConfigure>(this);
}

void MissionPlanner::initialize_services()
{
  // NOTE: The route interface should be mutually exclusive by callback group.
  srv_clear_route = create_service<ClearRoute>(
    "~/clear_route", service_utils::handle_exception(&MissionPlanner::on_clear_route, this));
  srv_set_lanelet_route = create_service<SetLaneletRoute>(
    "~/set_lanelet_route",
    service_utils::handle_exception(&MissionPlanner::on_set_lanelet_route, this));
  srv_set_waypoint_route = create_service<SetWaypointRoute>(
    "~/set_waypoint_route",
    service_utils::handle_exception(&MissionPlanner::on_set_waypoint_route, this));
}

void MissionPlanner::initialize_timers()
{
  // Route state will be published when the node gets ready for route api after initialization,
  // otherwise the mission planner rejects the request for the API.
  const auto period = rclcpp::Rate(10).period();
  data_check_timer_ = create_wall_timer(period, [this] { check_initialization(); });
}

void MissionPlanner::publish_processing_time(
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch)
{
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  pub_processing_time_->publish(processing_time_msg);
}

void MissionPlanner::publish_pose_log(const Pose & pose, const std::string & pose_type)
{
  const auto & p = pose.position;
  RCLCPP_INFO(
    this->get_logger(), "%s pose - x: %f, y: %f, z: %f", pose_type.c_str(), p.x, p.y, p.z);
  const auto & quaternion = pose.orientation;
  RCLCPP_INFO(
    this->get_logger(), "%s orientation - qx: %f, qy: %f, qz: %f, qw: %f", pose_type.c_str(),
    quaternion.x, quaternion.y, quaternion.z, quaternion.w);
}

void MissionPlanner::check_initialization()
{
  auto logger = get_logger();
  auto clock = *get_clock();

  if (!planner_->ready()) {
    RCLCPP_INFO_THROTTLE(logger, clock, 5000, "waiting lanelet map... Route API is not ready.");
    return;
  }
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(logger, clock, 5000, "waiting odometry... Route API is not ready.");
    return;
  }

  // All data is ready. Now API is available.
  is_mission_planner_ready_ = true;
  RCLCPP_DEBUG(logger, "Route API is ready.");
  change_state(RouteState::UNSET);

  // Stop timer callback.
  data_check_timer_->cancel();
  data_check_timer_ = nullptr;
}

void MissionPlanner::on_odometry(const Odometry::ConstSharedPtr msg)
{
  odometry_ = msg;

  // NOTE: Do not check in the other states as goal may change.
  if (state_.state == RouteState::SET) {
    PoseStamped pose;
    pose.header = odometry_->header;
    pose.pose = odometry_->pose.pose;
    if (arrival_checker_.is_arrived(pose)) {
      change_state(RouteState::ARRIVED);
    }
  }
}

void MissionPlanner::on_operation_mode_state(const OperationModeState::ConstSharedPtr msg)
{
  operation_mode_state_ = msg;
}

void MissionPlanner::on_map(const LaneletMapBin::ConstSharedPtr msg)
{
  map_ptr_ = msg;
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_ptr_, lanelet_map_ptr_);
}

Pose MissionPlanner::transform_pose(const Pose & pose, const Header & header)
{
  geometry_msgs::msg::TransformStamped transform;
  geometry_msgs::msg::Pose result;
  try {
    transform = tf_buffer_.lookupTransform(map_frame_, header.frame_id, tf2::TimePointZero);
    tf2::doTransform(pose, result, transform);
    return result;
  } catch (tf2::TransformException & error) {
    throw service_utils::TransformError(error.what());
  }
}

void MissionPlanner::change_state(RouteState::_state_type state)
{
  state_.stamp = now();
  state_.state = state;
  pub_state_->publish(state_);
}

void MissionPlanner::on_clear_route(
  const ClearRoute::Request::SharedPtr, const ClearRoute::Response::SharedPtr res)
{
  validate_ready_state();

  change_route();
  change_state(RouteState::UNSET);
  res->status.success = true;
}

void MissionPlanner::validate_ready_state()
{
  if (!is_mission_planner_ready_) {
    using ResponseCode = autoware_adapi_v1_msgs::msg::ResponseStatus;
    throw service_utils::ServiceException(
      ResponseCode::NO_EFFECT, "The mission planner is not ready.", true);
  }
}

RouteValidationResult MissionPlanner::validate_route_request(bool is_reroute)
{
  RouteValidationResult result;
  result.is_valid = true;
  result.is_reroute = is_reroute;

  // Check state validity
  if (state_.state != RouteState::UNSET && state_.state != RouteState::SET) {
    result.is_valid = false;
    result.error_code = autoware_adapi_v1_msgs::srv::SetRoute::Response::ERROR_INVALID_STATE;
    result.error_message = "The route cannot be set in the current state.";
    return result;
  }

  // Check if mission planner is ready
  if (!is_mission_planner_ready_) {
    result.is_valid = false;
    result.error_code = autoware_adapi_v1_msgs::srv::SetRoute::Response::ERROR_PLANNER_UNREADY;
    result.error_message = "The mission planner is not ready.";
    return result;
  }

  // Check operation mode state for reroute
  if (is_reroute && !operation_mode_state_) {
    result.is_valid = false;
    result.error_code = autoware_adapi_v1_msgs::srv::SetRoute::Response::ERROR_PLANNER_UNREADY;
    result.error_message = "Operation mode state is not received.";
    return result;
  }

  // Check autonomous driving mode
  result.is_autonomous_driving = operation_mode_state_ ?
    operation_mode_state_->mode == OperationModeState::AUTONOMOUS &&
    operation_mode_state_->is_autoware_control_enabled : false;

  // Check if reroute is allowed in autonomous mode
  if (is_reroute && !allow_reroute_in_autonomous_mode_ && result.is_autonomous_driving) {
    result.is_valid = false;
    result.error_code = autoware_adapi_v1_msgs::srv::SetRoute::Response::ERROR_INVALID_STATE;
    result.error_message = "Reroute is not allowed in autonomous mode.";
    return result;
  }

  return result;
}

void MissionPlanner::handle_route_planning(
  const LaneletRoute & route, const RouteValidationResult & validation_result,
  const Pose & initial_pose, const Pose & goal_pose)
{
  const bool is_reroute = validation_result.is_reroute;

  // Check if route is empty
  if (route.segments.empty()) {
    cancel_route();
    change_state(is_reroute ? RouteState::SET : RouteState::UNSET);
    throw service_utils::ServiceException(
      autoware_adapi_v1_msgs::srv::SetRoute::Response::ERROR_PLANNER_FAILED,
      "The planned route is empty.");
  }

  // Check reroute safety
  if (is_reroute && validation_result.is_autonomous_driving &&
      !check_reroute_safety(*current_route_, route)) {
    cancel_route();
    change_state(RouteState::SET);
    throw service_utils::ServiceException(
      autoware_adapi_v1_msgs::srv::SetRoute::Response::ERROR_REROUTE_FAILED,
      "New route is not safe. Reroute failed.");
  }

  // Apply route changes
  change_route(route);
  change_state(RouteState::SET);

  // Log poses
  publish_pose_log(initial_pose, "initial");
  publish_pose_log(goal_pose, "goal");
}

void MissionPlanner::on_set_lanelet_route(
  const SetLaneletRoute::Request::SharedPtr req, const SetLaneletRoute::Response::SharedPtr res)
{
  const auto validation = validate_route_request(state_.state == RouteState::SET);
  if (!validation.is_valid) {
    throw service_utils::ServiceException(validation.error_code, validation.error_message);
  }

  change_state(validation.is_reroute ? RouteState::REROUTING : RouteState::ROUTING);
  const auto route = create_route(*req);

  handle_route_planning(route, validation, odometry_->pose.pose, req->goal_pose);
  res->status.success = true;
}

void MissionPlanner::on_set_waypoint_route(
  const SetWaypointRoute::Request::SharedPtr req, const SetWaypointRoute::Response::SharedPtr res)
{
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoutePoints::Response;

  const auto validation = validate_route_request(state_.state == RouteState::SET);
  if (!validation.is_valid) {
    // Map the error code for waypoint route
    auto error_code = validation.error_code;
    if (error_code == autoware_adapi_v1_msgs::srv::SetRoute::Response::ERROR_INVALID_STATE) {
      error_code = ResponseCode::ERROR_INVALID_STATE;
    } else if (error_code == autoware_adapi_v1_msgs::srv::SetRoute::Response::ERROR_PLANNER_UNREADY) {
      error_code = ResponseCode::ERROR_PLANNER_UNREADY;
    }
    throw service_utils::ServiceException(error_code, validation.error_message);
  }

  change_state(validation.is_reroute ? RouteState::REROUTING : RouteState::ROUTING);
  const auto route = create_route(*req);

  handle_route_planning(route, validation, odometry_->pose.pose, req->goal_pose);
  res->status.success = true;
}

void MissionPlanner::change_route()
{
  current_route_ = nullptr;
  planner_->clearRoute();
  arrival_checker_.set_goal();

  // TODO(Takagi, Isamu): publish an empty route here
  // pub_route_->publish();
  // pub_marker_->publish();
}

void MissionPlanner::change_route(const LaneletRoute & route)
{
  PoseWithUuidStamped goal;
  goal.header = route.header;
  goal.pose = route.goal_pose;
  goal.uuid = route.uuid;

  current_route_ = std::make_shared<LaneletRoute>(route);
  planner_->updateRoute(route);
  arrival_checker_.set_goal(goal);

  pub_route_->publish(route);
  pub_marker_->publish(planner_->visualize(route));
}

void MissionPlanner::cancel_route()
{
  // Restore planner state that changes with create_route function.
  if (current_route_) {
    planner_->updateRoute(*current_route_);
  }
}

LaneletRoute MissionPlanner::create_route(const SetLaneletRoute::Request & req)
{
  const auto & header = req.header;
  const auto & segments = req.segments;
  const auto & goal_pose = req.goal_pose;
  const auto & uuid = req.uuid;
  const auto & allow_goal_modification = req.allow_modification;

  return create_route(header, segments, goal_pose, uuid, allow_goal_modification);
}

LaneletRoute MissionPlanner::create_route(const SetWaypointRoute::Request & req)
{
  const auto & header = req.header;
  const auto & waypoints = req.waypoints;
  const auto & goal_pose = req.goal_pose;
  const auto & uuid = req.uuid;
  const auto & allow_goal_modification = req.allow_modification;

  return create_route(
    header, waypoints, odometry_->pose.pose, goal_pose, uuid, allow_goal_modification);
}

LaneletRoute MissionPlanner::create_route(
  const Header & header, const std::vector<LaneletSegment> & segments, const Pose & goal_pose,
  const UUID & uuid, const bool allow_goal_modification)
{
  LaneletRoute route;
  route.header.stamp = header.stamp;
  route.header.frame_id = map_frame_;
  route.start_pose = odometry_->pose.pose;
  route.goal_pose = transform_pose(goal_pose, header);
  route.segments = segments;
  route.uuid = uuid;
  route.allow_modification = allow_goal_modification;
  return route;
}

LaneletRoute MissionPlanner::create_route(
  const Header & header, const std::vector<Pose> & waypoints, const Pose & start_pose,
  const Pose & goal_pose, const UUID & uuid, const bool allow_goal_modification)
{
  PlannerPlugin::RoutePoints points;
  points.push_back(start_pose);
  for (const auto & waypoint : waypoints) {
    points.push_back(transform_pose(waypoint, header));
  }
  points.push_back(transform_pose(goal_pose, header));

  LaneletRoute route = planner_->plan(points);
  route.header.stamp = header.stamp;
  route.header.frame_id = map_frame_;
  route.uuid = uuid;
  route.allow_modification = allow_goal_modification;
  return route;
}

bool MissionPlanner::check_reroute_safety(
  const LaneletRoute & original_route, const LaneletRoute & target_route)
{
  // Validate inputs
  if (!validate_reroute_inputs(original_route, target_route)) {
    return false;
  }

  const auto current_velocity = odometry_->twist.twist.linear.x;

  // If vehicle is stopped, reroute is safe
  if (current_velocity < 0.01) {
    return true;
  }

  // Find common segments between routes
  const auto common_segments = find_common_segments(original_route, target_route);
  if (!common_segments.has_value()) {
    RCLCPP_ERROR(
      get_logger(), "Check reroute safety failed. Cannot find common segments between routes.");
    return false;
  }

  // Verify ego position
  if (!verify_ego_position_in_target_route(target_route)) {
    return false;
  }

  // Calculate accumulated length
  const double accumulated_length = calculate_reroute_length(
    original_route, target_route, common_segments.value());

  // Check safety
  const double safety_length =
    std::max(current_velocity * reroute_time_threshold_, minimum_reroute_length_);

  if (accumulated_length > safety_length) {
    return true;
  }

  RCLCPP_WARN(
    get_logger(),
    "Length of lane where original and target routes overlap (= %f) is less than safety length (= %f), "
    "reroute is not safe.",
    accumulated_length, safety_length);
  return false;
}

bool MissionPlanner::validate_reroute_inputs(
  const LaneletRoute & original_route, const LaneletRoute & target_route) const
{
  if (original_route.segments.empty() || target_route.segments.empty() ||
      !map_ptr_ || !lanelet_map_ptr_ || !odometry_) {
    RCLCPP_ERROR(get_logger(), "Check reroute safety failed. Route, map or odometry is not set.");
    return false;
  }
  return true;
}

std::optional<CommonSegments> MissionPlanner::find_common_segments(
  const LaneletRoute & original_route, const LaneletRoute & target_route) const
{
  auto hasSamePrimitives = [](
    const std::vector<LaneletPrimitive> & original_primitives,
    const std::vector<LaneletPrimitive> & target_primitives) {
    if (original_primitives.size() != target_primitives.size()) {
      return false;
    }

    for (const auto & primitive : original_primitives) {
      const auto has_same = [&](const auto & p) { return p.id == primitive.id; };
      const bool is_same =
        std::find_if(target_primitives.begin(), target_primitives.end(), has_same) !=
        target_primitives.end();
      if (!is_same) {
        return false;
      }
    }
    return true;
  };

  // Find start indices
  std::optional<std::pair<size_t, size_t>> start_idx_opt;
  for (size_t i = 0; i < original_route.segments.size(); ++i) {
    const auto & original_segment = original_route.segments.at(i).primitives;
    for (size_t j = 0; j < target_route.segments.size(); ++j) {
      const auto & target_segment = target_route.segments.at(j).primitives;
      if (hasSamePrimitives(original_segment, target_segment)) {
        start_idx_opt = std::make_pair(i, j);
        break;
      }
    }
    if (start_idx_opt.has_value()) break;
  }

  if (!start_idx_opt.has_value()) {
    return std::nullopt;
  }

  const auto [start_idx_original, start_idx_target] = start_idx_opt.value();

  // Find end indices
  size_t end_idx_original = start_idx_original;
  size_t end_idx_target = start_idx_target;
  for (size_t i = 1; i < target_route.segments.size() - start_idx_target; ++i) {
    if (start_idx_original + i > original_route.segments.size() - 1) {
      break;
    }

    const auto & original_primitives =
      original_route.segments.at(start_idx_original + i).primitives;
    const auto & target_primitives = target_route.segments.at(start_idx_target + i).primitives;
    if (!hasSamePrimitives(original_primitives, target_primitives)) {
      break;
    }
    end_idx_original = start_idx_original + i;
    end_idx_target = start_idx_target + i;
  }

  return CommonSegments{start_idx_original, end_idx_original, start_idx_target, end_idx_target};
}

bool MissionPlanner::verify_ego_position_in_target_route(const LaneletRoute & target_route) const
{
  const bool ego_is_on_first_target_section = std::any_of(
    target_route.segments.front().primitives.begin(),
    target_route.segments.front().primitives.end(), [&](const auto & primitive) {
      const auto lanelet = lanelet_map_ptr_->laneletLayer.get(primitive.id);
      return lanelet::utils::isInLanelet(target_route.start_pose, lanelet);
    });

  if (!ego_is_on_first_target_section) {
    RCLCPP_ERROR(
      get_logger(),
      "Check reroute safety failed. Ego is not on the first section of target route.");
    return false;
  }
  return true;
}

double MissionPlanner::calculate_reroute_length(
  const LaneletRoute & original_route, const LaneletRoute & target_route,
  const CommonSegments & common_segments) const
{
  double accumulated_length = 0.0;

  // Calculate distance from current pose to the beginning of common segment
  accumulated_length += calculate_distance_to_common_segment(
    original_route, target_route, common_segments);

  // Calculate distance through common segments
  accumulated_length += calculate_common_segments_length(
    original_route, common_segments.start_idx_original, common_segments.end_idx_original);

  // Adjust for goal position if it's inside the target terminal lanelet
  accumulated_length = adjust_length_for_goal_position(
    target_route, common_segments.end_idx_target, accumulated_length);

  return accumulated_length;
}

double MissionPlanner::calculate_distance_to_common_segment(
  const LaneletRoute & original_route, const LaneletRoute & target_route,
  const CommonSegments & common_segments) const
{
  const auto current_pose = target_route.start_pose;

  if (common_segments.start_idx_target != 0 && common_segments.start_idx_original > 1) {
    // Calculate from current pose through the lanelet before common segment
    const auto primitives = original_route.segments.at(common_segments.start_idx_original - 1).primitives;
    return calculate_remaining_distance_in_lanelet(current_pose, primitives);
  } else {
    // Calculate from current pose to end of current lanelet
    const auto primitives = original_route.segments.at(common_segments.start_idx_original).primitives;
    return calculate_remaining_distance_in_lanelet(current_pose, primitives);
  }
}

double MissionPlanner::calculate_remaining_distance_in_lanelet(
  const Pose & current_pose, const std::vector<LaneletPrimitive> & primitives) const
{
  lanelet::ConstLanelets lanelets;
  for (const auto & primitive : primitives) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(primitive.id);
    lanelets.push_back(lanelet);
  }

  lanelet::ConstLanelet closest_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(lanelets, current_pose, &closest_lanelet)) {
    RCLCPP_ERROR(get_logger(), "Cannot find the closest lanelet.");
    return 0.0;
  }

  const auto & centerline_2d = lanelet::utils::to2D(closest_lanelet.centerline());
  const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(current_pose.position);
  const auto arc_coordinates = lanelet::geometry::toArcCoordinates(
    centerline_2d, lanelet::utils::to2D(lanelet_point).basicPoint());
  const double dist_to_current_pose = arc_coordinates.length;
  const double lanelet_length = lanelet::utils::getLaneletLength2d(closest_lanelet);

  return lanelet_length - dist_to_current_pose;
}

double MissionPlanner::calculate_common_segments_length(
  const LaneletRoute & route, size_t start_idx, size_t end_idx) const
{
  double length = 0.0;

  for (size_t i = start_idx + 1; i <= end_idx; ++i) {
    const auto primitives = route.segments.at(i).primitives;
    if (primitives.empty()) {
      break;
    }

    std::vector<double> lanelets_length(primitives.size());
    for (size_t primitive_idx = 0; primitive_idx < primitives.size(); ++primitive_idx) {
      const auto & primitive = primitives.at(primitive_idx);
      const auto & lanelet = lanelet_map_ptr_->laneletLayer.get(primitive.id);
      lanelets_length.at(primitive_idx) = lanelet::utils::getLaneletLength2d(lanelet);
    }
    length += *std::min_element(lanelets_length.begin(), lanelets_length.end());
  }

  return length;
}

double MissionPlanner::adjust_length_for_goal_position(
  const LaneletRoute & target_route, size_t end_idx_target, double accumulated_length) const
{
  const auto & target_end_primitives = target_route.segments.at(end_idx_target).primitives;
  const auto & target_goal = target_route.goal_pose;

  for (const auto & target_end_primitive : target_end_primitives) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(target_end_primitive.id);
    if (lanelet::utils::isInLanelet(target_goal, lanelet)) {
      const auto target_goal_position =
        lanelet::utils::conversion::toLaneletPoint(target_goal.position);
      const double dist_to_goal = lanelet::geometry::toArcCoordinates(
        lanelet::utils::to2D(lanelet.centerline()),
        lanelet::utils::to2D(target_goal_position).basicPoint())
        .length;
      const double target_lanelet_length = lanelet::utils::getLaneletLength2d(lanelet);
      const double remaining_dist = target_lanelet_length - dist_to_goal;
      return std::max(accumulated_length - remaining_dist, 0.0);
    }
  }

  return accumulated_length;
}

}  // namespace autoware::mission_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mission_planner::MissionPlanner)
