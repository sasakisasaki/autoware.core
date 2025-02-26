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

#ifndef AUTOWARE__PATH_GENERATOR__UTILS_HPP_
#define AUTOWARE__PATH_GENERATOR__UTILS_HPP_

#include "autoware/path_generator/common_structs.hpp"

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <optional>
#include <utility>
#include <vector>

namespace autoware::path_generator
{
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;

namespace utils
{
/**
 * @brief get lanelets within route that are in specified distance forward or backward from
 * current position
 * @param lanelet lanelet where ego vehicle is on
 * @param planner_data planner data
 * @param current_pose current pose of ego vehicle
 * @param backward_distance backward distance from ego vehicle
 * @param forward_distance forward distance from ego vehicle
 * @return lanelets in range (std::nullopt if target lanelet is not within route)
 */
std::optional<lanelet::ConstLanelets> get_lanelets_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data,
  const geometry_msgs::msg::Pose & current_pose, const double backward_distance,
  const double forward_distance);

/**
 * @brief get lanelets within route that are in specified distance backward from target
 * lanelet
 * @param lanelet target lanelet
 * @param planner_data planner data
 * @param distance backward distance from beginning of target lanelet
 * @return lanelets in range (std::nullopt if target lanelet is not within route)
 */
std::optional<lanelet::ConstLanelets> get_lanelets_within_route_up_to(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data, const double distance);

/**
 * @brief get lanelets within route that are in specified distance forward from target
 * lanelet
 * @param lanelet target lanelet
 * @param planner_data planner data
 * @param distance forward distance from end of target lanelet
 * @return lanelets in range (std::nullopt if target lanelet is not within route)
 */
std::optional<lanelet::ConstLanelets> get_lanelets_within_route_after(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data, const double distance);

/**
 * @brief get previous lanelet within route
 * @param lanelet target lanelet
 * @param planner_data planner data
 * @return lanelets in range (std::nullopt if previous lanelet is not found or not
 * within route)
 */
std::optional<lanelet::ConstLanelet> get_previous_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data);

/**
 * @brief get next lanelet within route
 * @param lanelet target lanelet
 * @param planner_data planner data
 * @return lanelets in range (std::nullopt if next lanelet is not found or not
 * within route)
 */
std::optional<lanelet::ConstLanelet> get_next_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data);

/**
 * @brief get waypoints in lanelet sequence and group them
 * @param lanelet_sequence lanelet sequence
 * @param lanelet_map lanelet map to get waypoints
 * @param group_separation_threshold maximum distance between waypoints to belong to same
 * group (see figure in README)
 * @param interval_margin_ratio ratio to expand interval bound of group according to the
 * lateral distance of first and last point of group
 * @return waypoint groups (each group is a pair of points and its interval)
 */
std::vector<std::pair<lanelet::ConstPoints3d, std::pair<double, double>>> get_waypoint_groups(
  const lanelet::LaneletSequence & lanelet_sequence, const lanelet::LaneletMap & lanelet_map,
  const double group_separation_threshold, const double interval_margin_ratio);

/**
 * @brief get bound of path cropped within specified range
 * @param lanelet_bound original bound of lanelet
 * @param lanelet_centerline centerline of lanelet
 * @param s_start longitudinal distance of start of bound
 * @param s_end longitudinal distance of end of bound
 * @return cropped bound
 */
std::vector<geometry_msgs::msg::Point> get_path_bound(
  const lanelet::CompoundLineString2d & lanelet_bound,
  const lanelet::CompoundLineString2d & lanelet_centerline, const double s_start,
  const double s_end);

/**
 * @brief Modify the path points near the goal to smoothly connect the input path and the goal
 * point
 * @details Remove the path points that are forward from the goal by the distance of
 * search_radius_range. Then insert the goal into the path. The previous goal point generated
 * from the goal posture information is also inserted for the smooth connection of the goal pose.
 * @param [in] search_radius_range distance on path to be modified for goal insertion
 * @param [in] search_rad_range [unused]
 * @param [in] input original path
 * @param [in] goal original goal pose
 * @param [in] goal_lane_id [unused]
 * @param [in] output_ptr output path with modified points for the goal
 */
bool set_goal(
  const double search_radius_range, const double search_rad_range, const PathWithLaneId & input,
  const geometry_msgs::msg::Pose & goal, const int64_t goal_lane_id, PathWithLaneId * output_ptr);

/**
 * @brief Recreate the goal pose to prevent the goal point being too far from the lanelet, which
 *  causes the path to twist near the goal.
 * @details Return the goal point projected on the straight line of the segment of lanelet
 *  closest to the original goal.
 * @param [in] goal original goal pose
 * @param [in] goal_lanelet lanelet containing the goal pose
 */
const geometry_msgs::msg::Pose refine_goal(const geometry_msgs::msg::Pose & goal, const lanelet::ConstLanelet & goal_lanelet);

/**
 * @brief Recreate the path with a given goal pose.
 * @param search_radius_range Searching radius.
 * @param search_rad_range Searching angle.
 * @param input Input path.
 * @param goal Goal pose.
 * @param goal_lane_id Lane ID of goal lanelet.
 * @return Recreated path
 */
PathWithLaneId refine_path_for_goal(
  const double search_radius_range, const double search_rad_range, const PathWithLaneId & input,
  const geometry_msgs::msg::Pose & goal, const int64_t goal_lane_id);

/**
 * @brief Extract lanelets from the path.
 * @param path Input path.
 * @param planner_data Planner data.
 * @return Extracted lanelets
 */
lanelet::ConstLanelets extract_lanelets_from_path(
  const PathWithLaneId & refined_path, const std::shared_ptr<const PlannerData> & planner_data);

/**
 * @brief Get the goal lanelet.
 * @param planner_data Planner data.
 * @param goal_lanelet Goal lanelet.
 * @return True if the goal lanelet is found, false otherwise
 */
bool get_goal_lanelet(const PlannerData & planner_data, lanelet::ConstLanelet * goal_lanelet);

/**
 * @brief Check if the pose is in the lanelets.
 * @param pose Pose.
 * @param lanes Lanelets.
 * @return True if the pose is in the lanelets, false otherwise
 */
bool is_in_lanelets(const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & lanes);

/**
 * @brief Check if the path is valid.
 * @param refined_path Input path.
 * @param planner_data Planner data.
 * @return True if the path is valid, false otherwise
 */
bool is_path_valid(
  const PathWithLaneId & refined_path, const std::shared_ptr<const PlannerData> & planner_data);

/**
 * @brief Modify the path to connect smoothly to the goal.
 * @param path Input path.
 * @param planner_data Planner data.
 * @return Modified path
 */
PathWithLaneId modify_path_for_smooth_goal_connection(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data_banana
);

}  // namespace utils
}  // namespace autoware::path_generator

#endif  // AUTOWARE__PATH_GENERATOR__UTILS_HPP_
