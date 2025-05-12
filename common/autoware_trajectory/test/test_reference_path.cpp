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

#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/reference_path.hpp"
#include "autoware/trajectory/threshold.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/topology.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_io/Io.h>

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace autoware::experimental
{

TEST(reference_path, NormalMap)
{
  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map/straight_waypoint";
  const auto map_path = sample_map_dir / "lanelet2_map.osm";
  const auto lanelet_map_ptr = lanelet2_utils::load_mgrs_coordinate_map(map_path.string());
  const auto [routing_graph, traffic_rules] =
    autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
      lanelet_map_ptr);

  const std::vector<lanelet::Id> ids = {1043, 1047, 1049};
  const auto lanelet_sequence = ids | ranges::views::transform([&](const auto & id) {
                                  return lanelet_map_ptr->laneletLayer.get(id);
                                }) |
                                ranges::to<std::vector>();
  const auto ego_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(autoware_utils_geometry::create_point(102, 101, 100.0))
      .orientation(autoware_utils_geometry::create_quaternion(0.0, 0.0, 1.0, 0.0));
  const auto reference_path_opt = trajectory::build_reference_path(
    lanelet_sequence, lanelet_sequence.front(), ego_pose, lanelet_map_ptr, routing_graph,
    traffic_rules, 1.0, 10.0, 200, 50);

  ASSERT_EQ(reference_path_opt.has_value(), true);
  const auto & reference_path = reference_path_opt.value();
  const auto points = reference_path.restore();
  const auto lanelet = lanelet::utils::combineLaneletsShape(lanelet_sequence);

  for (const auto [p1, p2] : ranges::views::zip(points, points | ranges::views::drop(1))) {
    EXPECT_EQ(
      autoware_utils_geometry::calc_distance3d(p1, p2) >=
        autoware::experimental::trajectory::k_points_minimum_dist_threshold,
      true);
    EXPECT_EQ(
      boost::geometry::within(
        lanelet::utils::to2D(lanelet::utils::conversion::toLaneletPoint(p1.point.pose.position)),
        lanelet.polygon2d().basicPolygon()),
      true);
    EXPECT_EQ(
      std::fabs(autoware_utils_geometry::calc_azimuth_angle(
        p1.point.pose.position, p2.point.pose.position)) < M_PI / 2.0,
      true);
  }
}

TEST(reference_path, DenseMap)
{
  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map/dense_centerline";
  const auto map_path = sample_map_dir / "lanelet2_map.osm";
  const auto lanelet_map_ptr = lanelet2_utils::load_mgrs_coordinate_map(map_path.string());
  const auto [routing_graph, traffic_rules] =
    autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
      lanelet_map_ptr);

  const std::vector<lanelet::Id> ids = {140, 137, 136, 138, 139, 135};
  const auto lanelet_sequence = ids | ranges::views::transform([&](const auto & id) {
                                  return lanelet_map_ptr->laneletLayer.get(id);
                                }) |
                                ranges::to<std::vector>();
  const auto ego_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(autoware_utils_geometry::create_point(127, 96, 100.0))
      .orientation(autoware_utils_geometry::create_quaternion(0.0, 0.0, 1.0, 0.0));
  const auto reference_path_opt = trajectory::build_reference_path(
    lanelet_sequence, lanelet_sequence.front(), ego_pose, lanelet_map_ptr, routing_graph,
    traffic_rules, 1.0, 10.0, 200, 50);

  ASSERT_EQ(reference_path_opt.has_value(), true);
  const auto & reference_path = reference_path_opt.value();
  const auto points = reference_path.restore();
  const auto lanelet = lanelet::utils::combineLaneletsShape(lanelet_sequence);

  for (const auto [p1, p2] : ranges::views::zip(points, points | ranges::views::drop(1))) {
    EXPECT_EQ(
      autoware_utils_geometry::calc_distance3d(p1, p2) >=
        autoware::experimental::trajectory::k_points_minimum_dist_threshold,
      true);
    EXPECT_EQ(
      boost::geometry::within(
        lanelet::utils::to2D(lanelet::utils::conversion::toLaneletPoint(p1.point.pose.position)),
        lanelet.polygon2d().basicPolygon()),
      true);
    EXPECT_EQ(
      std::fabs(autoware_utils_geometry::calc_azimuth_angle(
        p1.point.pose.position, p2.point.pose.position)) < M_PI / 2.0,
      true);
  }
}
}  // namespace autoware::experimental
