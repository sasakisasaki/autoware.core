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

#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/reference_path.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/pyplot/pyplot.hpp>
#include <autoware_test_utils/visualization.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace autoware::experimental;  // NOLINT

int main()
{
  pybind11::scoped_interpreter guard{};
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(2, 2);

  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_test_utils")) /
    "test_map/overlap";
  const auto intersection_crossing_map_path = sample_map_dir / "lanelet2_map.osm";

  const auto lanelet_map_ptr =
    lanelet2_utils::load_mgrs_coordinate_map(intersection_crossing_map_path.string());
  const auto [routing_graph, traffic_rules] =
    lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_ptr);

  const std::vector<lanelet::Id> ids = {609, 610, 612, 611, 613};
  const auto lanelet_sequence = ids | ranges::views::transform([&](const auto & id) {
                                  return lanelet_map_ptr->laneletLayer.get(id);
                                }) |
                                ranges::to<std::vector>();
  const auto ego_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(autoware_utils_geometry::create_point(1573.68, 389.857, 100.0))
      .orientation(autoware_utils_geometry::create_quaternion(0.0, 0.0, 0.999997, 0.00250111));

  {
    auto & ax = axes[0];
    const auto reference_path_opt = trajectory::build_reference_path(
      lanelet_sequence, lanelet_sequence.front(), ego_pose, lanelet_map_ptr, routing_graph,
      traffic_rules, 1.0, 10.0, 500, 0);
    if (!reference_path_opt) {
      return 0;
    }
    const auto & reference_path = reference_path_opt.value();
    autoware_internal_planning_msgs::msg::PathWithLaneId path;
    path.points = reference_path.restore();
    autoware::test_utils::plot_autoware_object(path, ax);
    for (const auto & route_lanelet : lanelet_sequence) {
      autoware::test_utils::plot_lanelet2_object(route_lanelet, ax);
    }
    ax.scatter(Args(ego_pose.position.x, ego_pose.position.y), Kwargs("label"_a = "ego position"));
    ax.set_aspect(Args("equal"));
    ax.set_title(Args("forward = 500, backward = 0"));
    ax.legend();
    ax.grid();
  }
  {
    auto & ax = axes[1];
    const auto reference_path_opt = trajectory::build_reference_path(
      lanelet_sequence, lanelet_sequence.front(), ego_pose, lanelet_map_ptr, routing_graph,
      traffic_rules, 1.0, 10.0, 0, 100);
    if (!reference_path_opt) {
      return 0;
    }
    const auto & reference_path = reference_path_opt.value();
    autoware_internal_planning_msgs::msg::PathWithLaneId path;
    path.points = reference_path.restore();
    autoware::test_utils::plot_autoware_object(path, ax);
    for (const auto & route_lanelet : lanelet_sequence) {
      autoware::test_utils::plot_lanelet2_object(route_lanelet, ax);
    }
    ax.scatter(Args(ego_pose.position.x, ego_pose.position.y), Kwargs("label"_a = "ego position"));
    ax.set_aspect(Args("equal"));
    ax.set_title(Args("forward = 0, backward = 100"));
    ax.legend();
    ax.grid();
  }
  {
    auto & ax = axes[2];
    const auto reference_path_opt = trajectory::build_reference_path(
      lanelet_sequence, lanelet_sequence.front(), ego_pose, lanelet_map_ptr, routing_graph,
      traffic_rules, 1.0, 10.0, 50, 10);
    if (!reference_path_opt) {
      return 0;
    }
    const auto & reference_path = reference_path_opt.value();
    autoware_internal_planning_msgs::msg::PathWithLaneId path;
    path.points = reference_path.restore();
    autoware::test_utils::plot_autoware_object(path, ax);
    for (const auto & route_lanelet : lanelet_sequence) {
      autoware::test_utils::plot_lanelet2_object(route_lanelet, ax);
    }
    ax.scatter(Args(ego_pose.position.x, ego_pose.position.y), Kwargs("label"_a = "ego position"));
    ax.set_aspect(Args("equal"));
    ax.set_title(Args("forward = 50, backward = 10"));
    ax.legend();
    ax.grid();
  }
  {
    auto & ax = axes[3];
    const auto reference_path_opt = trajectory::build_reference_path(
      lanelet_sequence, lanelet_sequence.front(), ego_pose, lanelet_map_ptr, routing_graph,
      traffic_rules, 1.0, 10.0, 0, 0);
    if (reference_path_opt) {
      const auto & reference_path = reference_path_opt.value();
      autoware_internal_planning_msgs::msg::PathWithLaneId path;
      path.points = reference_path.restore();
      autoware::test_utils::plot_autoware_object(path, ax);
    }
    for (const auto & route_lanelet : lanelet_sequence) {
      autoware::test_utils::plot_lanelet2_object(route_lanelet, ax);
    }
    ax.scatter(Args(ego_pose.position.x, ego_pose.position.y), Kwargs("label"_a = "ego position"));
    ax.set_aspect(Args("equal"));
    ax.set_title(Args("forward = 0, backward = 0 (reference path is not created)"));
    ax.legend();
    ax.grid();
  }
  plt.show();

  return 0;
}
