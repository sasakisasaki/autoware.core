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

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <memory>
#include <sstream>
#include <string>
#include <utility>

namespace autoware::experimental::lanelet2_utils
{

lanelet::LaneletMapConstPtr load_mgrs_coordinate_map(
  const std::string & path, const double centerline_resolution)
{
  lanelet::ErrorMessages errors{};
  lanelet::projection::MGRSProjector projector;
  auto lanelet_map_ptr_mut = lanelet::load(path, projector, &errors);

  for (auto & lanelet_obj : lanelet_map_ptr_mut->laneletLayer) {
    if (lanelet_obj.hasCustomCenterline()) {
      const auto & centerline = lanelet_obj.centerline();
      lanelet_obj.setAttribute("waypoints", centerline.id());
    }
    const auto fine_center_line =
      lanelet::utils::generateFineCenterline(lanelet_obj, centerline_resolution);
    lanelet_obj.setCenterline(fine_center_line);
  }
  return lanelet::LaneletMapConstPtr{std::move(lanelet_map_ptr_mut)};
}

std::pair<lanelet::routing::RoutingGraphConstPtr, lanelet::traffic_rules::TrafficRulesPtr>
instantiate_routing_graph_and_traffic_rules(
  lanelet::LaneletMapConstPtr lanelet_map, const char * location, const char * participant)
{
  auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(location, participant);
  return {
    lanelet::routing::RoutingGraph::build(*lanelet_map, *traffic_rules), std::move(traffic_rules)};
}

autoware_map_msgs::msg::LaneletMapBin to_autoware_map_msgs(const lanelet::LaneletMapConstPtr & map)
{
  const lanelet::LaneletMapPtr map_mut = std::const_pointer_cast<lanelet::LaneletMap>(map);

  std::stringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa << *map_mut;
  auto id_counter = lanelet::utils::getId();
  oa << id_counter;

  std::string data_str(ss.str());

  autoware_map_msgs::msg::LaneletMapBin msg;
  msg.data.clear();
  msg.data.assign(data_str.begin(), data_str.end());
  return msg;
}

lanelet::LaneletMapConstPtr from_autoware_map_msgs(
  const autoware_map_msgs::msg::LaneletMapBin & msg)
{
  std::string data_str;
  data_str.assign(msg.data.begin(), msg.data.end());

  std::stringstream ss;
  ss << data_str;
  boost::archive::binary_iarchive oa(ss);

  auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  oa >> *lanelet_map_ptr;
  lanelet::Id id_counter = 0;
  oa >> id_counter;
  lanelet::utils::registerId(id_counter);

  return lanelet_map_ptr;
}

geometry_msgs::msg::Point to_ros(const lanelet::BasicPoint3d & src)
{
  geometry_msgs::msg::Point dst;
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
  return dst;
}

geometry_msgs::msg::Point to_ros(const lanelet::ConstPoint3d & src)
{
  return to_ros(src.basicPoint());
}

geometry_msgs::msg::Point to_ros(const lanelet::BasicPoint2d & src, const double & z)
{
  geometry_msgs::msg::Point dst;
  dst.x = src.x();
  dst.y = src.y();
  dst.z = z;
  return dst;
}

geometry_msgs::msg::Point to_ros(const lanelet::ConstPoint2d & src, const double & z)
{
  return to_ros(src.basicPoint(), z);
}

lanelet::ConstPoint3d from_ros(const geometry_msgs::msg::Point & src)
{
  return lanelet::ConstPoint3d(lanelet::Point3d(lanelet::InvalId, src.x, src.y, src.z));
}

lanelet::ConstPoint3d from_ros(const geometry_msgs::msg::Pose & src)
{
  return from_ros(src.position);
}

}  // namespace autoware::experimental::lanelet2_utils
