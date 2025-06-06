// Copyright 2024 The Autoware Contributors
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

#include "localization_module.hpp"

#include <autoware/component_interface_specs/localization.hpp>

#include <autoware_adapi_v1_msgs/msg/response_status.hpp>

#include <memory>
#include <string>
#include <tuple>

namespace autoware::pose_initializer
{
using Initialize = autoware::component_interface_specs::localization::Initialize;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

LocalizationModule::LocalizationModule(rclcpp::Node * node, const std::string & service_name)
: logger_(node->get_logger()), cli_align_(node->create_client<RequestPoseAlignment>(service_name))
{
}

std::tuple<PoseWithCovarianceStamped, bool> LocalizationModule::align_pose(
  const PoseWithCovarianceStamped & pose)
{
  const auto req = std::make_shared<RequestPoseAlignment::Request>();
  req->pose_with_covariance = pose;

  if (!cli_align_->service_is_ready()) {
    autoware_adapi_v1_msgs::msg::ResponseStatus respose_status;
    respose_status.success = false;
    respose_status.code = autoware_adapi_v1_msgs::msg::ResponseStatus::SERVICE_UNREADY;
    respose_status.message = "align server is not ready.";
    throw respose_status;
  }

  RCLCPP_INFO(logger_, "Call align server.");
  const auto res = cli_align_->async_send_request(req).get();
  if (!res->success) {
    autoware_adapi_v1_msgs::msg::ResponseStatus respose_status;
    respose_status.success = false;
    respose_status.code = Initialize::Service::Response::ERROR_ESTIMATION;
    respose_status.message = "align server failed.";
    throw respose_status;
  }
  RCLCPP_INFO(logger_, "align server succeeded.");

  // Overwrite the covariance.
  return std::make_tuple(res->pose_with_covariance, res->reliable);
}
}  // namespace autoware::pose_initializer
