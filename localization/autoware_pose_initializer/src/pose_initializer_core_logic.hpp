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

#ifndef POSE_INITIALIZER_CORE_LOGIC_HPP_
#define POSE_INITIALIZER_CORE_LOGIC_HPP_

#include "pose_initializer_core_interfaces.hpp"

#include <autoware/component_interface_specs/localization.hpp>

#include <autoware_adapi_v1_msgs/msg/response_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <array>
#include <tuple>
#include <vector>

namespace autoware::pose_initializer
{
struct AutoInitializeOutcome
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  bool reliable{true};
  bool has_gnss_pose_error{false};
  double gnss_error_2d{0.0};
  bool is_gnss_pose_error_small{true};
};

class PoseInitializerCore
{
public:
  PoseInitializerCore(
    GnssProvider * gnss, PoseAligner * ndt, PoseAligner * yabloc,
    PoseErrorChecker * pose_error_check)
  : gnss_(gnss), ndt_(ndt), yabloc_(yabloc), pose_error_check_(pose_error_check)
  {
  }

  geometry_msgs::msg::PoseWithCovarianceStamped get_gnss_pose(
    const std::array<double, 36> & gnss_covariance) const
  {
    using Initialize = autoware::component_interface_specs::localization::Initialize;

    if (gnss_) {
      geometry_msgs::msg::PoseWithCovarianceStamped pose = gnss_->get_pose();
      pose.pose.covariance = gnss_covariance;
      return pose;
    }
    autoware_adapi_v1_msgs::msg::ResponseStatus response_status;
    response_status.success = false;
    response_status.code = Initialize::Service::Response::ERROR_GNSS_SUPPORT;
    response_status.message = "GNSS is not supported.";
    throw response_status;
  }

  template <typename PoseContainer>
  AutoInitializeOutcome compute_auto_initial_pose(
    const PoseContainer & input_pose, const std::array<double, 36> & output_covariance,
    const std::array<double, 36> & gnss_covariance) const
  {
    AutoInitializeOutcome outcome;
    outcome.pose = input_pose.empty() ? get_gnss_pose(gnss_covariance) : input_pose.front();

    if (ndt_) {
      std::tie(outcome.pose, outcome.reliable) = ndt_->align_pose(outcome.pose);
    } else if (yabloc_) {
      // If both the NDT and YabLoc initializer are enabled, prioritize NDT as it offers more
      // accuracy pose.
      std::tie(outcome.pose, outcome.reliable) = yabloc_->align_pose(outcome.pose);
    }

    if (pose_error_check_ && gnss_) {
      const auto latest_gnss_pose = get_gnss_pose(gnss_covariance);
      outcome.has_gnss_pose_error = true;
      outcome.is_gnss_pose_error_small = pose_error_check_->check_pose_error(
        latest_gnss_pose.pose.pose, outcome.pose.pose.pose, outcome.gnss_error_2d);
    }

    outcome.pose.pose.covariance = output_covariance;
    return outcome;
  }

private:
  GnssProvider * gnss_;
  PoseAligner * ndt_;
  PoseAligner * yabloc_;
  PoseErrorChecker * pose_error_check_;
};
}  // namespace autoware::pose_initializer

#endif  // POSE_INITIALIZER_CORE_LOGIC_HPP_
