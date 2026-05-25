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

#ifndef POSE_INITIALIZER_CORE_INTERFACES_HPP_
#define POSE_INITIALIZER_CORE_INTERFACES_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tuple>

namespace autoware::pose_initializer
{
class GnssProvider
{
public:
  virtual ~GnssProvider() = default;
  virtual geometry_msgs::msg::PoseWithCovarianceStamped get_pose() = 0;
};

class PoseAligner
{
public:
  virtual ~PoseAligner() = default;
  virtual std::tuple<geometry_msgs::msg::PoseWithCovarianceStamped, bool> align_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose) = 0;
};

class PoseErrorChecker
{
public:
  virtual ~PoseErrorChecker() = default;
  virtual bool check_pose_error(
    const geometry_msgs::msg::Pose & reference_pose, const geometry_msgs::msg::Pose & result_pose,
    double & error_2d) = 0;
};
}  // namespace autoware::pose_initializer

#endif  // POSE_INITIALIZER_CORE_INTERFACES_HPP_
