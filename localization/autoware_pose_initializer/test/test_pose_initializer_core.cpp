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

#include "../src/pose_initializer_core_logic.hpp"

#include <gmock/gmock.h>

#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace autoware::pose_initializer
{
namespace
{
class FakeGnssModule : public GnssProvider
{
public:
  void set_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose) { pose_ = pose; }

  geometry_msgs::msg::PoseWithCovarianceStamped get_pose() override
  {
    if (!pose_) {
      autoware_adapi_v1_msgs::msg::ResponseStatus response_status;
      response_status.success = false;
      response_status.code = autoware::component_interface_specs::localization::Initialize::
        Service::Response::ERROR_GNSS;
      response_status.message = "The GNSS pose has not arrived.";
      throw response_status;
    }
    return *pose_;
  }

private:
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> pose_;
};

class FakePoseErrorCheckModule : public PoseErrorChecker
{
public:
  explicit FakePoseErrorCheckModule(double threshold) : threshold_(threshold) {}

  bool check_pose_error(
    const geometry_msgs::msg::Pose & reference_pose, const geometry_msgs::msg::Pose & result_pose,
    double & error_2d) override
  {
    const double diff_x = reference_pose.position.x - result_pose.position.x;
    const double diff_y = reference_pose.position.y - result_pose.position.y;
    error_2d = std::sqrt(diff_x * diff_x + diff_y * diff_y);
    return error_2d < threshold_;
  }

private:
  double threshold_{0.0};
};

geometry_msgs::msg::PoseWithCovarianceStamped make_pose(double x, double y)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;
  return pose;
}

std::array<double, 36> make_covariance(double seed)
{
  std::array<double, 36> covariance{};
  covariance.front() = seed;
  covariance.back() = seed + 1.0;
  return covariance;
}

}  // namespace

TEST(PoseInitializerCore, UsesInputPoseAndSetsOutputCovariance)
{
  PoseInitializerCore core(nullptr, nullptr, nullptr, nullptr);

  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> input;
  input.push_back(make_pose(1.0, 2.0));
  const auto output_covariance = make_covariance(0.5);
  const std::array<double, 36> gnss_covariance{};

  const auto outcome = core.compute_auto_initial_pose(input, output_covariance, gnss_covariance);

  EXPECT_DOUBLE_EQ(outcome.pose.pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(outcome.pose.pose.pose.position.y, 2.0);
  EXPECT_EQ(outcome.pose.pose.covariance.front(), output_covariance.front());
  EXPECT_EQ(outcome.pose.pose.covariance.back(), output_covariance.back());
  EXPECT_FALSE(outcome.has_gnss_pose_error);
  EXPECT_TRUE(outcome.reliable);
}

TEST(PoseInitializerCore, UsesGnssPoseWhenInputEmpty)
{
  FakeGnssModule gnss;
  gnss.set_pose(make_pose(3.0, 4.0));

  PoseInitializerCore core(&gnss, nullptr, nullptr, nullptr);
  const auto output_covariance = make_covariance(0.25);
  const auto gnss_covariance = make_covariance(0.75);
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> input;

  const auto outcome = core.compute_auto_initial_pose(input, output_covariance, gnss_covariance);

  EXPECT_DOUBLE_EQ(outcome.pose.pose.pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(outcome.pose.pose.pose.position.y, 4.0);
  EXPECT_EQ(outcome.pose.pose.covariance.front(), output_covariance.front());
  EXPECT_TRUE(outcome.reliable);
}

TEST(PoseInitializerCore, ReportsPoseErrorStatusWhenEnabled)
{
  FakeGnssModule gnss;
  gnss.set_pose(make_pose(0.0, 0.0));
  FakePoseErrorCheckModule pose_error_check(0.5);

  PoseInitializerCore core(&gnss, nullptr, nullptr, &pose_error_check);
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> input;
  input.push_back(make_pose(10.0, 0.0));

  const auto outcome =
    core.compute_auto_initial_pose(input, make_covariance(1.0), make_covariance(2.0));

  EXPECT_TRUE(outcome.has_gnss_pose_error);
  EXPECT_FALSE(outcome.is_gnss_pose_error_small);
  EXPECT_GT(outcome.gnss_error_2d, 0.5);
}
}  // namespace autoware::pose_initializer
