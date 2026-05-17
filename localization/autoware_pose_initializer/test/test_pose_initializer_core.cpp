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
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <chrono>
#include <memory>
#include <vector>

namespace autoware::pose_initializer
{
namespace
{
class RclcppFixture : public ::testing::Test
{
public:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
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

void spin_until_gnss_available(
  GnssModule & gnss, rclcpp::executors::SingleThreadedExecutor & executor)
{
  for (int i = 0; i < 10; ++i) {
    executor.spin_some();
    try {
      (void)gnss.get_pose();
      return;
    } catch (const autoware_adapi_v1_msgs::msg::ResponseStatus &) {
      // Keep spinning until GNSS data is ready.
    }
  }
  FAIL() << "GNSS pose did not arrive in time.";
}
}  // namespace

TEST(PoseInitializerCore, UsesInputPoseAndSetsOutputCovariance)
{
  PoseInitializerCore core(nullptr, nullptr, nullptr, nullptr);

  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> input;
  input.push_back(make_pose(1.0, 2.0));
  const auto output_covariance = make_covariance(0.5);
  const std::array<double, 36> gnss_covariance{};

  const auto outcome =
    core.compute_auto_initial_pose(input, output_covariance, gnss_covariance);

  EXPECT_DOUBLE_EQ(outcome.pose.pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(outcome.pose.pose.pose.position.y, 2.0);
  EXPECT_EQ(outcome.pose.pose.covariance.front(), output_covariance.front());
  EXPECT_EQ(outcome.pose.pose.covariance.back(), output_covariance.back());
  EXPECT_FALSE(outcome.has_gnss_pose_error);
  EXPECT_TRUE(outcome.reliable);
}

TEST_F(RclcppFixture, UsesGnssPoseWhenInputEmpty)
{
  auto options = rclcpp::NodeOptions().append_parameter_override("gnss_pose_timeout", 1.0);
  auto node = std::make_shared<rclcpp::Node>("pose_initializer_core_gnss", options);
  GnssModule gnss(node.get());

  auto publisher = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "gnss_pose_cov", 1);
  auto message = make_pose(3.0, 4.0);
  message.header.stamp = node->now();
  publisher->publish(message);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  spin_until_gnss_available(gnss, executor);

  PoseInitializerCore core(&gnss, nullptr, nullptr, nullptr);
  const auto output_covariance = make_covariance(0.25);
  const auto gnss_covariance = make_covariance(0.75);
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> input;

  const auto outcome =
    core.compute_auto_initial_pose(input, output_covariance, gnss_covariance);

  EXPECT_DOUBLE_EQ(outcome.pose.pose.pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(outcome.pose.pose.pose.position.y, 4.0);
  EXPECT_EQ(outcome.pose.pose.covariance.front(), output_covariance.front());
  EXPECT_TRUE(outcome.reliable);
}

TEST_F(RclcppFixture, ReportsPoseErrorStatusWhenEnabled)
{
  auto options = rclcpp::NodeOptions()
                   .append_parameter_override("gnss_pose_timeout", 1.0)
                   .append_parameter_override("pose_error_threshold", 0.5);
  auto node = std::make_shared<rclcpp::Node>("pose_initializer_core_pose_error", options);
  GnssModule gnss(node.get());
  PoseErrorCheckModule pose_error_check(node.get());

  auto publisher = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "gnss_pose_cov", 1);
  auto message = make_pose(0.0, 0.0);
  message.header.stamp = node->now();
  publisher->publish(message);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  spin_until_gnss_available(gnss, executor);

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
