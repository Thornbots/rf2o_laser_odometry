#pragma once

#include <gtest/gtest.h>
#include <rcl/time.h>
#include <limits>
#include <fstream>
#include "rf2o_laser_odometry/CLaserOdometry2DNode.hpp"

namespace rf2o_test
{
constexpr float kAngleMin = -M_PI;
constexpr float kAngleIncrement = 2 * M_PI / 720;
constexpr float kAngleSpan = 719 * kAngleIncrement;
constexpr float kAngleMax = kAngleMin + kAngleSpan;
class Startup : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override
  {
    node.reset();
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"init_pose_from_topic", "/odom"},
                                 {"odom_prior_topic", "/odom"}});
    node = std::make_unique<rf2o::CLaserOdometry2DNode>(options);
  }
  void tf()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "base_link";
    t.child_frame_id = "laser";
    t.transform.rotation.w = 1.0;
    ASSERT_TRUE(node->buffer_->setTransform(t, "test", true));
  }
  void tfAt(double time, double yaw = 0.0)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9));
    t.header.frame_id = "base_link";
    t.child_frame_id = "laser";
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    t.transform.rotation = tf2::toMsg(q);
    t.transform.translation.x = 0.22 * std::cos(yaw);
    t.transform.translation.y = 0.22 * std::sin(yaw);
    ASSERT_TRUE(node->buffer_->setTransform(t, "test", false));
  }
  void odom(double time, double x)
  {
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    msg->header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9));
    msg->pose.pose.position.x = x;
    msg->pose.pose.orientation.w = 1.0;
    node->initPoseCallBack(msg);
    node->odomPriorCallBack(msg);
  }
  void scan(double time)
  {
    auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    msg->header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9));
    msg->header.frame_id = "laser";
    msg->angle_min = -M_PI;
    msg->angle_increment = 2 * M_PI / 720;
    msg->angle_max = kAngleMax;
    msg->range_min = 0.1;
    msg->range_max = 12;
    msg->ranges.resize(720, 2.0);
    node->LaserCallBack(msg);
  }
  void referencesOn()
  {
    node->reference_scans = true;
    node->confidence_enabled = true;
    node->rf2o_ref.fixed_heading = true;
    node->rf2o_ref.max_speed = 6.0;
  }
  void scene(double t, double x = 0.0, double wheel = 0.0,
             double head = 0.0, bool obstacle = false, bool prior = true, bool real_format = false)
  {
    if (prior) odom(t, wheel);
    tfAt(t, head);
    auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    msg->header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
    msg->header.frame_id = "laser";
    msg->angle_min = -M_PI;
    // A2M8 angle compensation includes both -pi and +pi endpoints.
    msg->angle_increment = 2 * M_PI / (real_format ? 719 : 720);
    // Keep the angular grid identical across compiler inlining/FMA choices.
    msg->angle_max = real_format ? static_cast<float>(M_PI)
                                 : kAngleMax;
    msg->range_min = 0.1;
    msg->range_max = 12;
    const double hx = x + 0.22 * std::cos(head), hy = 0.22 * std::sin(head);
    uint32_t seed = static_cast<uint32_t>(t * 1000) + 1;
    for (int j = 0; j < 720; ++j)
    {
      const double a = msg->angle_min + j * msg->angle_increment + head;
      const double c = std::cos(a), sn = std::sin(a);
      double r = std::numeric_limits<double>::infinity();
      if (std::abs(c) > 1e-9) r = std::min(r, ((c > 0 ? 4.0 : -4.0) - hx) / c);
      if (std::abs(sn) > 1e-9) r = std::min(r, ((sn > 0 ? 4.0 : -4.0) - hy) / sn);
      if (obstacle && std::abs(a) < 0.2) r = std::min(r, 1.0 / c);
      seed = seed * 1664525u + 1013904223u;
      const double noise = real_format
          ? 0.005 * (2.0 * (seed >> 8) / static_cast<double>(1u << 24) - 1.0)
          : 0.003 * std::sin(j * 13.7 + t * 0.83);
      msg->ranges.push_back(r + noise);
    }
    node->LaserCallBack(msg);
  }
  void park(bool obstacle = false, bool real_format = false)
  {
    referencesOn();
    for (int i = 0; i <= 70; ++i) scene(1.0 + i * 0.05, 0, 0, 0, obstacle, true, real_format);
    ASSERT_FALSE(node->references.empty());
  }
  std::unique_ptr<rf2o::CLaserOdometry2DNode> node;
};

}  // namespace rf2o_test
