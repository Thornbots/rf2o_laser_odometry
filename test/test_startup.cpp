#include <gtest/gtest.h>
#include <limits>
#include "rf2o_laser_odometry/CLaserOdometry2DNode.hpp"

namespace
{
class Startup : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override
  {
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
  void tfAt(double time)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9));
    t.header.frame_id = "base_link";
    t.child_frame_id = "laser";
    t.transform.rotation.w = 1.0;
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
    msg->angle_max = msg->angle_min + 719 * msg->angle_increment;
    msg->range_min = 0.1;
    msg->range_max = 12;
    msg->ranges.resize(720, 2.0);
    node->LaserCallBack(msg);
  }
  std::unique_ptr<rf2o::CLaserOdometry2DNode> node;
};

TEST_F(Startup, WaitsForFutureOdomAndTfThenSeedsAtScanStamp)
{
  odom(1.0, 0.0);
  scan(1.5);
  EXPECT_FALSE(node->rf2o_ref.is_initialized());
  odom(2.0, 1.0);
  EXPECT_FALSE(node->rf2o_ref.is_initialized());
  tf();
  node->drainPendingScans();
  ASSERT_TRUE(node->rf2o_ref.is_initialized());
  EXPECT_NEAR(node->rf2o_ref.robot_pose_.translation().x(), 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 1.5);
  EXPECT_TRUE(node->pending_scans.empty());
}

TEST_F(Startup, NeverHoldsOdomEndpoint)
{
  odom(1.0, 0.0);
  tf();
  scan(1.025);
  EXPECT_FALSE(node->rf2o_ref.is_initialized());
  odom(1.05, 0.2);
  ASSERT_TRUE(node->rf2o_ref.is_initialized());
  EXPECT_NEAR(node->rf2o_ref.robot_pose_.translation().x(), 0.1, 1e-8);
}

TEST_F(Startup, CustomInitKeepsExplicitOrigin)
{
  node->init_pose_from_topic = "/custom_origin";
  odom(1.0, 7.0);
  odom(2.0, 8.0);
  tf();
  scan(1.5);
  ASSERT_TRUE(node->rf2o_ref.is_initialized());
  EXPECT_DOUBLE_EQ(node->rf2o_ref.robot_pose_.translation().x(), 7.0);
}

TEST_F(Startup, QueueIsBoundedAndRejectsNonmonotonicScans)
{
  for (int i = 0; i < 200; ++i) scan(1.0 + i * 0.001);
  ASSERT_EQ(node->pending_scans.size(), 100u);
  const auto front = node->pending_scans.front();
  scan(1.199);
  scan(1.15);
  EXPECT_EQ(node->pending_scans.size(), 100u);
  EXPECT_EQ(node->pending_scans.front(), front);
  scan(4.0);
  EXPECT_EQ(node->pending_scans.size(), 1u);
}

TEST_F(Startup, QueuedGeometryChangeCannotAdvanceMatchedReference)
{
  odom(1.0, 0.0);
  scan(1.5);
  scan(1.6);
  node->pending_scans.back()->ranges.resize(360);
  odom(2.0, 1.0);
  tf();
  node->drainPendingScans();
  EXPECT_TRUE(node->pending_scans.empty());
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 1.5);
  EXPECT_DOUBLE_EQ(node->matched_prior.t, 1.5);
}

TEST_F(Startup, WheelFallbackSpansQueueDropsAfterHistoryExpires)
{
  node->confidence_enabled = true;
  odom(1.0, 0.0);
  tf();
  scan(1.0);
  odom(2.0, 1.0);
  odom(4.0, 3.0);
  ASSERT_GT(node->odom_prior_buf.front().t, 1.0);
  scan(4.0);  // First match has no usable previous pyramid: wheel fallback.
  EXPECT_TRUE(node->has_odom_prior);
  EXPECT_NEAR(node->rf2o_ref.robot_pose_.translation().x(), 3.0, 1e-6);
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 4.0);
}

TEST_F(Startup, StalledOdomStopsBlockingOnceANewerScanQueues)
{
  odom(1.0, 0.0);
  tf();
  scan(1.0);
  odom(1.1, 0.1);
  scan(1.1);
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_scan_time_.seconds(), 1.1);
  scan(1.2);  // /odom stops: waits for 1 s of newer scans.
  scan(2.2);
  EXPECT_EQ(node->pending_scans.size(), 2u);
  scan(2.3);
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_scan_time_.seconds(), 1.2);
  EXPECT_FALSE(node->has_odom_prior);
  EXPECT_FALSE(node->have_matched_prior);
  EXPECT_EQ(node->pending_scans.size(), 2u);
}

TEST_F(Startup, StalledTfSkipsOverdueScanAsStale)
{
  node->confidence_enabled = true;
  node->max_extrinsic_age = 0.05;
  odom(1.0, 0.0);
  odom(2.0, 0.0);
  tfAt(1.0);
  scan(1.0);
  ASSERT_TRUE(node->rf2o_ref.is_initialized());
  scan(1.1);
  scan(2.0);
  EXPECT_EQ(node->pending_scans.size(), 2u);
  scan(2.2);  // 1.1 is overdue and its latest extrinsic is 0.1 s old.
  EXPECT_TRUE(node->extrinsic_stale);
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_scan_time_.seconds(), 1.0);
  EXPECT_DOUBLE_EQ(node->matched_prior.t, 1.0);
  EXPECT_EQ(node->pending_scans.size(), 2u);
  odom(2.5, 0.0);
  tfAt(2.0);
  tfAt(2.2);
  node->drainPendingScans();
  EXPECT_FALSE(node->extrinsic_stale);
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_scan_time_.seconds(), 2.2);
  EXPECT_TRUE(node->pending_scans.empty());
}

TEST_F(Startup, RejectsMalformedCustomInitUntilValidPoseArrives)
{
  auto msg = std::make_shared<nav_msgs::msg::Odometry>();
  msg->pose.pose.orientation.w = 1.0;
  msg->pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  node->initPoseCallBack(msg);
  EXPECT_FALSE(node->GT_pose_initialized);
  msg->pose.pose.position.x = 1.0;
  msg->pose.pose.orientation.w = 0.0;
  node->initPoseCallBack(msg);
  EXPECT_FALSE(node->GT_pose_initialized);
  msg->pose.pose.orientation.w = 1.0;
  node->initPoseCallBack(msg);
  EXPECT_TRUE(node->GT_pose_initialized);
}

TEST_F(Startup, NonzeroInitialYawHasNoSpuriousAngularVelocity)
{
  auto msg = std::make_shared<nav_msgs::msg::Odometry>();
  msg->header.stamp = rclcpp::Time(static_cast<int64_t>(1e9));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 1.0);
  msg->pose.pose.orientation = tf2::toMsg(q);
  node->initPoseCallBack(msg);
  node->odomPriorCallBack(msg);
  tf();
  scan(1.0);
  node->rf2o_ref.current_scan_time = rclcpp::Time(static_cast<int64_t>(1.1e9), RCL_ROS_TIME);
  ASSERT_TRUE(node->rf2o_ref.deadReckon(rf2o::Pose3d::Identity()));
  EXPECT_NEAR(node->rf2o_ref.ang_speed, 0.0, 1e-9);
  EXPECT_FALSE(node->rf2o_ref.deadReckon(rf2o::Pose3d::Identity()));
}

TEST_F(Startup, RejectsFloatOverflowInDerivedPrior)
{
  odom(1.0, 0.0);
  tf();
  scan(1.0);
  odom(2.0, 1e40);
  node->rf2o_ref.current_scan_time = rclcpp::Time(static_cast<int64_t>(2e9), RCL_ROS_TIME);
  const auto previous = node->rf2o_ref.kai_loc_old_;
  EXPECT_FALSE(node->setOdomPrior());
  EXPECT_TRUE(node->rf2o_ref.kai_loc_old_.isApprox(previous));
  rf2o::Pose3d increment = rf2o::Pose3d::Identity();
  increment.translation().x() = 1e40;
  const auto pose = node->rf2o_ref.robot_pose_;
  EXPECT_FALSE(node->rf2o_ref.deadReckon(increment));
  EXPECT_TRUE(node->rf2o_ref.robot_pose_.isApprox(pose));
}

TEST_F(Startup, DropsScansOlderThanHistoryAndRejectsReorderedOdom)
{
  scan(1.0);
  odom(2.0, 2.0);
  EXPECT_TRUE(node->pending_scans.empty());
  odom(1.0, 9.0);
  ASSERT_EQ(node->odom_prior_buf.size(), 1u);
  EXPECT_DOUBLE_EQ(node->odom_prior_buf.front().x, 2.0);
}
}  // namespace
