#include "startup_fixture.hpp"

namespace
{
using rf2o_test::Startup;

TEST_F(Startup, RecoveredOdomCanCoverAnUncachedAcceptedScan)
{
  scene(1.0);
  scene(1.05);
  node->have_matched_prior = false;  // Accepted while odom was overdue.
  odom(1.1, 0.01);
  node->rf2o_ref.current_scan_time = rclcpp::Time(int64_t{1100000000}, RCL_ROS_TIME);
  EXPECT_TRUE(node->setOdomPrior());
  EXPECT_NEAR(node->odom_prior_increment.translation().x(), 0.01, 1e-9);
}

TEST_F(Startup, BackwardClockJumpResetsAndResumes)
{
  for (bool references : {false, true})
  {
    SetUp();
    park();
    node->reference_scans = references;
    for (int i = 1; i <= 400; ++i) scene(4.5 + i * 0.05, 0.15, 2.0);
    const auto pose = node->rf2o_ref.robot_pose_;
    scene(1.0, 0.15, 5.0);
    EXPECT_TRUE(node->rf2o_ref.robot_pose_.isApprox(pose, 1e-9));
    // Both delayed old-epoch streams must be rejected before they can
    // poison their high-water marks and trigger a second reset.
    odom(24.5, 2.0);
    scan(24.5);
    scene(1.05, 0.15, 5.0);
    EXPECT_NEAR(node->rf2o_ref.robot_pose_.translation().x(), pose.translation().x(), 0.003);
    EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 1.05);
    EXPECT_TRUE(node->references.empty());
    EXPECT_TRUE(node->have_matched_prior);
    EXPECT_LT(node->odom_prior_buf.back().t, 2.0);
  }
}

TEST_F(Startup, ScanFirstClockJumpWaitsForNewOdom)
{
  park();
  for (int i = 1; i <= 400; ++i) scene(4.5 + i * 0.05, 0.15, 2.0);
  const auto pose = node->rf2o_ref.robot_pose_;
  auto rewound = std::make_shared<sensor_msgs::msg::LaserScan>(node->last_scan);
  rewound->header.stamp = rclcpp::Time(int64_t{1000000000}, RCL_ROS_TIME);
  node->LaserCallBack(rewound);
  EXPECT_FALSE(node->rf2o_ref.is_initialized());
  EXPECT_TRUE(node->odom_prior_buf.empty());
  EXPECT_TRUE(node->references.empty());
  EXPECT_EQ(node->newest_scan_stamp, 1000000000LL);
  tfAt(1.0);
  odom(1.0, 5.0);
  EXPECT_TRUE(node->rf2o_ref.robot_pose_.isApprox(pose, 1e-9));
  odom(24.5, 2.0);
  scan(24.5);
  scene(1.05, 0.15, 5.0);
  EXPECT_NEAR(node->rf2o_ref.robot_pose_.translation().x(), pose.translation().x(), 0.003);
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 1.05);
  EXPECT_TRUE(node->pending_scans.empty());
}

TEST_F(Startup, RewindGuardTracksClockInEitherCallbackOrder)
{
  for (bool clock_first : {false, true})
  {
    SetUp();
    auto clock = node->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock, 24500000000LL), RCL_RET_OK);
    park();
    for (int i = 1; i <= 400; ++i) scene(4.5 + i * 0.05, 0.15, 2.0);
    if (clock_first)
    {
      ASSERT_EQ(rcl_set_ros_time_override(clock, 1000000000LL), RCL_RET_OK);
    }
    scene(1.0, 0.15, 5.0);
    if (!clock_first)
    {
      ASSERT_EQ(rcl_set_ros_time_override(clock, 1000000000LL), RCL_RET_OK);
    }
    scene(1.05, 0.15, 5.0);
    ASSERT_EQ(rcl_set_ros_time_override(clock, 2500000000LL), RCL_RET_OK);
    scene(2.5, 0.15, 5.0);
    EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 2.5);
    const double reset = node->reset_stamp;
    odom(24.5, 2.0);
    scan(24.5);
    scene(2.55, 0.15, 5.0);
    EXPECT_DOUBLE_EQ(node->reset_stamp, reset);
    EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 2.55);
    const auto pose = node->rf2o_ref.robot_pose_;
    scene(0.1, 0.15, 6.0);
    ASSERT_EQ(rcl_set_ros_time_override(clock, 100000000LL), RCL_RET_OK);
    scene(0.15, 0.15, 6.0);
    EXPECT_NEAR(node->rf2o_ref.robot_pose_.translation().x(), pose.translation().x(), 0.003);
    ASSERT_EQ(rcl_set_ros_time_override(clock, 1400000000LL), RCL_RET_OK);
    scene(1.4, 0.15, 6.0);
    EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 1.4);
  }
}

TEST_F(Startup, ClockResetRetainsStaticExtrinsics)
{
  odom(20.0, 0.0);
  tf();
  scan(20.0);
  odom(1.0, 0.0);
  scan(1.0);
  EXPECT_TRUE(node->rf2o_ref.is_initialized());
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 1.0);
}

}  // namespace
