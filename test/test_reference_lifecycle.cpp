#include "startup_fixture.hpp"

namespace
{
using rf2o_test::Startup;

TEST_F(Startup, ReferenceParametersRejectInvalidValues)
{
  for (const std::string name : {"reference_max_offset",
                                "reference_still_distance", "reference_still_time", "reference_max_age"})
    for (double value : {0.0, -0.01, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()})
    {
      rclcpp::NodeOptions options;
      options.parameter_overrides({rclcpp::Parameter(name, value)});
      EXPECT_THROW(rf2o::CLaserOdometry2DNode invalid(options), std::invalid_argument);
    }

}

TEST_F(Startup, AnchorsRequireExactStampedExtrinsics)
{
  park();
  const auto stamp = node->references.front().scan.stamp;
  node->rf2o_ref.current_scan_time = stamp + rclcpp::Duration::from_seconds(7.0);
  node->extrinsic_age = 0.01;
  node->updateReferences(rf2o::CLaserOdometry2DNode::Tier::GOOD, &node->references.front());
  ASSERT_EQ(node->references.size(), 1u);
  EXPECT_EQ(node->references.front().scan.stamp, stamp);
  node->references.clear();
  node->updateReferences(rf2o::CLaserOdometry2DNode::Tier::GOOD, nullptr);
  EXPECT_TRUE(node->references.empty());
}

TEST_F(Startup, NonRepeatingHeadAnglesHaveABoundedReferenceBank)
{
  referencesOn();
  size_t largest = 0;
  for (int i = 0; i <= 400; ++i)
  {
    scene(1.0 + i * 0.05, 0, 0, i * 0.011);
    largest = std::max(largest, node->references.size());
    ASSERT_LE(node->references.size(), 1u);
  }
  EXPECT_GT(largest, 0u);
  EXPECT_TRUE(node->rf2o_ref.robot_pose_.matrix().allFinite());
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 21.0);
}

TEST_F(Startup, InterruptedScansCannotUseAnExpiredAnchor)
{
  park();
  ASSERT_NE(node->findReference(), nullptr);
  node->rf2o_ref.current_scan_time = rclcpp::Time(int64_t{20000000000}, RCL_ROS_TIME);
  EXPECT_EQ(node->findReference(), nullptr);
}

TEST_F(Startup, AnchorExpiresAndDegradedMatchDropsIt)
{
  park();
  const auto stamp = node->references.front().scan.stamp;
  for (int i = 1; i <= 180; ++i) scene(4.5 + i * 0.05);
  ASSERT_EQ(node->references.size(), 1u);
  EXPECT_GT(node->references.front().scan.stamp, stamp);
  EXPECT_LE(node->rf2o_ref.current_scan_time.seconds() -
            node->references.front().scan.stamp.seconds(), node->reference_max_age);
  node->updateReferences(rf2o::CLaserOdometry2DNode::Tier::DEGRADED, &node->references.front());
  EXPECT_TRUE(node->references.empty());
}

TEST_F(Startup, PriorLossClearsAnchorsAndRecoversWithMovingHead)
{
  park();
  for (int i = 1; i <= 40; ++i)
    scene(4.5 + i * 0.05, i * 0.0005, 0, i * 0.002, false, false);
  EXPECT_FALSE(node->has_odom_prior);
  EXPECT_TRUE(node->references.empty());
  for (int i = 41; i <= 80; ++i)
    scene(4.5 + i * 0.05, i * 0.0005, i * 0.0005, i * 0.002);
  EXPECT_TRUE(node->has_odom_prior);
  EXPECT_NEAR(node->rf2o_ref.robot_pose_.translation().x(), 0.04, 0.012);
  EXPECT_GT(node->rf2o_ref.last_odom_time.seconds(), 8.0);
}

TEST_F(Startup, AnchorSeedRequiresFiniteCoveredPrior)
{
  tf();
  odom(1.0, 0.0);
  scan(1.0);
  rf2o::CLaserOdometry2DNode::Reference ref{
      node->rf2o_ref.currentReference(), {1.0, 0.0, 0.0, 0.0}, Eigen::Vector2d::Zero()};
  node->rf2o_ref.current_scan_time = rclcpp::Time(int64_t{2000000000}, RCL_ROS_TIME);
  const auto before = node->rf2o_ref.kai_loc_old_;
  EXPECT_FALSE(node->seedFromReference(ref));
  node->has_odom_prior = true;
  EXPECT_FALSE(node->seedFromReference(ref));
  odom(2.0, 0.01);
  ref.odom.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(node->seedFromReference(ref));
  EXPECT_TRUE(node->rf2o_ref.kai_loc_old_.isApprox(before));
  ref.odom.x = 0.0;
  EXPECT_TRUE(node->seedFromReference(ref));
}

}  // namespace
