#include "startup_fixture.hpp"

namespace
{
using rf2o_test::Startup;

TEST_F(Startup, ReferenceEntryBumpAndWheelCreepExit)
{
  park();
  for (int i = 1; i <= 5; ++i) scene(4.5 + i * 0.05, 0.03);
  EXPECT_NEAR(node->rf2o_ref.robot_pose_.translation().x(), 0.03, 0.01);
  EXPECT_LT(node->rf2o_ref.getMatchQuality().speed, 1.0);
  for (int i = 1; i <= 20; ++i) scene(4.75 + i * 0.05, 0.03 + i * 0.001, i * 0.001);
  EXPECT_TRUE(node->references.empty());
  EXPECT_NEAR(node->rf2o_ref.robot_pose_.translation().x(), 0.05, 0.012);
}

TEST_F(Startup, LongWheelSlipPushRemainsMeasured)
{
  park();
  for (int i = 1; i <= 300; ++i) scene(4.5 + i * 0.05, i * 0.0005);
  EXPECT_NEAR(node->rf2o_ref.robot_pose_.translation().x(), 0.15, 0.02);
  EXPECT_GT(node->rf2o_ref.last_odom_time.seconds(), 19.0);
}

TEST_F(Startup, SlowWheelHeldShovesDoNotLoseMotionAtReanchors)
{
  for (double increment : {0.001, 0.002, 0.0035})
  {
    SetUp();
    park(false, true);
    const Eigen::Vector3d raw_start = node->rf2o_ref.robot_pose_.translation();
    const Eigen::Vector3d output_start = node->rf2o_ref.published_robot_pose_.translation();
    int anchors = 0;
    double previous_anchor = node->references.front().scan.stamp.seconds();
    for (int i = 1; i <= 100; ++i)
    {
      const rf2o::Pose3d previous_output = node->rf2o_ref.published_robot_pose_;
      scene(4.5 + i * 0.05, i * increment, 0, 0, false, true, true);
      const Eigen::Vector3d velocity = node->rf2o_ref.published_robot_pose_.linear().transpose() *
          (node->rf2o_ref.published_robot_pose_.translation() - previous_output.translation()) / 0.05;
      EXPECT_NEAR(node->rf2o_ref.lin_speed, velocity.x(), 1e-7);
      EXPECT_NEAR(node->rf2o_ref.lat_speed, velocity.y(), 1e-7);
      ASSERT_NEAR(node->rf2o_ref.last_odom_time.seconds(), 4.5 + i * 0.05, 1e-8);
      if (!node->references.empty() && node->references.front().scan.stamp.seconds() != previous_anchor)
      {
        ++anchors;
        EXPECT_TRUE(node->references.front().scan.laser_pose.isApprox(node->rf2o_ref.laser_pose_));
        EXPECT_NEAR(node->references.front().robot_xy.x(), node->rf2o_ref.robot_pose_.translation().x(), 1e-9);
        previous_anchor = node->references.front().scan.stamp.seconds();
      }
    }
    EXPECT_GT(anchors, 2);
    const double distance = 100 * increment;
    RecordProperty("raw_error_mm_per_" + std::to_string(static_cast<int>(increment * 1e6)) + "um_scan",
        std::to_string(1000 * (node->rf2o_ref.robot_pose_.translation().x() - raw_start.x() - distance)));
    RecordProperty("output_error_mm_per_" + std::to_string(static_cast<int>(increment * 1e6)) + "um_scan",
        std::to_string(1000 * (node->rf2o_ref.published_robot_pose_.translation().x() - output_start.x() - distance)));
    EXPECT_NEAR(node->rf2o_ref.robot_pose_.translation().x() - raw_start.x(), distance,
                0.03 * distance) << increment;
    EXPECT_NEAR(node->rf2o_ref.published_robot_pose_.translation().x() - output_start.x(), distance,
                0.03 * distance) << increment;
  }
}

TEST_F(Startup, ObstacleCapturedInAnchorDoesNotBecomeRobotMotion)
{
  park(true);
  for (int i = 1; i <= 200; ++i)
  {
    scene(4.5 + i * 0.05);
    ASSERT_LT(node->rf2o_ref.robot_pose_.translation().head<2>().norm(), 0.004) << i;
    ASSERT_NEAR(node->rf2o_ref.last_odom_time.seconds(), 4.5 + i * 0.05, 1e-9);
  }
  EXPECT_GT(node->rf2o_ref.last_odom_time.seconds(), 8.0);
}

TEST_F(Startup, LiveObstacleMissingFromAnchorDoesNotBecomeRobotMotion)
{
  park();
  for (int i = 1; i <= 200; ++i)
  {
    scene(4.5 + i * 0.05, 0, 0, 0, true);
    ASSERT_LT(node->rf2o_ref.robot_pose_.translation().head<2>().norm(), 0.004) << i;
    ASSERT_NEAR(node->rf2o_ref.last_odom_time.seconds(), 4.5 + i * 0.05, 1e-9);
  }
}

TEST_F(Startup, ContinuousMultiTurnPatrolKeepsMatching)
{
  park();
  Eigen::Vector2d previous = node->rf2o_ref.published_robot_pose_.translation().head<2>();
  int anchored = 0;
  for (int i = 1; i <= 1400; ++i)
  {
    scene(4.5 + i * 0.05, 0, 0, i * 0.01);
    const Eigen::Vector2d xy = node->rf2o_ref.published_robot_pose_.translation().head<2>();
    ASSERT_LT((xy - previous).norm(), 0.004) << i;
    ASSERT_NEAR(node->rf2o_ref.last_odom_time.seconds(), 4.5 + i * 0.05, 1e-9);
    if (!node->references.empty()) ++anchored;
    ASSERT_LT(xy.norm(), 0.01) << i;
    previous = xy;
  }
  EXPECT_GT(anchored, 1300);
}

TEST_F(Startup, FullHeadSweepHasNoAnchorSwitchSteps)
{
  park();
  Eigen::Vector2d previous = node->rf2o_ref.published_robot_pose_.translation().head<2>();
  for (int i = 1; i <= 1200; ++i)
  {
    const double head = M_PI * (1.0 - std::cos(i * 0.004));
    scene(4.5 + i * 0.05, 0, 0, head);
    const Eigen::Vector2d xy = node->rf2o_ref.published_robot_pose_.translation().head<2>();
    ASSERT_LT((xy - previous).norm(), 0.004) << i;
    previous = xy;
    ASSERT_LT(xy.norm(), 0.01) << i;
    ASSERT_LE(node->references.size(), 1u);
  }
  EXPECT_GT(node->rf2o_ref.last_odom_time.seconds(), 64.0);
}

TEST_F(Startup, ReferencesOffMatchesUnanchoredStream)
{
  // Recorded by #16 (ee3ff63) with the identical scene stream; see README.
  std::ifstream baseline(std::string(RF2O_TEST_DATA) + "/ee3ff63_poses.txt");
  ASSERT_TRUE(baseline.good());
  std::vector<Eigen::Vector3d> poses;
  for (int pass = 0; pass < 2; ++pass)
  {
    SetUp();
    node->confidence_enabled = true;
    node->rf2o_ref.fixed_heading = true;
    node->reference_scans = pass != 0;
    for (int i = 0; i <= 200; ++i)
    {
      // Moving throughout: references ON must retain the scan-to-scan path.
      scene(1.0 + i * 0.05, i * 0.003, i * 0.003, 0.2 * std::sin(i * 0.01));
      const Eigen::Vector3d pose = node->rf2o_ref.robot_pose_.translation();
      if (pass == 0)
      {
        poses.push_back(pose);
        double x, y, yaw;
        char comma1, comma2;
        ASSERT_TRUE(baseline >> x >> comma1 >> y >> comma2 >> yaw);
        ASSERT_EQ(comma1, ',');
        ASSERT_EQ(comma2, ',');
        EXPECT_NEAR(pose.x(), x, 1e-7) << i;
        EXPECT_NEAR(pose.y(), y, 1e-7) << i;
        EXPECT_NEAR(rf2o::getYaw(node->rf2o_ref.robot_pose_.rotation()), yaw, 1e-7) << i;
      }
      else EXPECT_TRUE(pose.isApprox(poses[i], 1e-12)) << i;
      EXPECT_TRUE(node->references.empty());
    }
  }
}

}  // namespace
