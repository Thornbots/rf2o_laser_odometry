#include <gtest/gtest.h>
#include <limits>
#include <fstream>
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
    msg->angle_max = msg->angle_min + 719 * msg->angle_increment;
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
    node->reference_max_scans = 8;
  }
  void scene(double t, double x = 0.0, double wheel = 0.0,
             double head = 0.0, bool obstacle = false, bool prior = true)
  {
    if (prior) odom(t, wheel);
    tfAt(t, head);
    auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    msg->header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
    msg->header.frame_id = "laser";
    msg->angle_min = -M_PI;
    msg->angle_increment = 2 * M_PI / 720;
    msg->angle_max = msg->angle_min + 719 * msg->angle_increment;
    msg->range_min = 0.1;
    msg->range_max = 12;
    const double hx = x + 0.22 * std::cos(head), hy = 0.22 * std::sin(head);
    for (int j = 0; j < 720; ++j)
    {
      const double a = msg->angle_min + j * msg->angle_increment + head;
      const double c = std::cos(a), sn = std::sin(a);
      double r = std::numeric_limits<double>::infinity();
      if (std::abs(c) > 1e-9) r = std::min(r, ((c > 0 ? 4.0 : -4.0) - hx) / c);
      if (std::abs(sn) > 1e-9) r = std::min(r, ((sn > 0 ? 4.0 : -4.0) - hy) / sn);
      if (obstacle && std::abs(a) < 0.2) r = std::min(r, 1.0 / c);
      msg->ranges.push_back(r + 0.003 * std::sin(j * 13.7 + t * 0.83));
    }
    node->LaserCallBack(msg);
  }
  void park(bool obstacle = false)
  {
    referencesOn();
    for (int i = 0; i <= 70; ++i) scene(1.0 + i * 0.05, 0, 0, 0, obstacle);
    ASSERT_FALSE(node->references.empty());
  }
  std::unique_ptr<rf2o::CLaserOdometry2DNode> node;
};

TEST_F(Startup, ReferenceParametersRejectInvalidValues)
{
  for (const std::string name : {"reference_max_yaw", "reference_max_offset",
                                "reference_still_distance", "reference_still_time", "reference_max_age"})
    for (double value : {0.0, -0.01, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()})
    {
      rclcpp::NodeOptions options;
      options.parameter_overrides({rclcpp::Parameter(name, value)});
      EXPECT_THROW(rf2o::CLaserOdometry2DNode invalid(options), std::invalid_argument);
    }
  for (int limit : {0, -1, 4097})
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("reference_max_scans", limit)});
    EXPECT_THROW(rf2o::CLaserOdometry2DNode invalid(options), std::invalid_argument);
  }
}

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

TEST_F(Startup, NonRepeatingHeadAnglesHaveABoundedReferenceBank)
{
  referencesOn();
  node->reference_max_yaw = 1e-6;
  size_t largest = 0;
  for (int i = 0; i <= 400; ++i)
  {
    scene(1.0 + i * 0.05, 0, 0, i * 0.011);
    largest = std::max(largest, node->references.size());
    ASSERT_LE(node->references.size(), 8u);
  }
  EXPECT_GT(largest, 0u);
  EXPECT_TRUE(node->rf2o_ref.robot_pose_.matrix().allFinite());
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 21.0);
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
  node->reference_max_scans = 512;
  Eigen::Vector2d previous = node->rf2o_ref.robot_pose_.translation().head<2>();
  int anchored = 0;
  for (int i = 1; i <= 1400; ++i)
  {
    scene(4.5 + i * 0.05, 0, 0, i * 0.01);
    const Eigen::Vector2d xy = node->rf2o_ref.robot_pose_.translation().head<2>();
    ASSERT_LT((xy - previous).norm(), 0.004) << i;
    ASSERT_NEAR(node->rf2o_ref.last_odom_time.seconds(), 4.5 + i * 0.05, 1e-9);
    if (!node->references.empty()) ++anchored;
    ASSERT_LT(xy.norm(), 0.01) << i;
    previous = xy;
  }
  EXPECT_GT(anchored, 1300);
}

TEST_F(Startup, InterruptedScansCannotUseAnExpiredAnchor)
{
  park();
  ASSERT_NE(node->findReference(), nullptr);
  node->rf2o_ref.current_scan_time = rclcpp::Time(int64_t{20000000000}, RCL_ROS_TIME);
  EXPECT_EQ(node->findReference(), nullptr);
}

TEST_F(Startup, FullHeadSweepHasNoAnchorSwitchSteps)
{
  park();
  node->reference_max_scans = 512;
  Eigen::Vector2d previous = node->rf2o_ref.robot_pose_.translation().head<2>();
  for (int i = 1; i <= 1200; ++i)
  {
    const double head = M_PI * (1.0 - std::cos(i * 0.004));
    scene(4.5 + i * 0.05, 0, 0, head);
    const Eigen::Vector2d xy = node->rf2o_ref.robot_pose_.translation().head<2>();
    ASSERT_LT((xy - previous).norm(), 0.004) << i;
    previous = xy;
    ASSERT_LT(xy.norm(), 0.01) << i;
    ASSERT_LE(node->references.size(), 512u);
  }
  EXPECT_GT(node->rf2o_ref.last_odom_time.seconds(), 64.0);
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
  park();
  for (int i = 1; i <= 400; ++i) scene(4.5 + i * 0.05);
  scene(1.0);
  scene(1.05);
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 1.05);
  EXPECT_TRUE(node->references.empty());
  EXPECT_TRUE(node->have_matched_prior);
  EXPECT_LT(node->odom_prior_buf.back().t, 2.0);
}

TEST_F(Startup, ScanFirstClockJumpWaitsForNewOdom)
{
  park();
  for (int i = 1; i <= 400; ++i) scene(4.5 + i * 0.05);
  auto rewound = std::make_shared<sensor_msgs::msg::LaserScan>(node->last_scan);
  rewound->header.stamp = rclcpp::Time(int64_t{1000000000}, RCL_ROS_TIME);
  node->LaserCallBack(rewound);
  EXPECT_FALSE(node->rf2o_ref.is_initialized());
  EXPECT_TRUE(node->odom_prior_buf.empty());
  EXPECT_TRUE(node->references.empty());
  EXPECT_EQ(node->newest_scan_stamp, 1000000000LL);
  tfAt(1.0);
  odom(1.0, 0.0);
  scene(1.05);
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 1.05);
  EXPECT_TRUE(node->pending_scans.empty());
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
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"init_pose_from_topic", "/custom_origin"},
                               {"odom_prior_topic", "/odom"}});
  node = std::make_unique<rf2o::CLaserOdometry2DNode>(options);
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
  node->confidence_enabled = true;
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
  EXPECT_TRUE(node->have_matched_prior);
  EXPECT_DOUBLE_EQ(node->matched_prior.t, 1.1);
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 1.1);
  EXPECT_EQ(node->pending_scans.size(), 2u);
}

TEST_F(Startup, ResolvedTopicsSeedAtScanTime)
{
  for (const auto &args : std::vector<std::vector<std::string>>{
      {}, {"--ros-args", "-r", "__ns:=/robot"},
      {"--ros-args", "-r", "origin:=/wheel", "-r", "/odom:=/wheel"}})
  {
    rclcpp::NodeOptions options;
    options.arguments(args);
    options.parameter_overrides({{"init_pose_from_topic", args.size() > 4 ? "origin" : "odom"},
                                 {"odom_prior_topic", args.size() == 3 ? "/robot/odom" : "/odom"}});
    node = std::make_unique<rf2o::CLaserOdometry2DNode>(options);
    EXPECT_STREQ(node->initPose_sub->get_topic_name(), node->odom_prior_sub->get_topic_name());
    odom(1.0, 7.0);
    odom(2.0, 8.0);
    tf();
    scan(1.5);
    ASSERT_TRUE(node->rf2o_ref.is_initialized());
    EXPECT_NEAR(node->rf2o_ref.robot_pose_.translation().x(), 7.5, 1e-9);
  }
}

TEST_F(Startup, MovingHeadTimeoutFailureRetainsReferenceAndRecoversFullDisplacement)
{
  node->confidence_enabled = true;
  node->rf2o_ref.fixed_heading = true;
  odom(1.0, 0.0);
  tfAt(1.0, 0.2);
  tfAt(2.5, 0.8);
  scan(1.0);
  const auto pyramid = node->rf2o_ref.range;
  const auto pose = node->rf2o_ref.laser_pose_;
  const auto head = node->rf2o_ref.laser_pose_on_robot_prev_;
  scan(1.1);
  // Force NO_LEVELS while the base/head move and odometry is stalled.
  node->pending_scans.back()->ranges.assign(720, std::numeric_limits<float>::infinity());
  scan(1.2);
  node->pending_scans.back()->ranges.assign(720, std::numeric_limits<float>::infinity());
  scan(2.2);
  EXPECT_FALSE(node->has_odom_prior);
  EXPECT_EQ(node->rf2o_ref.getMatchQuality().failure, rf2o::MatchQuality::Failure::NO_LEVELS);
  EXPECT_TRUE(node->have_matched_prior);
  EXPECT_DOUBLE_EQ(node->matched_prior.t, 1.0);
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 1.0);
  EXPECT_TRUE(node->rf2o_ref.laser_pose_.isApprox(pose));
  EXPECT_TRUE(node->rf2o_ref.laser_pose_on_robot_prev_.isApprox(head));
  for (size_t i = 0; i < pyramid.size(); ++i)
    EXPECT_TRUE(node->rf2o_ref.range[i].isApprox(pyramid[i], 0.0f));
  scan(2.3);
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 1.0);
  EXPECT_DOUBLE_EQ(node->matched_prior.t, 1.0);
  EXPECT_NEAR(node->failed_extra, 0.2 * node->failed_variance_rate, 1e-9);
  // Another failure must retain the same reference, including after history expires.
  node->pending_scans.front()->ranges.assign(720, std::numeric_limits<float>::infinity());
  scan(2.5);
  node->pending_scans.back()->ranges.assign(720, std::numeric_limits<float>::infinity());
  odom(2.5, 0.3);
  EXPECT_TRUE(node->pending_scans.empty());
  EXPECT_NEAR(node->rf2o_ref.robot_pose_.translation().x(), 0.3, 1e-6);
  EXPECT_NEAR(node->rf2o_ref.robot_pose_.translation().y(), 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(node->rf2o_ref.last_odom_time.seconds(), 2.5);
  EXPECT_TRUE(node->rf2o_ref.laser_pose_on_robot_prev_.isApprox(node->rf2o_ref.laser_pose_on_robot_));
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
