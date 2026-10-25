// Match grading and deadReckon() in CLaserOdometry2D, on synthetic scans
// of a square room seen from its centre. Scans carry +-5 mm noise: with
// none, every range derivative is zero and rf2o's weights go NaN.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "rf2o_laser_odometry/CLaserOdometry2D.hpp"

using rf2o::CLaserOdometry2D;
using rf2o::MatchQuality;
using rf2o::Pose3d;

namespace
{

constexpr int kBeams = 720;
constexpr double kHalfSide = 2.0;  // m

sensor_msgs::msg::LaserScan roomScan(double t, double keep_fraction = 1.0)
{
  sensor_msgs::msg::LaserScan s;
  s.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
  s.header.frame_id = "laser";
  s.angle_min = -M_PI;
  s.angle_increment = 2.0 * M_PI / kBeams;
  s.angle_max = s.angle_min + (kBeams - 1) * s.angle_increment;
  s.range_min = 0.1;
  s.range_max = 12.0;
  s.ranges.resize(kBeams);
  uint32_t seed = static_cast<uint32_t>(t * 1000) + 1;
  for (int i = 0; i < kBeams; i++)
  {
    seed = seed * 1664525u + 1013904223u;
    const double noise = 0.005 * (2.0 * (seed >> 8) / double(1u << 24) - 1.0);
    const double a = s.angle_min + i * s.angle_increment;
    s.ranges[i] = i < keep_fraction * kBeams
        ? kHalfSide / std::max(std::abs(std::cos(a)), std::abs(std::sin(a))) + noise
        : std::numeric_limits<float>::infinity();
  }
  return s;
}

// roomScan() seen from a lidar at (x, y), turned by yaw, in the room's frame.
sensor_msgs::msg::LaserScan roomScanFrom(double t, double x, double y, double yaw)
{
  auto s = roomScan(t);
  uint32_t seed = static_cast<uint32_t>(t * 1000) + 1;
  for (int i = 0; i < kBeams; i++)
  {
    seed = seed * 1664525u + 1013904223u;
    const double noise = 0.005 * (2.0 * (seed >> 8) / static_cast<double>(1u << 24) - 1.0);
    const double a = s.angle_min + i * s.angle_increment + yaw;
    const double c = std::cos(a), sn = std::sin(a);
    double r = std::numeric_limits<double>::infinity();
    if (std::abs(c) > 1e-9) r = std::min(r, ((c > 0 ? kHalfSide : -kHalfSide) - x) / c);
    if (std::abs(sn) > 1e-9) r = std::min(r, ((sn > 0 ? kHalfSide : -kHalfSide) - y) / sn);
    s.ranges[i] = static_cast<float>(r + noise);
  }
  return s;
}

// The sentry's RPLIDAR A2M8 scan as the driver publishes it with
// angle_compensate:=true (2026-10-18 robot capture): 720 beams from -pi to +pi
// inclusive (increment 2pi/719), range_min 0.05, ~40% scattered no-returns
// (+inf). Seen from (px, 0) in a square room; the mask depends on beam index
// only, in runs of `run` beams like the open-air and glazing losses on the
// real field.
sensor_msgs::msg::LaserScan realFormatScan(double t, double px = 0.0, int run = 12)
{
  sensor_msgs::msg::LaserScan s;
  s.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
  s.header.frame_id = "lidar";
  s.angle_min = -M_PI;
  s.angle_max = M_PI;
  s.angle_increment = 2.0 * M_PI / (kBeams - 1);
  s.range_min = 0.05;
  s.range_max = 12.0;
  s.ranges.resize(kBeams);
  uint32_t noise_seed = static_cast<uint32_t>(t * 1000) + 1;
  for (int i = 0; i < kBeams; i++)
  {
    uint32_t h = (static_cast<uint32_t>(i / run) + 1u) * 2654435761u;
    h ^= h >> 15;
    const bool no_return = (h % 100u) < 40u;
    noise_seed = noise_seed * 1664525u + 1013904223u;
    const double noise = 0.005 * (2.0 * (noise_seed >> 8) / static_cast<double>(1u << 24) - 1.0);
    const double a = s.angle_min + i * s.angle_increment;
    const double c = std::cos(a), sn = std::sin(a);
    double r = std::numeric_limits<double>::infinity();
    if (std::abs(c) > 1e-9) r = std::min(r, ((c > 0 ? kHalfSide : -kHalfSide) - px) / c);
    if (std::abs(sn) > 1e-9) r = std::min(r, kHalfSide / std::abs(sn));
    s.ranges[i] = no_return ? std::numeric_limits<float>::infinity()
                            : static_cast<float>(r + noise);
  }
  return s;
}

class MatchQualityTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    rf2o = std::make_unique<CLaserOdometry2D>();
    // An offset, turned lidar, so the laser/robot frame change is exercised.
    Pose3d extrinsic = Pose3d::Identity();
    extrinsic.translation() << 0.1, -0.05, 0.3;
    extrinsic.linear() = rf2o::matrixYaw(0.4);
    rf2o->setLaserPose(extrinsic);
    geometry_msgs::msg::Pose origin;
    origin.orientation.w = 1.0;
    rf2o->init(roomScan(0.0), origin);
    // The first match runs against init()'s empty pyramid; get past it.
    match(roomScan(0.1));
  }

  bool match(const sensor_msgs::msg::LaserScan &scan)
  {
    return rf2o->odometryCalculation(scan);
  }

  std::unique_ptr<CLaserOdometry2D> rf2o;
};

TEST_F(MatchQualityTest, StationaryFullScanIsClean)
{
  ASSERT_TRUE(match(roomScan(0.2)));
  const MatchQuality &q = rf2o->getMatchQuality();
  EXPECT_EQ(q.failure, MatchQuality::Failure::NONE);
  EXPECT_TRUE(q.finest_level_solved);
  EXPECT_GT(double(q.valid_points) / q.points, 0.95);
  EXPECT_LT(q.speed, 0.05);
}

TEST_F(MatchQualityTest, VariableSizeScansAreSkippedWithoutChangingHistory)
{
  ASSERT_TRUE(match(roomScan(0.2)));
  const Pose3d before = rf2o->getPose();
  const auto history = rf2o->range;
  const auto previous_time = rf2o->last_odom_time;
  for (const size_t beams : {0u, 340u, 359u, 719u, 721u, 1440u})
  {
    auto scan = roomScan(0.3);
    scan.ranges.resize(beams, 2.0f);
    EXPECT_FALSE(match(scan)) << beams;
    EXPECT_EQ(rf2o->getMatchQuality().failure, MatchQuality::Failure::GEOMETRY);
    EXPECT_TRUE(rf2o->getPose().isApprox(before));
    EXPECT_EQ(rf2o->last_odom_time, previous_time);
    for (size_t i = 0; i < history.size(); ++i)
      EXPECT_TRUE(rf2o->range[i].isApprox(history[i], 0.0f));
  }
  ASSERT_TRUE(match(roomScan(0.4)));
  EXPECT_LT(rf2o->getMatchQuality().speed, 0.05);
}

TEST_F(MatchQualityTest, NonmonotonicStampsLeaveMatcherStateUntouched)
{
  ASSERT_TRUE(match(roomScan(0.2)));
  const Pose3d pose = rf2o->getPose();
  const Pose3d increment = rf2o->getIncrement();
  const auto covariance = rf2o->getIncrementCovariance();
  const auto history = rf2o->range;
  const auto old_history = rf2o->range_old;
  const auto input = rf2o->range_wf;
  const auto prior = rf2o->kai_loc_old_;
  const auto time = rf2o->current_scan_time;
  for (double stamp : {0.2, 0.1, 0.0})
  {
    EXPECT_FALSE(match(roomScan(stamp)));
    EXPECT_EQ(rf2o->getMatchQuality().failure, MatchQuality::Failure::TIMESTAMP);
    EXPECT_EQ(rf2o->current_scan_time, time);
    EXPECT_EQ(rf2o->last_odom_time, time);
    EXPECT_TRUE(rf2o->getPose().isApprox(pose));
    EXPECT_TRUE(rf2o->getIncrement().isApprox(increment));
    EXPECT_TRUE(rf2o->getIncrementCovariance().isApprox(covariance, 0.f));
    EXPECT_TRUE(rf2o->range_wf.isApprox(input, 0.f));
    EXPECT_TRUE(rf2o->kai_loc_old_.isApprox(prior, 0.f));
    for (size_t i = 0; i < history.size(); ++i)
    {
      EXPECT_TRUE(rf2o->range[i].isApprox(history[i], 0.f));
      EXPECT_TRUE(rf2o->range_old[i].isApprox(old_history[i], 0.f));
    }
  }
  EXPECT_TRUE(match(roomScan(0.3)));
}

TEST_F(MatchQualityTest, FailedScanStillDefinesTimestampAndPyramidReference)
{
  ASSERT_TRUE(match(roomScan(0.2)));
  EXPECT_FALSE(match(roomScan(0.4, 0.0)));
  ASSERT_EQ(rf2o->getMatchQuality().failure, MatchQuality::Failure::NO_LEVELS);
  const auto history = rf2o->range;
  const auto old_history = rf2o->range_old;
  for (double stamp : {0.3, 0.4})
  {
    EXPECT_FALSE(match(roomScan(stamp)));
    EXPECT_EQ(rf2o->getMatchQuality().failure, MatchQuality::Failure::TIMESTAMP);
    EXPECT_EQ(rf2o->last_odom_time, rclcpp::Time(int64_t{200000000}, RCL_ROS_TIME));
    EXPECT_EQ(rf2o->current_scan_time, rclcpp::Time(int64_t{400000000}, RCL_ROS_TIME));
    for (size_t i = 0; i < history.size(); ++i)
    {
      EXPECT_TRUE(rf2o->range[i].isApprox(history[i], 0.f));
      EXPECT_TRUE(rf2o->range_old[i].isApprox(old_history[i], 0.f));
    }
  }
  // The blank reference consumes the next scan; the following pair recovers.
  EXPECT_FALSE(match(roomScan(0.5)));
  EXPECT_TRUE(match(roomScan(0.6)));
}

TEST_F(MatchQualityTest, SolversRejectSingularAndNonfiniteNormalEquations)
{
  ASSERT_TRUE(match(roomScan(0.2)));
  const auto weights = rf2o->weights;
  rf2o->weights.setZero();
  EXPECT_FALSE(rf2o->solveSystemOneLevel());
  EXPECT_FALSE(rf2o->solveSystemNonLinear());
  rf2o->weights = weights;
  rf2o->dt.setConstant(std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(rf2o->solveSystemOneLevel());
  EXPECT_FALSE(rf2o->solveSystemNonLinear());
}

TEST_F(MatchQualityTest, FilterRejectsNonfiniteIncrementsWithoutChangingTransform)
{
  ASSERT_TRUE(match(roomScan(0.2)));
  rf2o->cov_odo.setIdentity();
  rf2o->kai_loc_old_.setZero();
  const auto transform = rf2o->transformations[rf2o->level];
  for (float invalid : {std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity()})
  {
    rf2o->kai_loc_level_.setZero();
    rf2o->kai_loc_level_(0) = invalid;
    EXPECT_FALSE(rf2o->filterLevelSolution());
    EXPECT_TRUE(rf2o->transformations[rf2o->level].isApprox(transform, 0.f));
  }
}

TEST_F(MatchQualityTest, FilterRejectsInvalidCovariancesWithoutChangingTransform)
{
  ASSERT_TRUE(match(roomScan(0.2)));
  rf2o->kai_loc_level_.setZero();
  rf2o->kai_loc_old_.setZero();
  const auto transform = rf2o->transformations[rf2o->level];
  for (int invalid = 0; invalid < 4; ++invalid)
  {
    rf2o->cov_odo.setIdentity();
    if (invalid == 0) rf2o->cov_odo(0, 0) = std::numeric_limits<float>::quiet_NaN();
    if (invalid == 1) rf2o->cov_odo(0, 0) = std::numeric_limits<float>::infinity();
    if (invalid == 2) rf2o->cov_odo(0, 0) = -1.f;
    if (invalid == 3) rf2o->cov_odo(0, 1) = 1.f;
    EXPECT_FALSE(rf2o->filterLevelSolution()) << invalid;
    EXPECT_TRUE(rf2o->transformations[rf2o->level].isApprox(transform, 0.f));
  }
}

TEST_F(MatchQualityTest, ZeroResidualRobustSolveFailsWithoutContaminatingPose)
{
  const auto reference = roomScan(0.2);
  ASSERT_TRUE(match(reference));
  const Pose3d pose = rf2o->getPose();
  const auto time = rf2o->last_odom_time;
  auto identical = reference;
  identical.header.stamp = roomScan(0.3).header.stamp;
  EXPECT_FALSE(match(identical));
  EXPECT_EQ(rf2o->getMatchQuality().failure, MatchQuality::Failure::EIGENSOLVER);
  EXPECT_TRUE(rf2o->getPose().isApprox(pose));
  EXPECT_EQ(rf2o->last_odom_time, time);
  rf2o->deadReckon(Pose3d::Identity());
  EXPECT_TRUE(match(roomScan(0.4)));
}

TEST_F(MatchQualityTest, NumericalFailureKeepsPoseAndRecoversThroughDeadReckoning)
{
  ASSERT_TRUE(match(roomScan(0.2)));
  const Pose3d pose = rf2o->getPose();
  const Pose3d increment = rf2o->getIncrement();
  const auto time = rf2o->last_odom_time;
  rf2o->kai_loc_old_(0) = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(match(roomScan(0.3)));
  EXPECT_EQ(rf2o->getMatchQuality().failure, MatchQuality::Failure::EIGENSOLVER);
  EXPECT_TRUE(rf2o->getPose().isApprox(pose));
  EXPECT_TRUE(rf2o->getIncrement().isApprox(increment));
  EXPECT_EQ(rf2o->last_odom_time, time);
  // As in the node, advance the pose to the failed scan's retained reference.
  rf2o->deadReckon(Pose3d::Identity());
  EXPECT_TRUE(rf2o->getPose().isApprox(pose));
  EXPECT_EQ(rf2o->last_odom_time, rf2o->current_scan_time);
  ASSERT_TRUE(match(roomScan(0.4)));
  EXPECT_TRUE(rf2o->getPose().matrix().allFinite());
  EXPECT_TRUE(rf2o->getIncrementCovariance().allFinite());
  EXPECT_LT(rf2o->getMatchQuality().speed, 0.05);
}

TEST_F(MatchQualityTest, ChangedAnglesAreSkippedAndOriginalGridRecovers)
{
  ASSERT_TRUE(match(roomScan(0.2)));
  const Pose3d before = rf2o->getPose();
  const auto previous_time = rf2o->last_odom_time;
  for (int field = 0; field < 3; ++field)
  {
    auto scan = roomScan(0.3);
    float* angle = field == 0 ? &scan.angle_min :
      field == 1 ? &scan.angle_increment : &scan.angle_max;
    *angle += 0.001f;
    EXPECT_FALSE(match(scan));
    EXPECT_EQ(rf2o->getMatchQuality().failure, MatchQuality::Failure::GEOMETRY);
    *angle = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(match(scan));
    EXPECT_TRUE(rf2o->getPose().isApprox(before));
    EXPECT_EQ(rf2o->last_odom_time, previous_time);
  }
  ASSERT_TRUE(match(roomScan(0.4)));
  EXPECT_LT(rf2o->getMatchQuality().speed, 0.05);
}

TEST_F(MatchQualityTest, BlankScanFailsInsteadOfReadingStopped)
{
  const Pose3d before = rf2o->getPose();
  EXPECT_FALSE(match(roomScan(0.2, 0.0)));
  EXPECT_EQ(rf2o->getMatchQuality().failure, MatchQuality::Failure::NO_LEVELS);
  EXPECT_TRUE(rf2o->getPose().isApprox(before));
}

TEST_F(MatchQualityTest, FewPointsFailsOnlyWhenEnabled)
{
  ASSERT_TRUE(match(roomScan(0.2, 0.15)));
  const MatchQuality &q = rf2o->getMatchQuality();
  EXPECT_LT(double(q.valid_points) / q.points, 0.25);
  rf2o->fail_valid_fraction = 0.25;
  EXPECT_FALSE(match(roomScan(0.3, 0.15)));
  EXPECT_EQ(rf2o->getMatchQuality().failure, MatchQuality::Failure::FEW_POINTS);
}

TEST_F(MatchQualityTest, DeadReckonMovesTheRobotByTheIncrement)
{
  const Pose3d before = rf2o->getPose();

  Pose3d inc = Pose3d::Identity();
  inc.translation() << 0.3, 0.1, 0.0;
  rf2o->current_scan_time = rclcpp::Time(int64_t{200000000}, RCL_ROS_TIME);
  rf2o->deadReckon(inc);

  const Eigen::Vector3d moved = rf2o->getPose().translation() - before.translation();
  EXPECT_NEAR(moved.x(), 0.3, 1e-4);
  EXPECT_NEAR(moved.y(), 0.1, 1e-4);
  EXPECT_EQ(rf2o->last_odom_time, rf2o->current_scan_time);
}

// A head-mounted lidar swings about the robot's z axis without the robot
// moving: rotate the fixture's extrinsic about the robot origin.
Pose3d swungExtrinsic(const Pose3d &extrinsic, double swing)
{
  Pose3d turn = Pose3d::Identity();
  turn.linear() = rf2o::matrixYaw(swing);
  return turn * extrinsic;
}

TEST_F(MatchQualityTest, DeadReckonCarriesAHeadSwing)
{
  rf2o->fixed_heading = true;
  const Pose3d before = rf2o->getPose();
  rf2o->setLaserPose(swungExtrinsic(rf2o->laser_pose_on_robot_, 0.3));
  Pose3d inc = Pose3d::Identity();
  inc.translation() << 0.3, 0.1, 0.0;
  rf2o->current_scan_time = rclcpp::Time(int64_t{200000000}, RCL_ROS_TIME);
  rf2o->deadReckon(inc);
  const Eigen::Vector3d moved = rf2o->getPose().translation() - before.translation();
  EXPECT_NEAR(moved.x(), 0.3, 1e-4);
  EXPECT_NEAR(moved.y(), 0.1, 1e-4);
}

// Seeded as the node's odom prior seeds a parked robot, a fast head swing
// (0.3 rad in one scan) must not read as speed or move the robot.
TEST_F(MatchQualityTest, SeededHeadSwingIsNeitherSpeedNorMotion)
{
  rf2o->fixed_heading = true;
  const Pose3d before = rf2o->getPose();
  // The fixture's scans put the lidar at the room centre, facing +x.
  const Pose3d extrinsic = rf2o->laser_pose_on_robot_;
  const Pose3d swung = swungExtrinsic(extrinsic, 0.3);
  const Pose3d lidar = extrinsic.inverse() * swung;
  rf2o->setLaserPose(swung);
  const Pose3d seed = rf2o->laserIncrement(Pose3d::Identity());
  rf2o->kai_loc_old_ << seed.translation().x(), seed.translation().y(),
      rf2o::getYaw(seed.rotation());
  ASSERT_TRUE(match(roomScanFrom(0.2, lidar.translation().x(), lidar.translation().y(),
                                 rf2o::getYaw(lidar.rotation()))));
  EXPECT_LT(rf2o->getMatchQuality().speed, 0.05);
  const Eigen::Vector3d moved = rf2o->getPose().translation() - before.translation();
  EXPECT_LT(moved.head<2>().norm(), 0.005) << moved.transpose();
}

TEST(RealFormatScan, InitializesAndMatchesTranslation)
{
  rclcpp::init(0, nullptr);
  {
    CLaserOdometry2D rf2o;
    // Same offset, turned lidar as the fixture above.
    Pose3d extrinsic = Pose3d::Identity();
    extrinsic.translation() << 0.1, -0.05, 0.3;
    extrinsic.linear() = rf2o::matrixYaw(0.4);
    rf2o.setLaserPose(extrinsic);
    geometry_msgs::msg::Pose origin;
    origin.orientation.w = 1.0;
    rf2o.init(realFormatScan(0.0), origin);
    ASSERT_TRUE(rf2o.module_initialized);

    auto match = [&](const sensor_msgs::msg::LaserScan &scan) {
      rf2o.current_scan_time = scan.header.stamp;
      return rf2o.odometryCalculation(scan);
    };
    // The first match runs against init()'s empty pyramid; get past it.
    match(realFormatScan(0.1));  // against init()'s empty pyramid: no_levels
    ASSERT_TRUE(match(realFormatScan(0.2)));
    const MatchQuality &still = rf2o.getMatchQuality();
    EXPECT_EQ(still.failure, MatchQuality::Failure::NONE);
    EXPECT_TRUE(still.finest_level_solved);
    EXPECT_LT(still.speed, 0.05);
    // About 60% of beams return; ~40% are +inf.
    EXPECT_GT(static_cast<double>(still.valid_points) / still.points, 0.5);
    EXPECT_LT(static_cast<double>(still.valid_points) / still.points, 0.7);

    const Pose3d before = rf2o.getPose();
    ASSERT_TRUE(match(realFormatScan(0.3, 0.05)));  // lidar moved +0.05 m in x
    EXPECT_EQ(rf2o.getMatchQuality().failure, MatchQuality::Failure::NONE);
    const Eigen::Vector3d moved = rf2o.getPose().translation() - before.translation();
    // Robot-frame displacement: the lidar's +x, turned by its 0.4 rad yaw.
    EXPECT_NEAR(moved.x(), 0.05 * std::cos(0.4), 0.01);
    EXPECT_NEAR(moved.y(), 0.05 * std::sin(0.4), 0.01);
  }
  rclcpp::shutdown();
}

}  // namespace
