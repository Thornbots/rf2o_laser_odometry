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
    rf2o->current_scan_time = scan.header.stamp;
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
