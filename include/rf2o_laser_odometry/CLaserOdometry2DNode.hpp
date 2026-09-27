#include "rf2o_laser_odometry/CLaserOdometry2D.hpp"

#include <tf2/convert.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2/impl/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <algorithm>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

namespace rf2o {

class CLaserOdometry2DNode : public rclcpp::Node
{
public:
  CLaserOdometry2DNode();
  enum class Tier { GOOD, DEGRADED, FAILED };

  void process();
  void publish(const Eigen::Matrix2d &position_cov);
  bool setLaserPoseFromTf();
  void warnOnSkippedScan();

  // Params & vars
  CLaserOdometry2D    rf2o_ref;
  bool                publish_tf;
  rclcpp::Time        last_scan_stamp{0, 0, RCL_ROS_TIME};
  double              min_scan_gap = 0.0;
  // Reported measurement uncertainty (variances, not standard deviations)
  // for the published Odometry. Left at zero these read as "infinitely
  // certain" to any downstream Kalman filter -- see publish().
  double              position_covariance;
  double              yaw_covariance;
  double              linear_velocity_covariance;
  double              angular_velocity_covariance;
  std::string         laser_scan_topic;
  std::string         odom_topic;
  std::string         base_frame_id;
  std::string         odom_frame_id;
  std::string         init_pose_from_topic;
  std::string         odom_prior_topic;

  // Match confidence; see README.md. Signals are always computed and
  // published on <odom_topic>/quality; confidence_enabled applies them.
  bool                confidence_enabled;
  double              min_valid_fraction;     // below: degraded
  double              fail_valid_fraction;    // below: failed
  double              max_match_sigma;        // m, per axis; above: degraded
  double              max_speed;              // m/s; above: failed
  double              degraded_position_variance;  // m^2, added per weak axis
  double              failed_variance_rate;   // m^2/s while failed
  double              max_failed_variance;    // m^2 cap
  int                 recovery_matches;       // good matches to shed it
  double              failed_extra = 0.0;     // current added variance, m^2
  double              recovery_step = 0.0;
  bool                extrinsic_stale = false;
  double              last_gap_periods = 0.0;
  bool                has_odom_prior = false;
  Pose3d              odom_prior_increment = Pose3d::Identity();

  sensor_msgs::msg::LaserScan                     last_scan;
  bool                                            GT_pose_initialized;
  std::shared_ptr<tf2_ros::Buffer>                buffer_;
  std::shared_ptr<tf2_ros::TransformListener>     tf_listener_;  
  std::unique_ptr<tf2_ros::TransformBroadcaster>  odom_broadcaster;
  nav_msgs::msg::Odometry                         initial_robot_pose;

  // Subscriptions & Publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      initPose_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_prior_sub;

  // Recent odom_prior_topic poses (stamp s, x, y, yaw), oldest first.
  struct OdomSample { double t, x, y, yaw; };
  std::deque<OdomSample>                                        odom_prior_buf;
  bool odomPriorAt(double t, OdomSample &out) const;
  bool setOdomPrior();
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr quality_pub;

  Tier classify(bool solved, const MatchQuality &q, const Eigen::Vector2d &sigma,
                std::string &reason) const;
  void publishQuality(Tier tier, const std::string &reason, const MatchQuality &q,
                      const Eigen::Vector2d &sigma, const char *increment);

  // CallBacks
  void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan);
  void initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose);
  void odomPriorCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);
};

