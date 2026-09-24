/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf
*
* Maintainer: Javier G. Monroy
* MAPIR group: https://mapir.isa.uma.es
*
* Modifications: Jeremie Deray & (see contributons on github)
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2DNode.hpp"

using namespace rf2o;

CLaserOdometry2DNode::CLaserOdometry2DNode(): Node("CLaserOdometry2DNode")
{
  RCLCPP_INFO(get_logger(), "Initializing RF2O node...");

  // Read Parameters
  //----------------
  this->declare_parameter<std::string>("laser_scan_topic", "/scan");
  this->get_parameter("laser_scan_topic", laser_scan_topic);
  this->declare_parameter<std::string>("odom_topic", "/odom_rf2o");
  this->get_parameter("odom_topic", odom_topic);
  this->declare_parameter<std::string>("base_frame_id", "base_link");
  this->get_parameter("base_frame_id", base_frame_id);
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->declare_parameter<bool>("publish_tf", true);
  this->get_parameter("publish_tf", publish_tf);
  this->declare_parameter<std::string>("init_pose_from_topic", "/base_pose_ground_truth");
  this->get_parameter("init_pose_from_topic", init_pose_from_topic);
  // true: the robot's yaw stays at its initial value (holonomic chassis
  // with a fixed heading); rf2o still solves for rotation, only the
  // accumulated heading is pinned.
  this->declare_parameter<bool>("fixed_heading", false);
  this->get_parameter("fixed_heading", rf2o_ref.fixed_heading);
  // Non-empty: seed each match with this Odometry topic's motion between
  // the two scans, in place of rf2o's constant-velocity guess, which reads
  // "stopped" at the start of every move and pulls the match short.
  this->declare_parameter<std::string>("odom_prior_topic", "");
  this->get_parameter("odom_prior_topic", odom_prior_topic);

  // Measurement uncertainty reported on the published Odometry (see
  // publish()). Defaults assume scan matching is noisier than wheel
  // encoders: 0.02m position and 0.05rad yaw standard deviation, squared
  // here into variances. Tune per robot/sensor via parameters rather than
  // editing these.
  this->declare_parameter<double>("position_covariance", 0.02 * 0.02);
  this->get_parameter("position_covariance", position_covariance);
  this->declare_parameter<double>("yaw_covariance", 0.05 * 0.05);
  this->get_parameter("yaw_covariance", yaw_covariance);
  this->declare_parameter<double>("linear_velocity_covariance", 0.05 * 0.05);
  this->get_parameter("linear_velocity_covariance", linear_velocity_covariance);
  this->declare_parameter<double>("angular_velocity_covariance", 0.1 * 0.1);
  this->get_parameter("angular_velocity_covariance", angular_velocity_covariance);

  // Init Publishers and Subscribers
  //---------------------------------
  buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  odom_pub  = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);
  // Every scan is matched against the one before it, so queue a burst
  // (a sim running faster than real time) instead of dropping to the newest.
  laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_scan_topic,rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
      std::bind(&CLaserOdometry2DNode::LaserCallBack, this, std::placeholders::_1));
  
  if (!odom_prior_topic.empty())
  {
    odom_prior_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_prior_topic, 50,
        std::bind(&CLaserOdometry2DNode::odomPriorCallBack, this, std::placeholders::_1));
  }

  // Initialize pose
  if (init_pose_from_topic != "")
  {
    initPose_sub = this->create_subscription<nav_msgs::msg::Odometry>(init_pose_from_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&CLaserOdometry2DNode::initPoseCallBack, this, std::placeholders::_1));
    GT_pose_initialized  = false;
  }
  else
  {
    // init to 0
    GT_pose_initialized = true;
    initial_robot_pose.pose.pose.position.x = 0;
    initial_robot_pose.pose.pose.position.y = 0;
    initial_robot_pose.pose.pose.position.z = 0;
    initial_robot_pose.pose.pose.orientation.w = 0;
    initial_robot_pose.pose.pose.orientation.x = 0;
    initial_robot_pose.pose.pose.orientation.y = 0;
    initial_robot_pose.pose.pose.orientation.z = 0;
  }

  // Init variables
  rf2o_ref.module_initialized = false;
  rf2o_ref.first_laser_scan   = true;
}


/**
 * Matches every scan against the previous one as it arrives, so the node
 * keeps pace with the scan stream in any clock (a sim at 10x real time
 * included). The first scan initializes the module instead.
*/
void CLaserOdometry2DNode::LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan)
{
  if (GT_pose_initialized)
  {
    last_scan = *new_scan;
    rf2o_ref.current_scan_time = last_scan.header.stamp;

    if (rf2o_ref.first_laser_scan == false)
    {
      warnOnSkippedScan();
      process();
    }
    else if (setLaserPoseFromTf())
    {
      // Initialize module on first scan (from laser params). Only once the
      // laser->base_frame_id transform actually resolves -- otherwise wait
      // and retry on the next scan rather than initializing from a bogus
      // identity transform.
      rf2o_ref.init(last_scan, initial_robot_pose.pose.pose);
      rf2o_ref.first_laser_scan = false;
    }
  }
}


/**
   * Gets the laser pose with respect the base_link (through TF)
   * This allow estimation of the odometry with respect to the robot base reference system.
   * Called every scan (not just once at startup) so a non-rigid sensor
   * mount (e.g. a lidar on a moving head joint) is tracked correctly.
   * Looked up at the scan's stamp, falling back to the latest transform
   * when TF hasn't reached that stamp yet. Returns false, leaving the last
   * known-good transform in place, if both lookups fail.
   */
bool CLaserOdometry2DNode::setLaserPoseFromTf()
{
  geometry_msgs::msg::TransformStamped tf_laser;

  try
  {
    tf_laser = buffer_->lookupTransform(base_frame_id, last_scan.header.frame_id,
                                        tf2_ros::fromMsg(last_scan.header.stamp));
  }
  catch (tf2::ExtrapolationException &)
  {
    try
    {
      tf_laser = buffer_->lookupTransform(base_frame_id, last_scan.header.frame_id, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(get_logger(), "Could not look up %s -> %s, keeping last known transform: %s",
                  base_frame_id.c_str(), last_scan.header.frame_id.c_str(), ex.what());
      return false;
    }
  }
  catch (tf2::TransformException &ex)
  {
    // Don't let a transient lookup failure overwrite the last known-good
    // extrinsic with a garbage/identity transform -- keep using it until
    // the next successful lookup. This matters once the laser->base_frame_id
    // transform is re-queried every scan (not just once at startup): a
    // momentary TF gap should degrade to "stale but valid", not corrupt.
    RCLCPP_WARN(get_logger(), "Could not look up %s -> %s, keeping last known transform: %s",
                base_frame_id.c_str(), last_scan.header.frame_id.c_str(), ex.what());
    return false;
  }

  // Keep this transform as Eigen Matrix3d
  tf2::Transform transform;
  tf2::convert(tf_laser.transform, transform);
  const tf2::Matrix3x3 &basis = transform.getBasis();
  Eigen::Matrix3d R;

  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      R(r,c) = basis[r][c];

  Pose3d laser_tf(R);

  const tf2::Vector3 &t = transform.getOrigin();
  laser_tf.translation()(0) = t[0];
  laser_tf.translation()(1) = t[1];
  laser_tf.translation()(2) = t[2];

  // Sets this transform in rf2o
  rf2o_ref.setLaserPose(laser_tf);

  return true;
}


/**
 * Warns when the gap to the previous scan's stamp exceeds 1.5 scan periods,
 * i.e. a scan was lost upstream or dropped from the full subscription queue.
 * The period is the scan's scan_time, or the smallest gap seen if that is 0.
*/
void CLaserOdometry2DNode::warnOnSkippedScan()
{
  const rclcpp::Time stamp(last_scan.header.stamp);
  if (last_scan_stamp.nanoseconds() > 0)
  {
    const double gap = (stamp - last_scan_stamp).seconds();
    if (gap > 0.0 && (min_scan_gap <= 0.0 || gap < min_scan_gap))
      min_scan_gap = gap;
    const double period = last_scan.scan_time > 0.0f ? last_scan.scan_time : min_scan_gap;
    if (period > 0.0 && gap > 1.5 * period)
      RCLCPP_WARN(get_logger(), "Scan gap %.3fs is %.1f scan periods; a scan was skipped",
                  gap, gap / period);
  }
  last_scan_stamp = stamp;
}


/**
 * Estimate the odometry from last_scan against the previous scan and publish it
*/
void CLaserOdometry2DNode::process()
{
  if (rf2o_ref.is_initialized())
  {
    // Refresh the laser->base_frame_id extrinsic every scan, not just once
    // at startup -- the sensor mount isn't assumed rigid (e.g. a
    // head-mounted lidar that pans independently of the base). Safe to do
    // per-scan: laser_pose_ (the scan-matched absolute laser pose) is
    // computed independently of this extrinsic; only the final
    // laser_pose_ -> robot_pose_ conversion uses it.
    setLaserPoseFromTf();

    // Replace the constant-velocity guess with measured motion, if any.
    if (!odom_prior_topic.empty())
      setOdomPrior();

    // Process odometry estimation
    rf2o_ref.odometryCalculation(last_scan);

    // Publish odometry over ROS2 (tf/topic)
    publish();
  }
}


void CLaserOdometry2DNode::odomPriorCallBack(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  OdomSample s;
  s.t = rclcpp::Time(msg->header.stamp).seconds();
  s.x = msg->pose.pose.position.x;
  s.y = msg->pose.pose.position.y;
  s.yaw = tf2::getYaw(msg->pose.pose.orientation);
  if (!odom_prior_buf.empty())
  {
    if (s.t == odom_prior_buf.back().t)
      return;  // duplicate stamp
    if (s.t < odom_prior_buf.back().t)
      odom_prior_buf.clear();  // time went backwards (sim reset): start over
  }
  odom_prior_buf.push_back(s);
  while (odom_prior_buf.size() > 2 && odom_prior_buf.back().t - odom_prior_buf.front().t > 2.0)
    odom_prior_buf.pop_front();
}


/**
 * Odom pose at time t, linearly interpolated; false if t is outside the
 * buffer by more than 50 ms.
 */
bool CLaserOdometry2DNode::odomPriorAt(double t, OdomSample &out) const
{
  if (odom_prior_buf.empty() || t < odom_prior_buf.front().t - 0.05 ||
      t > odom_prior_buf.back().t + 0.05)
    return false;
  if (t <= odom_prior_buf.front().t) { out = odom_prior_buf.front(); return true; }
  if (t >= odom_prior_buf.back().t) { out = odom_prior_buf.back(); return true; }
  for (size_t i = 1; i < odom_prior_buf.size(); i++)
  {
    const OdomSample &a = odom_prior_buf[i - 1], &b = odom_prior_buf[i];
    if (t <= b.t)
    {
      const double k = (b.t > a.t) ? (t - a.t) / (b.t - a.t) : 0.0;
      out.t = t;
      out.x = a.x + k * (b.x - a.x);
      out.y = a.y + k * (b.y - a.y);
      out.yaw = a.yaw + k * std::remainder(b.yaw - a.yaw, 2.0 * M_PI);
      return true;
    }
  }
  return false;
}


/**
 * Set rf2o's velocity prior to the odom motion between the last matched
 * scan and this one, as a per-scan increment in the previous laser frame.
 * Leaves the constant-velocity prior in place if odom doesn't cover both.
 */
bool CLaserOdometry2DNode::setOdomPrior()
{
  OdomSample p0, p1;
  if (!odomPriorAt(rf2o_ref.last_odom_time.seconds(), p0) ||
      !odomPriorAt(rf2o_ref.current_scan_time.seconds(), p1))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "No %s pose at both scan stamps (%.3f, %.3f; buffer %zu, %.3f..%.3f); "
      "using the constant-velocity prior", odom_prior_topic.c_str(),
      rf2o_ref.last_odom_time.seconds(), rf2o_ref.current_scan_time.seconds(),
      odom_prior_buf.size(), odom_prior_buf.empty() ? 0.0 : odom_prior_buf.front().t,
      odom_prior_buf.empty() ? 0.0 : odom_prior_buf.back().t);
    return false;
  }
  RCLCPP_INFO_ONCE(get_logger(), "Seeding scan matches from %s", odom_prior_topic.c_str());

  // Robot increment in the previous robot frame, then into the laser frame.
  const double c = std::cos(p0.yaw), s = std::sin(p0.yaw);
  const double dx = p1.x - p0.x, dy = p1.y - p0.y;
  Pose3d robot_inc = Pose3d::Identity();
  robot_inc.linear() = rf2o::matrixYaw(std::remainder(p1.yaw - p0.yaw, 2.0 * M_PI));
  robot_inc.translation() << c * dx + s * dy, -s * dx + c * dy, 0.0;
  const Pose3d laser_inc =
    rf2o_ref.laser_pose_on_robot_inv_ * robot_inc * rf2o_ref.laser_pose_on_robot_;

  rf2o_ref.kai_loc_old_(0) = laser_inc.translation()(0);
  rf2o_ref.kai_loc_old_(1) = laser_inc.translation()(1);
  rf2o_ref.kai_loc_old_(2) = rf2o::getYaw(laser_inc.rotation());
  return true;
}


/**
 * This function is used to initialize the robot pose before estimating its odometry.
 * By default the odometry will start from pose_0, but when comparing different methods
 * it may be necessary to start from a different pose.
*/
void CLaserOdometry2DNode::initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose)
{
  // Initialize module on first GT pose. Else do Nothing!
  if (!GT_pose_initialized)
  {
    initial_robot_pose = *new_initPose;
    GT_pose_initialized = true;
  }
}


/**
 * Publish current odocmetry estimation over ROS
 * According to the node parameters it will publish over tf and/or especified topic
*/
void CLaserOdometry2DNode::publish()
{
  // 1. publish odom as a topic (no harm!)
  RCLCPP_DEBUG(get_logger(), "Publishing odom over topic:[%s]", odom_topic.c_str());
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(0.0, 0.0, rf2o::getYaw(rf2o_ref.robot_pose_.rotation()));
  geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf_quaternion);
  
  // compose odom msg
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
  odom.header.frame_id = odom_frame_id;
  //set the position
  odom.pose.pose.position.x = rf2o_ref.robot_pose_.translation()(0);
  odom.pose.pose.position.y = rf2o_ref.robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternion;
  //set the velocity
  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = rf2o_ref.lin_speed;    //linear speed
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = rf2o_ref.ang_speed;   //angular speed

  // Report real measurement uncertainty. Both covariance matrices are 6x6
  // row-major over (x, y, z, roll, pitch, yaw), so the variance for each
  // axis sits on the diagonal at index 7*i.
  //
  // Leaving these all-zero (the default-constructed state, and what this
  // node did previously) is not neutral: a variance of exactly zero reads
  // as "infinitely certain" to a Kalman filter, so a consumer such as
  // robot_localization's ekf_node has no basis on which to weigh this
  // estimate against any other source.
  //
  // This is a planar scan matcher, so only x/y/yaw are actually observed.
  // The unobserved z/roll/pitch axes get a large variance rather than zero
  // so that a consumer configured to fuse them treats them as
  // uninformative instead of perfectly known.
  const double UNOBSERVED = 1e6;
  odom.pose.covariance[0]  = position_covariance;   // x
  odom.pose.covariance[7]  = position_covariance;   // y
  odom.pose.covariance[14] = UNOBSERVED;            // z
  odom.pose.covariance[21] = UNOBSERVED;            // roll
  odom.pose.covariance[28] = UNOBSERVED;            // pitch
  odom.pose.covariance[35] = yaw_covariance;        // yaw

  odom.twist.covariance[0]  = linear_velocity_covariance;   // vx
  odom.twist.covariance[7]  = linear_velocity_covariance;   // vy
  odom.twist.covariance[14] = UNOBSERVED;                   // vz
  odom.twist.covariance[21] = UNOBSERVED;                   // vroll
  odom.twist.covariance[28] = UNOBSERVED;                   // vpitch
  odom.twist.covariance[35] = angular_velocity_covariance;  // vyaw

  //publish the message
  odom_pub->publish(odom);

  // 2. publish over tf? (one one node should publish this transform!)
  if (publish_tf)
  {
    RCLCPP_DEBUG(get_logger(), "Publishing TF: [base_link] to [odom]");
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = rf2o_ref.robot_pose_.translation()(0);
    odom_trans.transform.translation.y = rf2o_ref.robot_pose_.translation()(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = quaternion;
    //send the transform
    odom_broadcaster->sendTransform(odom_trans);
  }
}

} /* namespace rf2o */


//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // Scan-driven: LaserCallBack does all the work, so there is no loop rate.
  rclcpp::spin(std::make_shared<rf2o::CLaserOdometry2DNode>());
  rclcpp::shutdown();

  return 0;
}