# rf2o_laser_odometry

The first scan fixes the matcher's angular grid. Later scans must have the
same range count, `angle_min`, `angle_increment` and `angle_max` (exact
message values). A mismatch is skipped with a throttled warning before
mapping ranges or updating scan history, timestamps, priors or published
odometry. Configure the driver to provide a fixed grid; the sentry launch
enables angle compensation for this reason.

Estimation of 2D odometry based on planar laser scans. rf2o is a fast and precise method to estimate the planar motion of a lidar from consecutive range scans. Useful for mobile robots with inaccurate wheel odometry.

For every scanned point we formulate the range flow constraint equation in terms of the sensor velocity, and minimize a robust function of the resulting geometric constraints to obtain the motion estimate. Conversely to traditional approaches, this method does not search for correspondences but performs dense scan alignment based on the scan gradients, in the fashion of dense 3D visual odometry.

The very low computational cost (0.9 milliseconds on a single CPU core) together whit its high precision, makes RF2O a suitable method for those robotic applications that require planar odometry.

For a full description of the algorithm, please refer to: **Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA 2016** Available at: http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf

## Scan-time startup

When `init_pose_from_topic` and `odom_prior_topic` name the same nonempty
odometry topic after ROS namespace resolution and remapping, the first
accepted scan initializes from its interpolated odometry pose at that scan's
stamp. A distinct init topic keeps its first
latched pose as the explicit origin; an empty init topic starts at zero.

Scans wait in arrival order for scan-time TF and, when configured, odometry
coverage. The executor retries without blocking subscriptions. After
initialization a scan waits until scans 1 s newer queue: it then runs
without the odometry prior, or on the latest TF within `max_extrinsic_age`
(older is skipped as `extrinsic_stale` when `confidence_enabled`), so a
stalled `/odom` or joint stream degrades instead of stopping rf2o. The queue
holds at most 100 scans and two seconds of scan stamps, dropping the oldest
on overflow; scans older than available odometry history are discarded.
Odometry endpoints are never held beyond their stamps. The previous matched
odometry pose is retained for fallback across a dropped-scan gap. Duplicate
and backward scan/odometry stamps are ignored; restart the node after a clock
reset. Invalid solves and covariances fail before updating the pose.

Initialization builds the first scan's pyramid. If a later match fails and
wheel fallback cannot advance the pose, its candidate pyramid is discarded.
The accepted pose, scan, head extrinsic, timestamp and cached odometry pose
remain together; recovery spans the full interval from that reference.
