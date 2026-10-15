# rf2o_laser_odometry

Estimation of 2D odometry based on planar laser scans. rf2o is a fast and precise method to estimate the planar motion of a lidar from consecutive range scans. Useful for mobile robots with inaccurate wheel odometry.

For every scanned point we formulate the range flow constraint equation in terms of the sensor velocity, and minimize a robust function of the resulting geometric constraints to obtain the motion estimate. Conversely to traditional approaches, this method does not search for correspondences but performs dense scan alignment based on the scan gradients, in the fashion of dense 3D visual odometry.

The very low computational cost (0.9 milliseconds on a single CPU core) together whit its high precision, makes RF2O a suitable method for those robotic applications that require planar odometry.

For a full description of the algorithm, please refer to: **Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA 2016** Available at: http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf

## Strict scan consumption

`strict_capture` defaults to `false`, preserving the normal scan callback and
latest-transform fallback. When enabled, scans are processed FIFO only after
the exact `base_frame_id <- scan.header.frame_id` transform is available. After
initialization, a configured `odom_prior_topic` must also bracket both scan
stamps. Scans below `minimum_scan_stamp_s` are skipped; pending prerequisites
have a 2 s wall timeout and the queue is limited to 100 scans.

Set `consumed_topic` to publish one `diagnostic_msgs/DiagnosticArray` receipt
per initialized, matched, or skipped scan. Its header preserves the scan
header, and its single status is named after this node with message `published`
when odometry with that exact stamp was emitted, otherwise `no_output`. This
receipt can gate consumers that need to wait for scan-time odometry.
