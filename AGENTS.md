# rf2o_laser_odometry: agent notes

Vendored fork of
[MAPIRlab/rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry),
scan-to-scan planar odometry. **This repo's default branch is `ros2`, not
`main`** — commit there. Upstream's `README.md` describes the range-flow
algorithm; the paper it cites is the reference for anything in
`CLaserOdometry2D.cpp`.

## How the Sentry actually uses it

Launched only by `../sentry_localization`'s `localization.launch.py`, and only
when `use_ekf:=true`. It reads `/scan`, publishes `/scan_odom`, and
`publish_tf` is **false** — `robot_localization`'s EKF owns `odom->root`, not
this node. `base_frame_id` is `root`, `freq` is 20 Hz.

Why each of those values is what it is, and the measurement history behind them,
lives in `../sentry_localization/README.md` (`## Notes`). Read that before
retuning anything here.

**This package is shadowed by `/workspaces/ros2_ws`** (`Dockerfile.thornbots`
LAYER 5 copies this directory in at build time). Once it's built locally, an
edit under `src/rf2o_laser_odometry` is live under `dexec.sh` but _not_ in the
user's terminal, which resolves to the image-baked snapshot. Before trusting any
result:
`../isaac_ros_common/scripts/dexec.sh -- ros2 pkg prefix rf2o_laser_odometry`
(`/workspaces/isaac_ros-dev/…` = your edit is live).

## Thornbots changes to upstream

- **Re-query the `laser -> base_frame_id` transform every scan** (`7a07a0f`).
  Upstream sampled it once on the first scan, which is fine for a fixed lidar
  and wrong for the Sentry's head-mounted one.
- **Publish real covariance** (`78d6a05`). Upstream left `pose.covariance` and
  `twist.covariance` all-zero, which a Kalman filter reads as "infinitely
  certain." The diagonal now comes from four parameters — `position_covariance`
  (default `0.02**2`), `yaw_covariance` (`0.05**2`),
  `linear_velocity_covariance` (`0.05**2`), `angular_velocity_covariance`
  (`0.1**2`) — with the unobserved z/roll/pitch axes set to `1e6`.
- **Dropped the `cmake_modules` dependency** (`c076912`), which isn't packaged
  for Humble.

## Scope

The odometry estimator and its message contract. How `/scan_odom` is weighted
against wheel odometry, and every EKF/AMCL/SLAM parameter, belong to
`../sentry_localization`. The `/scan` this consumes is produced by
`../thornbots_pkg`'s `lidar_self_filter`, not by the raw driver.

## Open

- **The covariance defaults are guesses, not measurements.** They were picked to
  be roughly comparable to wheel encoders, never validated against the drift
  suite in `../sim/test/localization/`. Either measure them or expose them from
  `sentry_localization`'s launch so they can be tuned without a rebuild.
