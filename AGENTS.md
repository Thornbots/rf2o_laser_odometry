# rf2o_laser_odometry

Follow [workspace rules](../AGENTS.md) and [CI](../docs/CI.md).
Keep the diff to [upstream](README.md) small. Its range-flow paper is the
algorithm reference. Read [Sentry tuning rationale](../sentry_localization/README.md#notes)
before changing the estimator.

## Scope

Own scan odometry and match grading. `sentry_localization` owns fusion/tuning
and `odom->root`; `thornbots_pkg` supplies the filtered scan.
Preserve the intentional upstream diffs: live head extrinsics, per-scan
callback matching at scan stamps, real covariances, `fixed_heading`,
`odom_prior_topic`, no republish after a failed match, and grading from matcher
evidence, never agreement with the wheel-odometry prior.
Parameter values live in source and `../sentry_localization/config/rf2o.yaml`.

## Testing

Use noisy synthetic scans: identical noiseless scans yield zero derivatives
and NaN weights. See `test/test_match_quality.cpp` and
[quality recording](../sim/README.md#more-on-the-tests).

## Open

- Covariances are estimates; grade thresholds have drift-suite evidence.
- `package.xml` is deprecated format 1; migration to format 3 is a separate change.
