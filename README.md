esstimator (SIQ)
================

esstimator is a fork of robot_localization that provides nonlinear state estimation nodes. The original package was developed by Charles River Analytics, Inc.

Please see the upstream robot_localization documentation here: http://wiki.ros.org/robot_localization

## Tuning Telemetry And Visualizer

For the new EKF/UKF tuning window and practical tuning workflow, see [TUNING_VISUALIZER_GUIDE.md](TUNING_VISUALIZER_GUIDE.md).

The tuning visualizer treats each fused input branch as its own stream. For example, an IMU can appear as separate orientation, angular velocity, and linear acceleration inputs, each with its own covariance, innovation, and Mahalanobis behavior. The UI also shows the corresponding ROS topic name for each configured stream and keeps inactive configured streams visible as gray `[no data]` panels.

### Parameters

Enable the diagnostics you want in the filter YAML:

```yaml
mahalanobis_publish_rate: 10.0
tuning_visualizer_enabled: true
tuning_visualizer_history_seconds: 60.0
tuning_telemetry_enabled: true
tuning_telemetry_publish_rate: 5.0
```

- `mahalanobis_publish_rate`: publishes the latest Mahalanobis distance for each configured stream on `/mahalanobis/<stream_name>`. Set to `0.0` or less to disable periodic publishing. Rejected measurements are still published immediately.
- `tuning_visualizer_enabled`: publishes the visualizer diagnostics under `/tuning/*` and attempts to auto-launch the matplotlib tuning window when a desktop display is available.
- `tuning_visualizer_history_seconds`: rolling time window shown by the visualizer.
- `tuning_telemetry_enabled`: publishes periodic `/tuning/*` snapshots intended for rosbag capture and offline analysis.
- `tuning_telemetry_publish_rate`: periodic snapshot rate for the telemetry topics. Set to `0.0` or less to disable periodic publishing even when telemetry is enabled.

### Topics

- `/mahalanobis/<stream_name>` (`esstimator/msg/StampedFloat64`): latest branch Mahalanobis distance for each configured stream. Published periodically when `mahalanobis_publish_rate > 0.0`, with rejected measurements also published immediately, and stamped with the latest measurement time.
- `/tuning/innovation_diagnostics` (`esstimator/msg/InnovationDiagnostic`): per-update measurement, prediction, innovation, covariance diagonals, threshold, and accept/reject result for the visualizer. Published when `tuning_visualizer_enabled` is `true`.
- `/tuning/filter_state` (`esstimator/msg/FilterStateDiagnostic`): current filter state together with estimate covariance and process noise diagonals. Published when `tuning_visualizer_enabled` is `true`.
- `/tuning/configured_streams` (`std_msgs/msg/String`): newline-separated `stream_name<TAB>source_topic<TAB>state_indices` entries for every configured stream. Published when either `tuning_visualizer_enabled` or `tuning_telemetry_enabled` is `true`.
- `/tuning/stream_telemetry` (`esstimator/msg/TelemetrySnapshot`): periodic per-stream gate metrics, innovation norms, per-component units, and per-component innovations. Published when `tuning_telemetry_enabled` is `true`.
- `/tuning/branch_fusion_status` (`esstimator/msg/BranchStatusSnapshot`): periodic branch-level fused/not-fused snapshots. Published when `tuning_telemetry_enabled` is `true`.
- `/tuning/component_fusion_status` (`esstimator/msg/ComponentStatusSnapshot`): periodic per-component fused/not-fused snapshots. Published when `tuning_telemetry_enabled` is `true`.
- `/tuning/branch_fused/<stream_name>` (`esstimator/msg/StampedUInt8`): PlotJuggler-friendly per-stream fused flag, published as `0` or `1` with `header.stamp` set to the latest measurement time for that stream.
- `/tuning/gate_metric/<stream_name>` (`esstimator/msg/StampedFloat64`): PlotJuggler-friendly per-stream Mahalanobis gate metric, stamped with the latest measurement time.
- `/tuning/gate_threshold/<stream_name>` (`esstimator/msg/StampedFloat64`): PlotJuggler-friendly per-stream rejection threshold, stamped with the latest measurement time.
- `/tuning/innovation_norm/<stream_name>` (`esstimator/msg/StampedFloat64`): PlotJuggler-friendly per-stream innovation norm, stamped with the latest measurement time.

With:

```yaml
tuning_visualizer_enabled: false
tuning_telemetry_enabled: true
```

the expected `/tuning/*` outputs are only:

- `/tuning/configured_streams`
- `/tuning/stream_telemetry`
- `/tuning/branch_fusion_status`
- `/tuning/component_fusion_status`

`/tuning/innovation_diagnostics` and `/tuning/filter_state` are not published unless `tuning_visualizer_enabled` is `true`.

`/tuning/stream_telemetry` only includes streams that have already produced at least one Mahalanobis result. `/tuning/branch_fusion_status` and `/tuning/component_fusion_status` include every configured stream, using `fused: false` until that stream has produced live gating data.

For PlotJuggler specifically, prefer the flat per-stream topics under `/tuning/branch_fused/<stream_name>`, `/tuning/gate_metric/<stream_name>`, `/tuning/gate_threshold/<stream_name>`, and `/tuning/innovation_norm/<stream_name>`. They avoid the nested array-of-struct layout that many viewers collapse to `streams[0]`, and they carry `header.stamp` so bag playback can align them to the originating measurement time instead of recorder receive time.

### ROS 2 CLI Examples

```bash
ros2 topic list | grep -E '(^/tuning/|^/mahalanobis/)'
ros2 topic info /tuning/stream_telemetry
ros2 topic echo /tuning/configured_streams --once
ros2 topic echo /mahalanobis/<stream_name>
```

```bash
ros2 bag record \
  /tuning/innovation_diagnostics \
  /tuning/filter_state \
  /tuning/configured_streams \
  /tuning/stream_telemetry \
  /tuning/branch_fusion_status \
  /tuning/component_fusion_status \
  /mahalanobis/<stream_name>
```

If you want every tuning topic in one bag, enable both `tuning_visualizer_enabled` and `tuning_telemetry_enabled`.

For Mahalanobis topics, replace `<stream_name>` with the concrete stream names reported in `/tuning/configured_streams`, for example:

```bash
ros2 bag record \
  /tuning/configured_streams \
  /tuning/stream_telemetry \
  /tuning/branch_fusion_status \
  /tuning/component_fusion_status \
  /mahalanobis/imu0_pose \
  /mahalanobis/odom0_pose \
  /mahalanobis/odom1_pose \
  /mahalanobis/twist0
```

### Interpretation Notes

- The true gate decision is the branch-level Mahalanobis test in `/mahalanobis/<stream_name>`, `/tuning/innovation_diagnostics`, and `/tuning/branch_fusion_status`. The per-component values in `/tuning/stream_telemetry` and `/tuning/component_fusion_status` are useful proxies, but they are not the exact multivariate branch gate when a stream fuses multiple components.
- Angular values in the periodic telemetry snapshots are converted to display units: `deg` for orientation and `deg/s` for angular velocity. Linear quantities remain in `m`, `m/s`, and `m/s^2`.
