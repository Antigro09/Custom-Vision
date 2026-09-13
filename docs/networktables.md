# Robot NetworkTables contract

The Jetson is an NT4 client of team 1086's roboRIO (or `networktables.server` when
explicitly supplied). Each pipeline publishes under `/CustomVision/<name>`.

| Topic | Type | Meaning |
|---|---|---|
| `result` | string | One complete JSON packet; use this to keep one frame coherent |
| `connected` | boolean | Pipeline has a fresh usable camera frame; not NT connectivity |
| `has_target` | boolean | At least one detection in the current packet |
| `frame_id` | integer | Sequence since process start; resets at restart |
| `count` | integer | Number of current detections |
| `latency_ms` | double | Host frame-read completion to publication |
| `tag_ids` | integer array | Current AprilTag IDs, empty for object pipeline/failure |

The JSON includes `schema_version:1`, `pipeline`, `type`, `connected`, `frame_id`,
`capture_monotonic_us`, `publish_unix_us`, `latency_ms`, `detections`, and `error`.
Detection fields are documented in the pipeline guides. Convenience topics are
published separately and are not an atomic snapshot; parse `result` to correlate
arrays and poses.

**Time and freshness:** `capture_monotonic_us` is Jetson monotonic time at completion
of `VideoCapture.read()`. It is not sensor exposure time or roboRIO FPGA time.
`publish_unix_us` is wall-clock logging metadata. Do not pass either directly to
WPILib pose-estimator latency compensation. A timestamp conversion and measured
capture latency are required for odometry integration.

The Jetson emits an empty result on read failure, shutdown, or a 500ms frame
watchdog timeout. The robot must also expire packets using its own local receipt
time: a powered-off Jetson cannot clear already-published NT values. A sensible
starting receipt-age limit is 250ms; tune only after measuring end-to-end latency.
On disconnect, age timeout, `connected:false`, invalid pose, or empty detections,
stop using the previous target. Repeated valid empty results mean no target.

A Java consumer can use the standard WPILib subscriber API:

```java
var resultSub = NetworkTableInstance.getDefault()
    .getStringTopic("/CustomVision/front_tags/result").subscribe("");
// In robotPeriodic, process queued new samples (not the last cached string):
for (var sample : resultSub.readQueue()) {
    // Parse JSON; validate schema, connected, and pose_valid.
    // Record Timer.getFPGATimestamp() as local receive time.
    // Reset validity on every empty/invalid sample.
}
// Invalidate if no new result was received within the configured receipt-age limit.
```

This is an integration sketch, not a complete robot subsystem. No autonomous
motion or field pose integration is included. AprilTag poses are tag-relative to
camera in OpenCV axes: x right, y down, z forward. Object bearings use yaw right
positive and pitch up positive. Do not mix these with WPILib NWU axes without an
explicit transform. Robot-to-camera extrinsics and the confirmed game field tag
layout are separate required inputs for field localization.
