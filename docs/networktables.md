# NetworkTables contract, schema 2

The Jetson is an NT4 client of team 1086's roboRIO or an explicit configured server.
The default topic root is `/CustomVision/jetson-tags/<pipeline>`. Give each Jetson a
unique root. This protocol borrows familiar vision concepts but **is not compatible
with PhotonLib or Limelight helpers without a custom adapter**. Java robot code is
intentionally deferred until camera and robot testing.

| Topic | Type | Meaning |
|---|---|---|
| `result` | string | Complete coherent JSON snapshot; preferred robot interface |
| `connected` | boolean | Fresh camera frame, not NT connectivity |
| `has_target`, `count` | boolean, integer | Current detections |
| `frame_id` | integer | Sequence, resets at runtime restart |
| `latency_ms` | double | Host read completion through publication preparation |
| `tag_ids` | integer[] | IDs in this result |
| `pose_valid` | boolean | Valid **field robot** pose, including measured mounting |
| `field_to_robot` | double[] | `[x,y,z,qw,qx,qy,qz]`, meters, or empty when invalid |
| `used_tag_ids` | integer[] | IDs used for the valid field robot pose |
| `capture_server_us` | integer | Estimated frame time in NT server clock, 0 if unavailable |
| `time_sync_valid` | boolean | NT clock offset was available for this frame |

JSON floating-point values are rounded to six decimal places to reduce wire size;
integer timestamps retain full precision. Host-side geometry stays full precision.
Typed topics are independent updates. Use `result` to associate poses, IDs, quality
and time with exactly one frame. Publication requests 10 ms periodic updates,
`sendAll`, duplicate preservation and a flush; consumers must also request an
appropriate periodic interval and read new queued samples. Flush does not guarantee
zero networking latency. [WPILib NetworkTables](https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-intro.html).

## Result contents

Every packet contains `schema_version:2`, unique `boot_id`, `pipeline`, `type`,
`mode`, `backend`, `detector_device`, `input_kind`, `connected`, `frame_id`,
`capture_monotonic_us`, `publish_unix_us`, `latency_ms`, `capture_server_us`,
`time_sync_valid`, `timestamp_source`, `capture_latency_offset_ms`, `detections`,
and `error`. Normal frames also carry `frame_size`, `processing_ms`, `detector_ms`,
`localization_ms`, `queue_ms`, `native_timings`, `fps` and `dropped_frames`.
Diagnostic fields can be absent on failure packets; consumers must tolerate that.

AprilTag detections include ID, corrected bits, decision margin, decoded corners,
center, area, yaw and pitch, pose validity and rejection reasons. Metric target
transforms use `camera_to_target` and, with measured mounting, `robot_to_target`.
`localization` contains validity, method, `field_to_camera`, `field_to_robot`,
used/rejected IDs, ambiguity, reprojection error and rejection reason. Pose objects
use meters, quaternion **WXYZ**, and a `frame: wpilib_nwu` label. Raw OpenCV
`tvec_m`/`rvec_rad` are retained as diagnostics and use different axes.

Standardized AprilTag transforms use WPILib NWU: **+X forward, +Y left, +Z up**.
AprilTag `yaw_deg` is positive left and `pitch_deg` positive up. Do not assume
PhotonVision pixel-yaw signs or the unchanged legacy object pipeline's signs match.
The blue-origin field coordinates supplied in the field file remain fixed across
alliances; alliance-dependent path handling belongs on the robot.
[Coordinate and localization details](localization.md),
[WPILib coordinate conventions](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html).

## Time and freshness

Capture time currently means **completion of the host camera read**, after MJPEG
decode. It is not a hardware exposure timestamp. `capture_monotonic_us` belongs to
the Jetson monotonic clock; `publish_unix_us` is logging metadata. Neither is a
roboRIO FPGA timestamp.

When NTCore supplies a server clock offset, the publisher translates host frame age
into the NT server clock and subtracts `camera.capture_latency_offset_ms`. This
optional correction must be measured for the actual camera mode/exposure. Leave it
zero until measured. On a roboRIO-hosted NT server the server clock is suitable for
FPGA-time integration after verification; a desktop NT server is not the roboRIO
clock. Reject latency compensation when `time_sync_valid` is false. The timestamp
still has unmeasured capture delay until the physical correction is established.

The default watchdog clears results after 100 ms without fresh processed data.
Over-age computations cannot revive expired targets. Reconfiguration, disconnect
and shutdown send invalid/empty packets. The robot must separately expire results
using its own local receipt clock and NT connection status: a powered-off Jetson
cannot clear cached topics. Start with a 100 ms receipt timeout and validate it
against measured frame intervals and network jitter.

A future consumer should:

1. Read queued new `result` samples; validate schema and pipeline identity.
2. Track `(boot_id, frame_id)`; discard duplicates/out-of-order samples and reset
   state when boot ID changes.
3. Clear target and pose state on every invalid or empty result; never retain a
   previous pose because a new frame lacks one.
4. Reject stale receipt times and disconnected NT sessions independently.
5. For odometry, require valid field robot pose, synchronized corrected timestamp,
   reasonable field bounds and measured quality thresholds. Select pose-estimator
   uncertainty from distance, geometry and residuals after field testing.
6. For aiming, use a fresh `robot_to_target`, or document a calibrated 2D fallback.

No robot movement, autonomous commands or pose-estimator integration is enabled by
this repository. Future Java helpers can wrap this packet without forcing a
separate published library.
