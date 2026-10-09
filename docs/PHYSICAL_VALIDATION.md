# Physical validation checklist

Local contract/replay/NT4 loopback tests do not qualify a camera, Jetson, controller,
calibration, source-loss response, motion behavior or end-to-end latency. Robot-code
integration, deployment and hardware qualification are outside this software check.

For hardware qualification, record software commits, producer
package versions, controller toolchain/image, camera mode, exposure, resolution,
measured mount, public geometry revisions and the test method/results.

- **Capture correction:** measure exposure-to-host-read-complete delay and its
  spread at each actual camera/USB/decode mode. Keep correction verification false
  and uncertainty null until measured. A measured zero correction is legitimate;
  a configured nonzero correction is not evidence. Retest after mode/exposure changes.
- **Calibration and frames:** independently verify intrinsics/distortion at the
  capture resolution; check known tag/target positions, metric scale and corner
  order. Survey mount and fixed field origin. Verify NWU, meters, WXYZ, camera tx
  right-positive, ty up-positive and robot yaw left-positive. Revision hashes only
  identify public configuration; they do not prove physical accuracy.
- **Object/POI geometry:** validate target height relative to robot origin, box or
  segmentation anchor bias, range error and covariance calibration using team
  data. Check ambiguity/calibration gates and missing-mount camera-only POI.
  Verify valid POI/object geometry with rejected localization. Boxes are approximate
  plane intersections; no field tracking or odometry-derived POI is supplied.
- **Motion:** measure distortion/blur and capture-age error over the operating
  velocity/angular-rate envelope. Leave motion_compensated and path_validated
  false. Validate any future capture-time robot transforms/pose fusion separately;
  this checklist authorizes no autonomous motion.
- **Loss/restart:** under a safe separately approved hardware setup, test camera
  unplug, frozen frames, producer power loss, network disconnect/reconnect, retained
  NT topics, watchdog, reload and new boot. Confirm same-frame larger-sequence
  invalidation and independent per-source receipt expiry; another source must not
  keep lost geometry actionable. Require fresh session evidence after reconnection.
- **Clock alignment:** compare NT server time to the selected controller clock,
  including restart, offset loss and disconnect transition. JSON remains us;
  2027 alpha-7 Java sample metadata is ns and needs its own adapter. Verify units
  on actual toolchain/image before capture-time integration. Wall time is logging
  metadata. WPILib 2026 + Systemcore has no official supported target in the checked
  compatibility matrix.
- **Worst-case load:** measure complete camera-to-consumer median/tails, dropped
  frames, stale suppression, CPU/GPU/RAM, power and thermal behavior with the
  actual number of workers, detection/segmentation, POI/MultiTag, preview controls
  and networking. Run resource-heavy tests in the vision environment with
  an explicit resource budget.

Store measured artifacts privately without credentials/paths in public packets.
Select receipt expiry and quality thresholds from these results, not synthetic
compute benchmarks. No hardware item above was run by this implementation task.
