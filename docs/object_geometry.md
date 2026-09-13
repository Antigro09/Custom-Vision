# Calibrated object position and association

`ObjectGeometry` turns detections into approximate robot-relative positions at
**capture time**. It uses a measured camera mount, lens calibration at the exact
capture resolution, and an explicitly chosen target height. These inputs are
required; missing inputs retain the 2D detections and return a reason instead of
inventing a metric position. This is geometric ranging, not learned monocular
depth. No camera, game-piece model, or autonomous pickup accuracy is implied by
the synthetic software tests.

## Coordinate and height contract

Robot coordinates follow WPILib NWU: +X forward, +Y left, +Z up. Mount
`rotation_rpy_deg` uses `Rz(yaw) Ry(pitch) Rx(roll)`; positive pitch turns a
forward camera downward. OpenCV's optical right/down/forward axes are converted
once, before applying the measured mount. A rear-facing camera can correctly
produce negative robot X; this is different from an intersection behind its
optical ray.

For an undistorted pixel ray `d` expressed in robot coordinates and camera origin
`c`, the target point is `c + ((target_height_m - c.z) / d.z) * d`. The configured
plane is expressed relative to the same robot origin as the mount; using a ball
radius directly assumes robot Z=0 lies on the floor. Otherwise include the floor
height offset. For example, if robot Z=0 is 0.20 m above the floor, a 0.05 m
radius ball uses `target_height_m: -0.15`. Signed target-plane heights from -3 to
+3 m are supported, and the camera must be above that plane. The plane assumes a
level floor and chassis. It does not compensate for robot
pitch/roll, floor slopes, elevated pieces, or a moved mount. A time-aligned
robot-attitude or odometry integration is future work.

Use `bbox_center` with the **known ball radius** as `target_height_m` for a first
ball-center estimate. For `bbox_bottom` or `mask_bottom`, use zero only when that
anchor is a reasonable approximation of the floor contact. A silhouette's
bottommost visible pixel is not necessarily the true 3D contact point. A projected
sphere silhouette center also differs from the projection of its 3D center at
close range. All current anchors are therefore explicitly labeled approximate.
Do not use a single plane for mixed classes that have materially different
heights. Filter to the intended class or configure separate pipelines.

Supported anchors:

- `bbox_center`: center of `bbox_xyxy` in the original capture image.
- `bbox_bottom`: bottom-center of that box; a simple contact approximation.
- `mask_centroid`: `segmentation.centroid_px` from the actual instance mask.
- `mask_bottom`: `segmentation.bottom_px` from that mask.

A missing requested mask anchor is rejected; the implementation never silently
switches it to a box anchor. Preview rotation and letterbox dimensions never
replace original capture coordinates. Clipped boxes are rejected by default.

## Configuration

Place these settings in an object's pipeline-level `geometry` block, alongside
its `settings`, `camera`, `calibration`, and `robot_to_camera` fields:

```yaml
geometry:
  target_height_m: null        # Required measurement: e.g. actual ball radius.
  anchor: bbox_center
  min_range_m: 0.05
  max_range_m: 5.0             # Tune from measured error over your intake range.
  min_downward_angle_deg: 8.0
  pixel_std_px: 2.0
  height_std_m: 0.015          # Combined camera-to-target height separation error.
  pitch_std_deg: 0.5
  mount_xy_std_m: 0.01
  max_position_std_m: 0.30
  reject_clipped_boxes: true
  max_targets: 100
  tracking_gate_m: 0.30
  tracking_ttl_s: 0.15
  selection_hysteresis_m: 0.20
  intake_offset_m: [0.0, 0.0] # Measured robot-forward, robot-left intake position.
  approach_standoff_m: 0.10
```

These numerical defaults are initial limits, **not validated accuracy claims**.
Unknown setting names and nonfinite/out-of-range values are errors. Set
`target_height_m` only after the actual game piece and anchor are known.

Pixel uncertainty is propagated through lens undistortion and the plane
intersection using central finite differences. Height separation, mount pitch,
and XY mount uncertainty are added to the XY covariance under an independent
error assumption. The output includes XY covariance, XY standard deviations,
range and bearing standard deviations, and the largest principal position
standard deviation. Near-horizon rays, behind-camera intersections, invalid
ranges and excessive predicted position uncertainty are rejected. An XY position
at the robot origin is also rejected because its bearing is undefined, even if
the configured minimum range is zero. The model does
not include all lens calibration, mask bias, chassis attitude, or correlated
errors; validate its estimates against measured floor coordinates.

## Fresh targets and tracking

Tracking associates only matching `(class_id, label)` detections inside a bounded
XY distance gate. It remembers IDs for at most `tracking_ttl_s`; at most twice
`max_targets` tracks are retained, keeping work bounded. Detection order changes
do not change a clear nearest association. Closely crossing, indistinguishable
objects can exchange IDs. IDs are local to a pipeline/runtime boot and must be
used together with the runtime's boot ID.

Robot-relative motion combines robot movement and object movement. Without
odometry this implementation deliberately publishes **no velocity, prediction,
or field map**. A target missing in the current frame is immediately absent from
`targets` and cannot remain selected, even if its association ID survives briefly.
Measurements are not smoothed into an old position. A non-increasing capture
timestamp invalidates the frame and clears association. `reset()` clears tracks
on a camera/runtime failure without reusing IDs.

The nearest currently observed target to the configured intake is selected.
`selection_hysteresis_m` keeps the prior target when another target is only
slightly closer. The suggested `approach.translation_m` is a displacement from
the robot at capture time that preserves its current heading and accounts for the
intake offset and standoff. It is **not** a field waypoint, obstacle check,
kinematically validated path, or robot movement command. Java integration remains
deferred.

## Python and NetworkTables result shape

```python
geometry = ObjectGeometry(settings, calibration, robot_to_camera)
result = geometry.enrich(detections, frame.shape, capture_monotonic_s)
geometry.reset()  # Camera disconnect or runtime invalidation.
```

The Python return value and browser `/api/status` retain expanded object
measurements. Their `objects` dictionary contains:

- `valid`, `targets`, `selected_target` (or null), `selected_track_id` (or null).
- `capture_monotonic_us`, `invalid_reason`, and `motion_compensated: false`.
- Frame name `robot_relative_at_capture_wpilib_nwu`.

Each valid target contains `detection_index`, `track_id`, class metadata,
`translation_m`, `range_xy_m`, `bearing_deg` (+left), `range_from_intake_m`,
`anchor_px`, `anchor`, `approximate`, `uncertainty`, `approach`, capture timestamp,
`track_age_ms`, `track_observations`, `observed: true`, and `predicted: false`.
In the Python return value and browser status, the raw detection's
`robot_relative` field references this same expanded target. Invalid detections
receive `{valid: false, invalid_reason: ...}`.

For the coherent NetworkTables `result` JSON, the publisher sends each complete
metric target once in `objects.targets`. Resolve `objects.selected_track_id` in
that list; the duplicated `objects.selected_target` field is omitted on the wire.
A valid detection's wire `robot_relative` is only `{valid: true, track_id: ...}`,
pointing to its corresponding target. Invalid reasons are preserved. This keeps
robot messages compact without losing measurements. [The NetworkTables contract](networktables.md)
also documents typed selection topics. A missing pose never inherits coordinates
from an earlier frame. Capture clock conversion remains the publisher's
responsibility.

Tests cover independent 3D-to-image projection, distortion, mount roll/pitch/yaw,
front/rear and left/right conventions, unknown calibration/height/mount,
near-horizon and range rejection, uncertainty, mask anchors, clipped detections,
track order/TTL/reset/class gating, timestamp rejection, intake offset selection,
and concurrent reset behavior. Real calibration and trained-model tests are
still required for useful robot measurements.
