# Normative fields, units and validity

`result` is the coherent JSON snapshot. Typed NT topics are independent updates
and cannot assemble a coherent packet. Float values are rounded to six decimal
places using Python `round(value, 6)` before compact JSON encoding; this is a
precision bound, not a promise of six printed trailing digits. Integer timestamps
are never converted through a float in serialization. Nonfinite JSON is rejected.

| Envelope field | Meaning / unit |
| --- | --- |
| `schema_version` | Integer `2`; additive fields do not change existing meanings. |
| `protocol_profile` | `custom-vision-schema2-2026.1` for this profile. |
| `boot_id` | Opaque runtime session identity. A new runtime/configuration epoch gets a new ID. |
| `pipeline` | Configured pipeline name. Source identity also includes its configured NT root. |
| `packet_seq` | Per `(source root, pipeline, boot_id)` publication counter; integer 0 through 2^53−1. |
| `frame_id` | Capture/processing identity, not publication sequence; invalidation can reuse it. |
| `connected` | A fresh camera frame is available. This is independent of NT connectivity. |
| `type`, `mode` | `apriltag` with `2d|3d`, or `object` with `detect|segment`. |
| `backend`, `detector_device`, `pose_device` | Actual pipeline execution diagnostics. They confer no validity by themselves. |
| `capture_monotonic_us` | Host frame-read completion in host monotonic microseconds. |
| `capture_server_us` | Estimated same capture event in NT server microseconds, or null. |
| `time_sync_valid` | True only when the version-aware clock adapter returned a usable synchronized value. |
| `publish_unix_us` | Unix microseconds for logging; never an estimator clock. |
| `timestamp_source` | `host_frame_read_complete`; not a hardware exposure event. |
| `capture_latency_offset_ms` | Configured correction subtracted from capture event time, in milliseconds. |
| `latency_ms` | Host read completion to publication preparation, in milliseconds. |
| `error` | Null on a normal frame; string describing an invalidation/error. |
| `calibration_revision`, `mount_revision`, `field_layout_revision` | Opaque nullable `sha256:<64 lowercase hex>` public geometry identifiers. |

`packet_seq` is allocated under the serialization/publication lock. The first
serialized packet is 0. Normal frames, empty frames, camera errors, watchdogs,
reload and shutdown all consume a sequence. A same-frame invalidation has a
larger sequence than the observation it invalidates. Repeated invalidation also
advances the sequence. An encoding rejected before publication does not consume
one. It must never wrap; exhaustion fails publication until a new producer boot.
At 120 publications/second, the numeric bound takes over 2 million years. This
bound preserves exact integers in common JSON parsers; timestamp fields permit
signed-64-bit nonnegative integers, so readers must use integer-preserving parsing.

Frame IDs and object track IDs remain local to source, pipeline and boot. A track
ID is brief association among current observations; it is not persistent physical
identity or a field-fixed prediction. A new boot resets consumer association.

## Timing metadata

When present, `timing` contains exactly these defined meanings (additional future
fields may be ignored):

| Field | Meaning |
| --- | --- |
| `clock_domain` | Constant `nt_server`; the domain of `capture_server_us`. |
| `timestamp_unit` | Constant `us`; JSON `_us` units stay microseconds. |
| `capture_event` | Constant `host_frame_read_complete`. |
| `capture_correction_verified` | Physical measurement acknowledged for this configured camera mode/correction. |
| `capture_correction_uncertainty_ms` | Nullable nonnegative measured uncertainty in milliseconds; null means unknown. |

Verification is independent of the numeric correction. A measured correction
can be zero. A nonzero configured correction can be unverified. A synchronized
server clock does not make exposure timing verified, and an acknowledged capture
correction does not synchronize a missing clock. Missing offsets, disconnection,
unsupported API versions, invalid clock values or unusable clocks result in
`capture_server_us: null` and `time_sync_valid: false`. Typed unavailable capture
time is 0 and must be interpreted with typed `time_sync_valid`.

No exposure, USB/decode delay, NT clock alignment, or correction uncertainty is
invented by the producer. Consumers that require physically corrected estimator
insertion must impose their own verified timing/uncertainty gates.

## Geometry revisions and configuration epochs

The deterministic `public-geometry-v1` method hashes UTF-8 compact, sorted-key,
ASCII JSON with `allow_nan=False` in the envelope
`{"format":"public-geometry-v1","kind":K,"parameters":P}`. The digest is SHA-256
with the `sha256:` prefix. Inputs use the production validators' normalized
numbers/defaults; negative float zero becomes positive zero. Field tags are sorted
by ID and equivalent quaternion signs are canonicalized by making the first
nonzero WXYZ component positive.

* `calibration`: only normalized width, height, distortion model, camera matrix,
  and distortion coefficients.
* `mount`: normalized robot-to-camera translation in meters and RPY in degrees.
* `field_layout`: normalized field dimensions and tag IDs/poses in the supplied
  fixed-origin field coordinates.

Missing inputs yield null. Filenames, source addresses, credentials, arbitrary
metadata, calibration quality and physical verification acknowledgements are
excluded. Revisions identify configuration, not a validated physical measurement.

Intrinsic, mounting or field-coordinate changes invalidate pending observations.
The current configuration reload path sends an invalidation and starts a new
`boot_id`; this is also required for source/camera changes, capture-mode/timing
correction changes, target-plane/anchor/covariance changes, and POI geometry or
quality-gate changes. Those latter settings are not represented by these three
hashes. No accepted packet may combine an old observation with a new configuration.

## Detection families and actionable gates

Each family is independent: valid object or POI geometry can coexist with invalid
field localization. Missing family means no actionable output for that family in
this snapshot. Empty list means no observations in this snapshot. Explicit
`valid:false` means the family failed its gates; geometry retained for preview or
diagnostics must not be used as an action. Null means unavailable, never identity
transform, zero pose or unknown-but-valid coordinates. Optional failure diagnostics
can be missing; consumers must clear state instead of reusing them.

AprilTag `corners`/`center` are pixels. Diagnostic OpenCV `tvec_m` is right/down/
forward meters and `rvec_rad` is axis-angle radians. `camera_to_target`,
`robot_to_target`, `field_to_camera`, and `field_to_robot` use meters, NWU
(+X forward, +Y left, +Z up) and WXYZ quaternions with `frame:wpilib_nwu`.
`yaw_deg` is left-positive and `pitch_deg` up-positive for AprilTags. A pose object
must also have a numerically usable normalized quaternion. `pose_valid` and quality
gates are separate: an ambiguous tag solve can expose diagnostic metric geometry
while localization and POI remain invalid. Residuals are pixels, not pose covariance.

`localization.valid` can identify a valid field-camera pose while
`field_to_robot` is null due to missing mount. Only connected, valid localization
with nonnull `field_to_robot` makes typed `pose_valid` true. The supplied field
origin stays fixed across alliances. There is no odometry substitute.

Independent `poi` uses a currently observed single-tag solve **before field
enrichment**, verified calibration and ambiguity/reprojection gates. It reports
`uses_odometry:false` and `uses_field_layout:false`. `offset_m` is tag WPILib
(out of printed front, right when viewing upright print, up), not optical axes.
`camera_translation_m` is OpenCV right/down/forward, camera `tx_deg` is right-positive
and `ty_deg` up-positive. Mounted `robot_translation_m` is NWU and
`robot_yaw_deg` is left-positive. Missing mount retains valid camera aiming with
null robot coordinates; it never invents an identity mount. A valid POI is not a
shooter model, heading command, field robot pose, or planned path.

Objects retain raw boxes/segmentation in `detections`. Object camera `yaw_deg`
retains its legacy right-positive meaning; robot `bearing_deg` is left-positive.
The one canonical metric list is `objects.targets`. A valid detection's compact
`robot_relative:{valid:true,track_id:I}` must resolve to exactly one canonical
target. Nonnull `objects.selected_track_id` must resolve in that same list. IDs
must be unique within it. The wire never duplicates the full `selected_target`;
the dashboard retains its expanded full-precision copy.

Object geometry is an approximate calibrated ray intersection with the configured
signed target-height plane. It preserves `anchor`, `anchor_px`, `target_height_m`,
`method`, `approximate:true`, configured first-order covariance in square meters,
range uncertainty in meters and bearing uncertainty in degrees. A bounding box is
not an exact 3D measurement. Consumers must check covariance is finite, symmetric
and usable; rounding can introduce sub-micrometer-scale numerical drift.
`observed:true`, `predicted:false`, `motion_compensated:false` and
`approach.path_validated:false` remain mandatory for this implemented capability.
Approach translation is meters at capture heading, not an obstacle-checked motion
command. Missing calibration, mount or target height keeps raw 2D detections and
produces no metric targets.

Errors publish empty observations and invalid localization/object/POI families,
clearing every actionable typed output. A normal empty frame also clears action
for absent detections. Source power loss may leave retained topics untouched;
consumers must independently expire them by receipt time and NT connection status.

## Consumer acceptance order

1. Validate root/source, pipeline, schema/profile, active NT session and boot
   eligibility. Validate coherent publication sequence and reject malformed or
   out-of-order publications. A retained old boot must not automatically replace
   a newer active boot.
2. Apply every accepted invalidation/empty/invalid family immediately, including
   repeated same-frame invalidation with larger `packet_seq`.
3. Deduplicate reusable measurements by `(source,pipeline,boot_id,frame_id)` only
   after validity/invalidation has been applied. It must not suppress clearing.
4. Check each family's prerequisites and synchronized/verified timing policy
   before using geometry. Validate selected/compact references against targets.
5. Expire each source independently using the consumer's local receipt clock,
   regardless of duplicate captures, timestamps, retained topics or other sources'
   liveness. A duplicate reusable measurement is not a new capture. Receipt
   liveness and eligibility do not justify re-inserting it into an estimator.
