# AprilTag targeting and field localization

Each camera runs independently. `apriltags.mode: 2d` returns IDs, decoded corners,
center, image area, yaw and pitch, and explicitly invalidates metric poses.
`mode: 3d` adds calibrated pose estimation. A measured `robot_to_camera` mount is
required for robot-relative targets. An uploaded field layout is required for
field localization. No season or camera mount is supplied by assumption.

Calibrate the **raw capture resolution** and measure the detected tag side in
meters. The default `.1651` m must be verified against the tags actually used.
Missing calibration or a resolution mismatch preserves 2D observations but
withholds metric geometry. A glued lens and nominal FOV do not replace camera
calibration; follow [the calibration guide](apriltags.md).

## Coordinates and robot mounting

Every named pose uses meters and WPILib NWU: +X forward, +Y left, +Z up. Roll,
pitch and yaw are right-hand rotations about X, Y and Z, respectively. The camera
origin is its optical center. The tag origin is its center, with +X out of the
printed face, +Y to the right as viewed by an observer, and +Z up. Consequently,
an upright tag directly in front of an upright camera has positive X translation
and 180 degrees yaw. These conventions match [PhotonVision's pose coordinates](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html).

An example **synthetic mount**, to replace with your measured values:

```yaml
robot_to_camera:
  translation_m: [0.30, 0.12, 0.50]
  rotation_rpy_deg: [0, -15, 0]
```

This means camera optical center 30 cm forward, 12 cm left, 50 cm up from your
chosen robot origin, pitched 15 degrees upward. Negative WPILib pitch rotates
forward toward up. The rotation is `Rz(yaw) @ Ry(pitch) @ Rx(roll)`.
An omitted/null mount never becomes an identity mount. Explicit zero mounting is
valid only if it describes the camera's actual position and orientation.

`A_to_B` describes B's pose in A coordinates; its matrix maps B-local points into
A. The relationships implemented and tested are:

```text
robot_to_target = robot_to_camera * camera_to_target
field_to_camera = field_to_tag * inverse(camera_to_target)
field_to_robot  = field_to_camera * inverse(robot_to_camera)
```

Each pose object includes `translation_m: [x,y,z]`,
`rotation_quaternion_wxyz: [w,x,y,z]`, `rotation_rpy_deg: [roll,pitch,yaw]` and
`frame: wpilib_nwu`. Quaternions avoid Euler singularities; prefer them when the
robot integration constructs a `Rotation3d`.

The raw `rvec_rad`/`tvec_m` fields remain available for OpenCV projection and
debugging. Their optical camera frame is +X right, +Y down, +Z forward, and
`rvec_rad` is Rodrigues axis-angle, **not Euler roll/pitch/yaw**. The detector's
decoded raw object corner order is
`[-h,+h,0], [+h,+h,0], [+h,-h,0], [-h,-h,0]`. For an upright rendered marker this
is printed TR, TL, BL, BR. Do not sort these corners by screen position.

`yaw_deg` is positive left and `pitch_deg` is positive up, based on the center's
undistorted camera ray. Pitch uses ray elevation relative to the horizontal
plane. This yaw sign is deliberately documented separately from PhotonVision's
target-yaw convention; the protocols are not interchangeable. `area_pct` is the
detected quadrilateral's percentage of raw image area. Without matching
calibration the angles use a pinhole approximation with nominal FOV and carry
`angle_source: nominal_fov_approximate`. Nominal FOV does not produce range.

## Single tags, MultiTag, and ambiguity

Single-tag PnP evaluates both planar solutions, rejects negative depth, and
reports distorted-pixel RMS reprojection error. Its ambiguity is the ratio of
the two errors **before refinement**. Only the best branch is then refined;
refining both can collapse distinct branches and obscure ambiguity. The solver
internally rotates its corner basis to avoid a numerical rotation-near-pi
singularity, then restores the decoded tag frame. The returned corner order
never changes.

`pose_valid` means the target PnP fit passed its checks. It does not mean the
orientation is unambiguous. `pose_ambiguity`, `pose_ambiguous`, and the alternate
raw pose expose that distinction. Field poses are withheld if ambiguity exceeds
`max_ambiguity` (default `0.2`). An exact symmetric frontal view can have near-zero
error in both branches; it is rejected as ambiguous rather than assigned false
certainty. The ambiguity ratio is not a position covariance. [PhotonVision describes the planar ambiguity problem and 0.2 rejection threshold](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html).

With `multitag: true` (default) and two or more known IDs in one image, the system
fits all their field corners jointly. It uses a planar solver when the geometry
is coplanar and SQPnP otherwise, then refines the accepted fit. RANSAC and
single-tag hypotheses are evaluated only when the direct fit disagrees. Inliers
are **whole tags**, so accepting a few corners of a bad detection does not turn
it into a MultiTag observation. Bad tags are reported in `rejected_tag_ids`.
Duplicate observed IDs are excluded from field localization. Unknown IDs still
provide independent target-relative observations.

Two mutually inconsistent known tags produce an invalid field result; there is
no arbitrary choice of which map entry to trust. With one usable known tag, or
`multitag: false`, the single-tag path applies. Disabling MultiTag selects the
lowest-ambiguity valid known single tag. Even multiple coplanar tags may remain
ambiguous at long distance or unfavorable angles; those results are withheld.
Incorrect but geometrically consistent maps cannot be detected automatically.

`always_single_tag: false` is the default performance setting. After an accepted
joint pose, target-relative transforms for the participating known IDs are
derived from that result and labeled `pose_source: field_layout_multitag`.
Set it true to pay for separate observed single-tag solutions when comparing
map accuracy. This is similar to the optional individual-target computation in
[PhotonVision's MultiTag pipeline](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html).

## Field-layout upload

Use a WPILib `AprilTagFieldLayout` JSON containing `field.length`, `field.width`
and a list of `tags`. All distances are meters. Each tag has an integer `ID`,
`pose.translation` with `x/y/z`, and `pose.rotation.quaternion` with `W/X/Y/Z`.
Dimensions must be positive, IDs unique, all numbers finite, and quaternions
unit length. Small serialization rounding is normalized; arbitrary quaternion
scaling is rejected. [WPILib specifies the JSON and default blue-wall origin](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/apriltag/AprilTagFieldLayout.html).

This is a schema illustration, **not a competition or practice field map**:

```json
{
  "field": {"length": 16.5, "width": 8.2},
  "tags": [{
    "ID": 1,
    "pose": {
      "translation": {"x": 2.0, "y": 1.0, "z": 1.3},
      "rotation": {"quaternion": {"W": 1.0, "X": 0.0, "Y": 0.0, "Z": 0.0}}
    }
  }]
}
```

Upload the map matching the actual practice/event field construction and tag
placement. Field outputs preserve the uploaded origin; they do not flip with
alliance color. Use the same origin on the robot. Keep real maps, calibration
measurements and camera mounts in local configuration, out of Git.

## Output, preview, and robot use

The `localization` result includes `valid`, `method`, `used_tag_ids`,
`inlier_tag_count`, `reprojection_error_px`, `ambiguity`, `field_to_camera`, and
`field_to_robot`. Invalid results have a reason and null field poses. A valid
field-camera result can still have null `field_to_robot` with
`robot_pose_invalid_reason: no_robot_to_camera`. An empty frame clears the
previous result. Per-camera observations remain independent; robot-side pose
estimation should combine accepted measurements with odometry and their capture
timestamps. See [the NetworkTables contract](networktables.md).

This project's NT4 payload is its own versioned protocol, not the PhotonLib
binary interface or Limelight's array schema. The distinction between target,
camera, robot, and field transforms is also reflected in
[Limelight's named pose outputs](https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api).

The web preview draws decoded outlines plus a 3D wireframe extending from the
tag's visible face. Axis colors are red +X, green +Y, blue +Z in the WPILib tag
frame. This box is an aid to inspecting pose, not a measured physical volume.
Rendering occurs in the preview path at preview FPS. Rotate the rendered preview
after drawing; raw detector pixels, calibration and robot geometry stay in the
capture orientation. A resolution mismatch suppresses the 3D overlay.

Synthetic tests cover independent absolute field scenes, nonidentity camera
mounts, actual decoded marker pixels through four orientations, calibrated lens
distortion, single/MultiTag recovery, ambiguous coplanar scenes, inconsistent
maps, whole-tag outliers, duplicate IDs, mode changes and projection overlays.
These tests establish software geometry, not real camera accuracy or latency.
Before autonomous use, measure pose at surveyed positions and headings, then
test motion, illumination, latency, stale-result rejection and odometry fusion
on the robot.
