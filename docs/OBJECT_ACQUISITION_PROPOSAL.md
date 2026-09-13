# Proposed floor-object acquisition — implementation deferred

The user requested brainstorming only for this phase. Existing object code remains
unchanged. No new object training, ranging, tracking, planner or robot Java code
has been implemented. Actual game pieces are undecided; small Wiffle balls are a
hypothesis, and the future color camera specification is not known.

## Recommended geometry

A fixed, downward-looking camera can estimate a floor object's position without a
depth sensor when the floor plane and camera mounting pose are known. Calibrate
the camera intrinsics, distortion, and its measured position/orientation on the
robot. Undistort a detected target pixel and convert it to a ray. Rotate that ray
into the robot's WPILib coordinate system; intersect it with the assumed target
height plane. This returns robot-relative forward/left coordinates and a bearing.

For camera origin `c` and unit ray direction `d`, both expressed in robot NWU:

```
ray = c + lambda * d
lambda = (target_height - c.z) / d.z
point_robot = c + lambda * d
range_xy = sqrt(point_robot.x^2 + point_robot.y^2)
bearing_left = atan2(point_robot.y, point_robot.x)
```

The ray begins as `K^-1 [u,v,1]` after undistortion, in OpenCV right/down/forward
axes. Convert to NWU once, then apply measured robot-to-camera rotation. Reject
rays near parallel to the plane, behind the camera, outside the calibrated image
region, or beyond a validated range. Use robot pitch/roll from a time-aligned IMU
when the chassis tilts; a fixed robot-Z floor plane otherwise becomes incorrect.

This is the general ray/plane form of the fixed-height target ranging used in
[PhotonVision's target-data guide](https://docs.photonvision.org/en/latest/docs/programming/photonlib/using-target-data.html).
It is a proposed design, not a claim of measured accuracy on this camera.

## Where to aim the ray

For arbitrary floor pieces, train a contact-point keypoint or segmentation model
and project its estimated contact with the floor. Bounding-box bottom center is a
simple starting approximation, but perspective, occlusion and detector padding
can shift it away from the real contact point. Measure that bias rather than
calling it exact distance.

For a ball of known radius, a detected projected center and a plane at `z=radius`
can estimate the ball center. Segmentation/circle fitting may improve the center
estimate. A visible silhouette center is not perfectly the projected 3D sphere
center at close distance; verify against labeled ground truth. Known apparent
size can provide a second consistency check, but should not be the primary range
method when the piece is partially hidden or the size is not confirmed.

A floor homography is a useful equivalent approach on a flat, fixed plane. It
maps image coordinates directly into floor XY; it still requires calibration
and becomes wrong when camera mounting, chassis tilt or assumed plane changes.
Stereo/depth hardware becomes worthwhile if objects are elevated, piled up, or
plane-based distance error is unacceptable. Avoid monocular learned-depth models
as the first solution: they add compute and calibration uncertainty to a geometry
problem with a known plane.

## Detection and tracking

Use a small detector trained on the actual production color-camera images. Pick
input size/model size using held-out recall at intake distances and measured
TensorRT latency. Segmentation or a learned contact keypoint may be worth a few
milliseconds if it materially improves range consistency. Run false-positive
checks on field tape, reflections, holes, robot wheels and empty-field footage.

Track candidates over time rather than choosing a new box independently each
frame. Associate detections in robot/field XY, carry an uncertainty estimate,
expire missing detections, and avoid switching targets mid-approach. Convert a
robot-relative observation to the field using the robot pose at **capture time**,
not the newest robot pose at receipt. Continue publishing the raw robot-relative
point so the robot can also do short-range visual servoing without global drift.

Estimate uncertainty by perturbing pixel location, camera height, pitch, and
calibration parameters. Ground-plane range becomes very sensitive near the
horizon, so a maximum trustworthy range is more useful than a single global
accuracy number. Provide x/y uncertainty, observation age, quality, and method
along with each target. Do not invent a probability from a heuristic score.

## Robot behavior after approval

The planned Java helper should read coherent timestamped NT results, transform
camera observations to robot/intake coordinates, and expose helpers such as
`getFreshTargets()`, `getNearestReachableTarget()` and `getApproachPose()`.
Names are proposals only; no library is required yet.

Select a reachable object, create an approach pose that accounts for the intake's
position and bumper clearance, and update a short path as observations improve.
Use field keep-out zones and obstacle checks; a target pixel is not evidence that
the route is clear. As the object reaches the intake, reduce speed and steer
using lateral target error. Confirm acquisition using an intake sensor or another
reliable mechanism state. Stop/replan on stale or low-quality observations.

## Hardware split and acceptance experiment

Start with two independently timestamped AprilTag cameras on this Jetson. A
second Jetson can handle one or two color intake cameras after the detection
requirements are measured. This keeps neural inference from competing with
localization; it is not evidence that two computers are always required. Do not
fuse asynchronous cameras by averaging their poses; fuse timestamped observations
on the robot with suitable uncertainty and shared-tag correlation in mind.

Before implementation, confirm the game piece, camera model/lens, mounting height
and pitch, expected acquisition range, acceptable XY error, and intake offset.
Record objects at measured floor coordinates, repeat under different lighting
and chassis tilt, and report median/p95 XY error, false targets, missed targets,
complete measurement age, and acquisition success. Compare full autonomous
acquisition runs—not just neural inference FPS.
