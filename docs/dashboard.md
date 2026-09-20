# Browser setup and preview

Open `http://<jetson-address>:5801` from a laptop on the robot network. The default
bind address is `0.0.0.0`, so the browser can run on the driver laptop; no monitor
is needed on the Jetson. Existing PhotonVision commonly uses port 5800, so this
project defaults to 5801. NetworkTables is a separate connection to the robot.
The UI is implemented locally, with no CDN or internet requirement.

Select the camera pipeline, adjust settings, then **Save & apply settings**. The
runtime validates the proposed configuration and reports whether a restart is
required. Capture changes can briefly interrupt frames. The UI reports failures;
it never invents a connected camera or a pose. Save pending edits before uploading
calibration or a field layout, because uploads apply to the saved configuration.

## Camera settings

The camera device supplies complete **format + width + height + FPS** tuples via
`v4l2-ctl --list-formats-ext`. The UI groups them into a single selector. The saved
mode remains visible as *configured; unverified* if no matching camera is attached.
Some drivers describe continuous or stepwise mode ranges; this UI deliberately
offers only explicit discrete tuples instead of guessing combinations.

Only actual advertised UVC controls are offered. Writable integer, boolean and
menu controls are validated against the device's range, step, and menu before
application. Automatic exposure / white balance can make their manual controls
inactive: turn automatic operation off, save and refresh, then set the newly
available manual controls. Driver read-only and inactive controls are disabled.
The OV9281 has a glued fixed-focus lens; there is no focus slider. Existing
`convert_rgb` behavior belongs to the OpenCV backend rather than a fabricated
UVC hardware control. If `v4l2-ctl` is absent, install `v4l-utils` to discover and
adjust device controls; the dashboard otherwise remains available.

Prefer a stable `/dev/v4l/by-id/...` source in YAML when two cameras are installed.
Check actual captured dimensions in diagnostics; requesting a mode is not proof
that the driver accepted it. Calibrations must match the raw image resolution.

## AprilTag setup

- **2D:** image corners and aiming. No tag pose is claimed.
- **3D:** calibrated tag pose. Measured intrinsic calibration is required.
- **Multi-tag:** known field tag corners jointly constrain field localization.
  Upload a WPILib `AprilTagFieldLayout` JSON and measure the camera mounting pose.
- The robot-to-camera mounting transform uses WPILib NWU coordinates: X forward,
  Y left, Z up, translations in meters and roll/pitch/yaw in degrees. Zero values
  shown before measurement are placeholders, not verified physical geometry.
  Check **Camera mount has been measured** only after entering measured values;
  leaving it unchecked saves a null mount and prevents robot-pose publication.
- Upload intrinsic calibration as this project's camera calibration JSON; use the
  calibration CLI and `docs/apriltags.md` for the expected format and procedure.
  Nominal lens FOV and a fixed focal distance do not supply measured intrinsics.

With **native**, the **Detection device** selector chooses CPU AprilTag 3 or the
CUDA detector. CPU is the default; GPU selection requires an available CUDA
detector build. This is separate from the preprocessing setting. Pipeline health
uses the running detector status (`C++ / CPU` or `CUDA / GPU`), rather than assuming
a saved change has already taken effect. Review measured latency and distant-tag
recall for both devices and when adjusting decimation or thread count. Synthetic
input is explicitly labeled **Synthetic preview**, and the frame footer reports
the running detection mode rather than an unapplied setting.

The separate **Pose device** selector controls single-tag PnP, joint field
MultiTag and deferred single-tag fallback: CPU or the custom CUDA solvers. CUDA
requires a native CUDA-pose build, 3D mode, matching intrinsics and 4/5/8-coefficient
OpenCV pinhole distortion. It supports up to 256 observed mapped tags per joint
solve. Unsupported CUDA configurations fail instead of silently using CPU PnP.
The timing strip labels the single-tag and field solvers that actually ran;
“single-tag pose not run” can accompany a valid CUDA field solve when individual
poses were skipped. Choose by full processing latency and accuracy, not the GPU
label alone. Coordinate transforms, POI projection and the UI remain CPU work.

**Point of interest aiming** accepts named tag IDs with metric offsets from each
tag's center, in configured priority order. Tag axes are X out of the printed
front, Y right when viewing the upright tag, and Z up. Its tx/ty bearings remain
independent of field localization. The calibration verification checkbox stays
off until physically checked; replacing calibration clears the old acknowledgement.
Orange **PREVIEW ONLY** points never publish a valid aim flag. The
[POI guide](POI_AND_CUDA_POSE.md) provides examples and the NetworkTables contract.

## Object setup

Selecting an object pipeline shows object settings and its target plane instead
of AprilTag settings and field layout. Capture, intrinsic calibration upload,
measured camera mount, and preview controls remain shared.

Choose **TensorRT** for a trained GPU model or **OpenCV ONNX** for a trained CPU
model. Enter the path to an existing model file on the Jetson; the browser does
not upload model binaries. Set detection or segmentation, the exported output
format (`yolov8_raw` or `yolo26_end2end`), labels in training class order, input
width and height, confidence threshold, and maximum detections. Labels accept
commas or newlines. These settings must match the exported model. No trained
game-piece model is included. **Contour** and **HSV** are shape/color candidate
baselines; their scores are circularity/color fill rather than trained model
probabilities. Their existing detailed YAML settings are preserved when saving.

For robot-relative ranging, upload matching intrinsics, measure the camera mount,
and enter the selected anchor's **measured Z in the robot frame**. A blank
height explicitly keeps results 2D only. Zero specifies the floor only when the
robot origin lies on the floor; an elevated origin requires a negative floor Z.
Choose box center/bottom or segmentation centroid/bottom to match that plane.
Mask anchors require segmentation output, and masks/anchors are approximate.
Use range and position-uncertainty limits to reject unreliable geometry. Intake
offsets use robot +X forward and +Y left; the approach standoff is in meters.
These are geometric outputs, not robot motion commands.

Pipeline health shows the selected track, robot X/Y and horizontal range only
for a valid current target. Coordinates describe the robot frame at capture
time, without motion compensation. Invalid geometry or a disconnect clears
the displayed selection and range. Preview draws object boxes, optional mask
contours, track IDs, valid ranges and a green selected target. No field position
is implied. As with AprilTags, all overlays are drawn before preview rotation.

## Stream and performance

Preview dimensions, rate, JPEG quality, and rotation are independent of capture.
Overrides are saved in each pipeline's `preview` mapping; missing values inherit
the dashboard defaults. Rotation is 0, 90 clockwise, 180, or 270 degrees (90
counterclockwise). Overlays are drawn in raw camera coordinates before display
rotation. Detection, calibration, corners, and NetworkTables results always use
the unrotated capture frame.

For AprilTags, **3D box depth / tag size** changes the preview prism: 0.5 is the
original short box and 1.0 is a cube. It changes only drawing geometry.

The inference thread stores only the latest frame reference and result. A separate
worker draws overlays, resizes, rotates and compresses JPEGs at the configured
preview cap. It discards obsolete pending frames and stops compressing when no
browser has requested that pipeline for two seconds. Browser polling has at most
one frame request outstanding; hidden tabs back off. A disconnect clears both
preview and results, including any encode already in flight. Preview is helpful
but consumes CPU and network; benchmark both with and without viewers.

The displayed latency begins at host frame receipt, **not sensor exposure**.
Exposure, USB transfer and robot fusion delay need a physical measurement.
**Processed FPS** measures the smoothed interval between completed frames, not
sensor FPS, preview FPS, or the inverse of processing latency. It clears to zero
on disconnect. The stage strip separates detection, single-tag pose, field/POI
work and queue delay; object pipelines show their own inference/geometry stages.
`GET /api/preview` reports preview encode time, dimensions and frame IDs separately
from the inference/publication timing returned by `GET /api/status`.

For fully headless operation set `dashboard.enabled: false` or pass
`--no-dashboard`. The normal browser configuration is also headless on the Jetson:
it does not launch a local desktop or display window.

## Local HTTP interface

- `GET /api/status`: latest results keyed by pipeline name (read-only).
- `GET /api/config`: `{config, csrf_token, writable}`.
- `GET /api/devices`: real device inventory with modes and controls.
- `GET /frame/<pipeline>`: latest JPEG; HTTP 404 if disconnected or unavailable.
- `POST /api/config`: full editable configuration object.
- `POST /api/calibration`: `{pipeline, data}` with parsed calibration JSON.
- `POST /api/field-layout`: `{data}` with parsed WPILib field-layout JSON.

Writes require JSON, a body no larger than 1 MiB, and the
`X-Custom-Vision-CSRF` header with the token supplied by `/api/config`. Browser
writes must originate from the same host. This prevents unrelated websites from
silently reconfiguring a robot camera; it is not user authentication. A runtime
restart rotates the setup token. If a later write is explicitly rejected because
of that token, the browser fetches a fresh token and retries once without losing
the form edits. It does not replay writes after an uncertain network failure. Keep the
service on a trusted robot / development LAN rather than exposing it publicly.
Upload filenames are chosen by the runtime controller, not the request. The
server exposes only three fixed static assets and validated pipeline frame names.

A dashboard instantiated without a controller remains read-only for compatibility
with diagnostics and tests. The `Dashboard.set_renderer(callback)` hook receives
`(payload, frame)` on its preview worker; a renderer must copy before drawing,
because camera frame references are shared with runtime processing.
