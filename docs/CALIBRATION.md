# Guided Arducam calibration with mrcal

`calibration.py` is the new repository-root entry point. The existing
`custom_vision/calibration.py` image-only OpenCV API is unchanged. The new tool
captures video and selected images, fits **actual mrcal models**, compares a rich
reference model with the runtime model, and exports the runtime JSON contract.
No camera calibration, mount, or robot configuration is bundled or guessed.

## Install on the Jetson

Use a local graphical desktop/monitor for the live window. A terminal over SSH
without a working display needs `--headless --seconds 180` instead. Ensure the
camera is not already owned by PhotonVision or another process; this tool does
not stop services or change their startup settings.

```bash
# From the repository root, on the calibration branch:
bash scripts/install_calibration.sh
.venv-calibration/bin/python calibration.py doctor
```

The installer uses the distribution's APT packages for mrcal, mrgingham and
scientific Python. It creates `.venv-calibration` with system site packages,
separate from the robot runtime's `.venv`. It does **not** use pip or replace
CUDA/TensorRT/PyTorch with generic wheels. It checks the simulated APT plan for
NVIDIA GPU package changes and refuses them, and uses `--no-remove`.
It adds no third-party package repositories. Ubuntu 22.04 and 24.04 are the CI
platforms; installation on the actual Jetson still requires its normal sudo
permissions and working APT repositories. Solver execution defaults to
`/usr/bin/python3`; override with `--mrcal-python` only for another compatible
mrcal environment. Distro versions may be older than upstream releases.

## Board and camera preparation

Measure the printed square pitch, in meters, in both directions. The numeric
example below assumes **10 by 10 INNER corners**, meaning 11 by 11 squares, with
25 mm squares. Replace those numbers with your physical board's measured values.
Do not confuse squares with intersections. A larger rigid board may be needed
for sharp close-ups with a fixed-focus lens; a small board extremely close to the
camera can be out of focus. Do not use a chessboard displayed on a monitor as a
substitute for a verified flat target.

Mount the pattern on a rigid flat support. Keep the camera's final focus/lens
settings fixed. Use bright, even, flicker-free lighting and avoid glare and motion
blur. Hold the board briefly at each pose. Include tilted views about both axes,
different distances, and especially image edges/corners, with all inner corners
visible. Avoid large in-plane diagonal rotations when using mrgingham. Do not
interpret the coverage grid as a calibration accuracy estimate.

Calibration is specific to the physical camera, lens settings and imaging mode.
Use the final image resolution/crop. If changing FPS changes sensor crop/readout,
recalibrate in that mode. Default capture is 1280x800 MJPG at 30 FPS to keep the
recording manageable, not a 120 FPS performance test. `--fps 120` requests 120;
it does not guarantee that the USB path, codec or disk can sustain it.

## Live preview, video recording and selected frames

```bash
.venv-calibration/bin/python calibration.py capture \
  --camera /dev/video0 --width 1280 --height 800 --fps 30 \
  --board-cols 10 --board-rows 10 --square-size-m 0.025 \
  --session data/calibration/arducam-front-01
```

Use a NEW session directory each time. Existing directories are never overwritten.
The left pane is the current live view with occupied corner cells. The right pane
is the last analyzed frame with its own detected corners. This separation is
intentional: expensive corner detection must not draw old corners over a newer
frame. Capture/recording and board detection run in separate threads with bounded
queues. The source images are never resized, annotated, or undistorted before
calibration. Only the preview is resized.

Keys: **Q** finishes capture and starts the solve; **Esc** saves without solving;
**A** toggles automatic sampling; **Space** requests a sample. Manual sampling
still rejects poor or duplicate views. The live tool rejects blurry/low-contrast,
very small, clipped-corner and near-duplicate views. These are configurable
heuristics, not proofs that every retained frame is accurate.

Default selection is at most one candidate every 0.7 seconds, at most 180 saved
views, with at least 40 required for a solve. Aim for roughly 100 diverse usable
views, then use the diagnostics to decide whether more are needed. Recording
continues after the selected-view limit; use `--max-views` to change the limit.
`--seconds 180` bounds a live capture. `--capture-only` skips the automatic solve.

The session contains original-resolution PNG samples, `session.json`, optional
`raw.avi` using FFV1, and `raw-timestamps.jsonl`. FFV1 adds no video compression
loss to decoded frames; it does not undo the camera's original MJPEG losses.
Recordings can be large. `--no-record` disables the full video while keeping PNG
samples. A slow disk can reduce delivered FPS. Hardware frame/exposure counts
are not available: host-read timestamps are not exposure timestamps. The video
container's nominal FPS may differ from the acquisition rate; the timestamp
sidecar preserves actual host-read intervals.

## An existing video

```bash
.venv-calibration/bin/python calibration.py video \
  --video /path/to/board-video.avi \
  --board-cols 10 --board-rows 10 --square-size-m 0.025 \
  --session data/calibration/from-video-01
```

The window shows each analyzed video sample and the detected-corner overlay while
processing. This is sampled analytical playback, not a real-time media player.
Use `--headless` to process without a window. The same frame-quality and novelty
filters apply. The solver begins after EOF or Q. Re-encoded, cropped, stabilized,
resized or severely compressed footage cannot recover the original camera's
calibration automatically; use untouched native-resolution footage.

When replaying this tool's `raw.avi`, it automatically reads the neighboring
`raw-timestamps.jsonl`. Use `--timestamps PATH` if you renamed or moved that file.
Other videos use frame index divided by reported FPS; variable-frame-rate videos
need a matching sidecar for accurate temporal holdout grouping. Images themselves
are calibrated in their native pixel coordinates regardless of video timing.

Rerun a saved session without opening the camera:

```bash
.venv-calibration/bin/python calibration.py solve \
  --session data/calibration/arducam-front-01
```

## What the mrcal solve does

Final corners are detected again from lossless samples. `--corner-detector auto`
uses actual mrgingham for square grids when installed, and high-accuracy OpenCV
SB for rectangular grids. Both feed **mrcal**, not `cv2.calibrateCamera`.
`--corner-detector mrgingham` explicitly requires a square grid; use `opencv-sb`
for rectangular boards. Preview selection uses OpenCV SB for either shape, so it
can miss frames that mrgingham would have detected. The retained raw video allows
future extraction/selection improvements without recapturing everything.

The solver withholds whole three-second blocks (about 20% of views) before fitting
training models. It checks the held-out corners with intrinsics and board warp
fixed; only the six board-pose variables are fitted for each unseen view. This
reduces adjacent-frame leakage, but is not an independent second capture and
cannot detect every systematic board or lens error. It then refits on all views
for the deliverable models. mrcal's default board-warp fitting, regularization,
and corner-outlier rejection remain enabled.

By default it fits `LENSMODEL_OPENCV8` and a moderate cubic
`LENSMODEL_SPLINED_STEREOGRAPHIC` reference. It computes projection uncertainty
at infinity and a rotation-aligned projection-difference map. The moderate spline
grid is a practical starting point, not a proven optimal model for every lens.
`--fov-deg 81` is an INITIAL estimate, not vendor calibration. A wrong spline FOV
can compromise image-edge support; inspect the maps. `--no-spline` skips the rich
reference explicitly, without claiming model bias was checked.

Quality flags currently include less than 70% corner-cell coverage, held-out RMS
above 0.6 px, more than 5% rejected corners, uncertainty p95 above 0.5 px, undefined
diagnostics, or spline/runtime disagreement above 0.5 px p95. These thresholds are
heuristics, not specifications or calibration certificates. They are not tuned to
real Arducam data. A low RMS alone does not certify correctness. mrcal uncertainty
primarily addresses sampling noise, not all board deformation, lens-model bias,
wrong square-size scale or physical survey errors.

Every solve creates a NEW `solve-*` subdirectory with:

| Output | Meaning |
|---|---|
| `opencv8/full/*.cameramodel` | mrcal runtime-compatible model, with optimization inputs |
| `spline/full/*.cameramodel` | Richer reference; NOT directly loadable by this robot runtime |
| `*/training/*.cameramodel` | Training-only models used to evaluate held-out frames |
| `intrinsics.json` | Exact OPENCV8-to-runtime JSON, only when automated heuristics pass |
| `intrinsics.candidate.json` | Retained candidate when quality checks need review |
| `board-poses.json` | Per-frame board-to-camera CV transforms; NOT robot mount extrinsics |
| `*-uncertainty.png` | Sampling uncertainty grids; larger values need investigation |
| `spline-vs-opencv8.png` | Model disagreement, not measured ground-truth error |
| `report.json` | Split provenance, source hashes, errors, warnings and status |
| `*/solver.log`, `*/corners.vnl` | Real mrcal commands, observations and solver output |

Exit codes: 0 = heuristics passed (still needs physical validation), 2 = retained
candidate needs review, 1 = failed command/solve, 130 = interrupted. Capture-only
success also returns 0 without implying a calibration exists. `latest-solve.json`
points to the latest completed solve. Failed attempts retain a report and logs.

The runtime export contains fx/fy/cx/cy, the 3x3 camera matrix, eight OpenCV
rational distortion coefficients and the calibration resolution. The exporter
checks mrcal/OpenCV projections numerically. A spline is NEVER silently converted
to eight coefficients. If the richer reference differs materially, improve the
data/model or explicitly design a rectification-aware runtime before deployment.

## Robot-mount extrinsics are a separate measurement

A freehand video determines camera intrinsics and the board's pose in each frame.
It cannot reveal where the camera is on the robot: the robot coordinate system
never appears in those images. This tool therefore does not manufacture a zero
mount, copy a board pose into `robot_to_camera`, or claim stereo calibration.

For the mount workflow, hold the camera in its final robot mount and take a NEW
image of a fixed chessboard whose pose you have surveyed relative to the robot.
Supply `survey.json` containing a proper 4x4 `robot_T_board` transform:

```text
p_robot_NWU = robot_T_board[:3,:3] @ p_board + robot_T_board[:3,3]
```

Robot axes: X forward, Y left, Z up. Board origin: detected inner corner 0.
Board +X: along the first row toward the last column. Board +Y: along the first
column toward the last row. Board +Z is their right-handed cross product. Matrix
columns are those measured board axes expressed in robot coordinates, and the
last column is the measured board origin in meters. Do not put example/identity
numbers here unless they actually describe your physical setup.

```bash
.venv-calibration/bin/python calibration.py mount \
  --calibration data/calibration/arducam-front-01/solve-ACTUAL-ID/intrinsics.json \
  --session data/calibration/arducam-front-01 \
  --image data/calibration/surveyed-board.png \
  --survey data/calibration/survey.json \
  --output data/calibration/mount-front.json
```

First invocation writes `mount-front.corners.png` and exits with code 2, without
writing a mount. Inspect the labeled origin/+X/+Y against the actual surveyed
board. Ordinary chessboards have ambiguous corner numbering: `--reverse-corners`
handles 180-degree reversal; square boards can also have 90-degree ambiguity.
Survey the ACTUAL displayed ordering; do not guess it. Repeat the command with
`--confirm-board-order` only after checking. The pose solver rejects poor and
near-equally-plausible planar solutions, converts optical CV axes to the runtime's
WPILib NWU convention, and writes `robot_to_camera` plus explicit full transforms.

This mount solve uses OpenCV planar PnP/refinement with the mrcal-calibrated
intrinsics. Reprojection error does not include survey error. Repeat with other
independently surveyed board positions and compare the resulting mounts; verify
translation, signs, angles and metric scale physically before robot use. A single
surveyed view is not maximum-accuracy metrology or a mount uncertainty estimate.

## Put the calibration into the vision runtime

Do not deploy `intrinsics.candidate.json` just to remove an error. Inspect its
report, collect missing views and repeat first. After physical validation, use
the existing dashboard calibration upload, or set the pipeline's `calibration`
to the exact JSON path in `config/local.yaml` (paths are relative to that YAML).
Set `robot_to_camera` separately from the verified mount output. Keep these local
files under ignored `data/` or `config/local.yaml`; never commit real calibration,
recordings or surveyed positions by accident.

Calibration improves geometric correctness; it does not itself make neural
inference faster. This command runs offline and adds no per-frame runtime work.
The runtime uses the same OpenCV model family and schema it already supports.

## Validation and limitations

`tests/test_guided_calibration.py` includes actual PNG/FFV1 round trips, corner
detection on rendered boards, nonmutating overlays, selection/holdout tests,
threaded acquisition error cleanup, timestamp alignment, axis/transform tests,
actual mrgingham execution, and actual mrcal fits/uncertainty/export when installed.
The calibration workflow runs the installed distro mrcal on Ubuntu 22.04/24.04
and exercises the GUI under Xvfb. Synthetic tests validate software contracts,
not real Arducam accuracy, Jetson camera behavior, USB throughput or robot safety.

For more defensible accuracy, independently repeat the capture with a verified
rigid board, compare projection maps, and check measured physical targets at the
working distances. Keep camera mode/focus/mount fixed throughout.

## Primary references

- mrcal capture, model and diagnostic workflow: https://mrcal.secretsauce.net/how-to-calibrate.html
- Sampling uncertainty and limitations: https://mrcal.secretsauce.net/uncertainty.html
- Independent calibration comparisons: https://mrcal.secretsauce.net/cross-validation.html
- Installation: https://mrcal.secretsauce.net/install.html
- Lens models: https://mrcal.secretsauce.net/lensmodels.html
- Solver flags/corner-cache contract: https://mrcal.secretsauce.net/mrcal-calibrate-cameras.html
- Python projection/uncertainty APIs: https://mrcal.secretsauce.net/mrcal-python-api-reference.html

mrcal provides useful high-accuracy tools, but its documentation does not establish
it as the universally best calibrator. Capture quality, model adequacy, independent
validation and the target accuracy requirement matter more than a brand ranking.
