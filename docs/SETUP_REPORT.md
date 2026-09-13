# Jetson setup and implementation verification — 2026-09-13

FRC team 1086, `/home/jetsonorin/Documents/Custom-Vision`, repository
[Antigro09/Custom-Vision](https://github.com/Antigro09/Custom-Vision). Earlier source
remains in Git history; this implementation follows the requested fresh start.

## Installed and preserved

- Jetson Orin Nano Super 8 GB, aarch64 Ubuntu 22.04.5, Python 3.10.12, L4T 36.4.7.
- Existing CUDA 12.6, TensorRT 10.3, PyTorch 2.3.0 and MAXN_SUPER preserved.
- Project `.venv`, editable `frc-custom-vision 0.2.0`, NumPy 1.26.4,
  Python OpenCV 4.10.0, PyYAML 6.0.2, NTCore 2024.3.2.1 and pytest 8.4.2.
- C++17 release extension with private pinned AprilTag 3.4.5, pybind11 2.13.6,
  existing native OpenCV 4.13.0-dev and actual CUDA preprocessing/detection support.
- Optional NVIDIA cuAprilTags header/static archive from Isaac ROS 3.2, pinned,
  hash-verified, licensed notices retained. No ROS install, JetPack replacement,
  system AprilTag replacement, Python upgrade or global GPU wheel install.
- Existing PhotonVision remains running on 5800. Custom Vision browser uses 5801.
- User service installed at
  `/home/jetsonorin/.config/systemd/user/custom-vision.service`, disabled/stopped.
- Ignored `config/local.yaml` holds this machine's profile. Only `front_tags`
  enabled; `rear_tags` awaits a second physical camera. No invented calibration,
  camera mount or season field layout is installed in the production profile.
- No `/dev/video*` cameras were available. Passwordless sudo and user lingering
  are unavailable; boot without login and locked clocks were not configured.

## Completed capabilities and validation

The AprilTag phase had **218 passing tests on the Jetson**. That coverage remains
in the current expanded suite. Tests cover:

- CPU/native and actual GPU detection of rendered tag36h11 patterns, all four
  rotations, GPU decimations 1–4, raw pixel corner recovery, distorted calibration,
  single-tag pose, ambiguity, margin filtering, resolution changes and concurrency.
- Joint MultiTag geometry, fixed field origin, robot mounting transforms, whole-tag
  outliers, duplicate IDs, missing/mismatched calibration, and 2D pose invalidation.
- Browser configuration/upload validation, camera control discovery and bounds,
  demand-driven preview, rotation and stale-image clearing. Client regressions
  cover token renewal after restart and runtime-versus-saved status display.
- Actual browser checks: consecutive saves across worker restarts, 2D/3D mode,
  synthetic labeling, robot field pose display and rotated 3D overlays.
- Real local NT4 client/server publication, synchronized clock conversion,
  coherent frame packets, invalid pose/topic clearing and wire numeric precision.
- Runtime stale-frame watchdog, delayed-publication race, hung capture reader
  ownership, UVC source aliases, restart refusal and safe shutdown.
- Existing object/TensorRT regression coverage, including execution of a tiny
  synthetic neural network on the GPU. This is not a trained game-piece model.

The updated video smoke test exercises 10 MJPEG frames each through 2D, single-tag
PnP and MultiTag. It verifies IDs 7/12/20, expected robot position and clearing at
shutdown. See ignored `data/smoke/summary.json`; all inputs/calibration are synthetic.
Configuration initialization, shell syntax, JavaScript syntax and Git whitespace
checks passed. Benchmark methodology/results are in [PERFORMANCE.md](PERFORMANCE.md),
with seven recorded two-camera runs, each 600 measured frames.

## Remaining physical validation

Connect both cameras; confirm actual MJPG mode, exposure, USB throughput and
ownership. Measure intrinsics at the raw operating resolution and camera mounts;
upload the confirmed field layout. Validate distance, field pose, ambiguity,
small-tag recall, clock conversion and total camera-to-robot latency using real
motion/lighting. Keep CPU/GPU choices and decimation tied to these measurements.

The 12–24 ms total-latency target is not certified by synthetic compute results.
Robot Java and autonomous movement remain deferred. Actual trained game-piece
weights and physical camera validation still require the team's data.

## Object implementation follow-up

The user approved full object implementation after reviewing the proposal.
The runtime now handles YOLO26 detection and instance segmentation, calibrated
target-plane ranging with uncertainty rejection, brief track association,
current-frame selection, intake-relative approach displacement, compact NT4
object packets and typed selection topics. Object model/geometry controls and
box/mask/selected-target overlays are available in the browser.

The existing direct cuAprilTags integration remains. It uses NVIDIA's library
distributed with Isaac ROS without adding ROS nodes or a ROS-to-NT bridge.
See [the architecture decision](ROS_AND_OBJECT_DESIGN.md).

Ultralytics 8.4.150 is installed only in `.venv-export` with automatic dependency
installation disabled. Existing system PyTorch/CUDA/TensorRT remain unchanged.
Training and static ONNX export scripts are provided; actual team training has
not been run without a dataset. Official COCO YOLO26n detection and segmentation
exports were generated at 640x640 for real TensorRT compatibility/performance
checks. They retain their original labels and are not production game-piece models.

Browser verification used the explicit synthetic object demo with NT disabled.
Three known floor targets produced valid robot coordinates. Clearing target
height preserved 2D boxes and removed metric selection; restoring the height
restored ranging. Consecutive saves restarted workers successfully. Preview
rotation and conditional TensorRT model fields were checked in the browser.

The dedicated `config/objects.yaml` profile supports two cameras; it leaves
calibration, measured mount and target height unset. Its enabled first camera
requires a real engine before it passes initialization. The normal local profile
keeps its existing AprilTag camera enabled and the new object entry disabled.

The complete updated suite passed **326 tests in 55.64 seconds on this Jetson**,
including actual FP32/FP16 TensorRT execution, input-dependent GPU outputs,
independent camera contexts/streams, segmentation prototypes and buffer cleanup.
It also covers geometry projection/uncertainty, signed plane heights, undefined
bearings, tracking/freshness, exact FP16 normalization and one-ULP FP32 parity,
export validation and benchmark worker-failure handling. All 8 browser-client
JavaScript tests passed. A real local NT4 server/client round trip verified
object selection, position, approach arrays, compact coherent JSON and stale
clearing; this does not substitute for testing the eventual Java consumer.

Both full YOLO26 models were compared against upstream PyTorch predictions on a
COCO test image. Each matched all five detections, with minimum box IoU 0.997343;
bounded mask outlines are explicitly approximate. Four final GPU benchmark runs
cover one and two camera workers. See [PERFORMANCE.md](PERFORMANCE.md) for the
recorded numbers and all excluded camera/robot costs.

Final smoke checks passed: 10 frames each for 2D, single-tag PnP and MultiTag,
plus 20 synthetic object frames (16 observed-target frames and 4 empty frames).
Object positions matched the known rendered coordinates within 0.015 m, track IDs
stayed stable across fresh consecutive frames, and target loss/shutdown cleared
results. This tolerance belongs to an ideal synthetic fixture, not real-world
accuracy. The production AprilTag profile passed initialization; shell/JavaScript
syntax and Git whitespace checks passed.
