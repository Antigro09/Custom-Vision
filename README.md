# Team 1086 Custom Vision

AprilTag localization and object acquisition vision for the Jetson Orin Nano Super, with a
browser setup interface and NetworkTables 4 output. Native C++/CUDA handles tag
detection and optional single-tag and joint field PnP. Python manages cameras,
configuration, coordinate transforms and publication. Each camera has an independent worker that takes the newest
frame. Preview rendering runs separately and only when requested.

## Implemented

- **2D:** tag IDs, corners, calibrated bearings (nominal FOV fallback explicitly marked).
- **3D:** calibrated single-tag PnP, ambiguity and reprojection checks, camera- and
  robot-relative tag transforms, and adjustable 3D box/axis preview overlays.
  Optional custom CUDA IPPE/refinement is selectable independently of detection;
  the same pose-device setting controls joint MultiTag and pose fallbacks.
- **POI aiming:** configured tag-relative 3D offsets with current-frame angular
  bearings, quality checks and NT4 topics; independent of field maps and odometry.
- **MultiTag:** a CPU or custom CUDA joint solve using uploaded WPILib field
  coordinates, whole-tag outlier rejection and field camera/robot poses. Robot poses require measured
  camera mounting extrinsics. Individual PnP is skipped when a joint solve suffices,
  except when POI aiming needs an independent tag observation.
- **Browser setup:** camera mode tuples, driver-reported UVC controls, calibration
  and field JSON uploads, measured mount, 2D/3D, decoder settings, and independent
  preview resolution, FPS, quality and 90° rotation. Processed FPS, latency and
  stage times are displayed separately. Saving validates and restarts
  the runtime; old targets are invalidated during reconfiguration.
- **NT4:** coherent versioned JSON per frame, synchronized server timestamps when
  available, pose convenience topics, boot identifiers and freshness watchdogs.

YOLO26 detection and instance segmentation now run through TensorRT, with
calibrated object ranging, track IDs, current-frame selection and intake-offset
approach coordinates. The default profile includes a disabled future color-camera
pipeline; `config/objects.yaml` is the dedicated object-Jetson template. Supply the
trained model, real camera calibration and measured geometry before enabling it.
See [object setup](docs/objects.md) and [the ROS/box/segmentation decision](docs/ROS_AND_OBJECT_DESIGN.md).

## Start on this Jetson

```bash
cd /home/jetsonorin/Documents/Custom-Vision
./scripts/setup_jetson.sh
# First setup only, if local.yaml does not exist:
cp -n config/vision.yaml config/local.yaml
.venv/bin/python -m custom_vision.app --config config/local.yaml --check
.venv/bin/python -m custom_vision.app --config config/local.yaml
```

Open `http://<jetson-address>:5801` from a laptop on the robot LAN. Port 5801 avoids
this Jetson's existing PhotonVision on 5800. `--headless` disables the web server;
`--no-nt` disables robot publication. `--check` validates configuration/backends
without opening cameras or contacting the robot.

The default profile enables `front_tags`; `rear_tags` is ready but disabled until
a second camera is attached. Set stable `/dev/v4l/by-id/...` sources, upload real
calibration and field layout, and enter the measured robot-to-camera transform.
The default NT root is `/CustomVision/jetson-tags`. Use different roots for different
Jetsons. This is a documented custom protocol, not a PhotonLib drop-in replacement.

## Hardware and speed

The supplied OV9281-class Swyft/Arducam is monochrome, global shutter, USB 2 UVC,
**MJPEG only**, 1280×800 up to 120 FPS, 81° × 52° FOV, with a fixed-focus lens.
120 FPS is a camera specification, not verified pose throughput. Intrinsics cannot
be inferred accurately from its factory focus description or nominal FOV.

The C++ build privately pins AprilTag 3.4.5, uses CPU-specific release optimization,
releases Python's GIL, avoids nested OpenCV thread pools, and provides optional
CUDA preprocessing, CUDA detection and custom CUDA single-tag and joint MultiTag PnP. See [native build details](native/README.md)
and [the performance report](docs/PERFORMANCE.md) for the measured paths and limits.
CUDA must be selected explicitly and passes capability validation; it never silently
falls back to CPU PnP. CUDA pose supports 4/5/8-coefficient pinhole distortion
and up to 256 observed mapped tags per joint solve. Coordinate transforms, POI
projection, NetworkTables, UI and default camera JPEG decoding still use the CPU.

The initial target is **12–24 ms camera-to-robot latency**, subject to measurement.
No physical cameras were available during implementation. Synthetic compute timings
exclude exposure, USB transport, capture decode, networking and robot scheduling.
No claim of outperforming PhotonVision or Limelight is made without the same live
scene, cameras, calibration, quality thresholds and timing method.

## Verify and explore

```bash
.venv/bin/python -m pytest -q
.venv/bin/python scripts/smoke_test.py
.venv/bin/python scripts/smoke_objects.py
.venv/bin/python scripts/benchmark_apriltags.py --backend native --cameras 2
# Actual GPU cases explicitly skip without a CUDA build/device:
.venv/bin/python -m pytest -q -ra tests/test_cuda_pose.py tests/test_cuda_multitag.py
.venv/bin/python scripts/demo_apriltags.py --port 5802
# Run separately on the same port for the object geometry demo:
.venv/bin/python scripts/demo_objects.py --port 5802
```

The AprilTag demo serves an explicitly synthetic three-tag field scene at
`http://127.0.0.1:5802`, with NetworkTables disabled. Its files are under ignored
`data/apriltag-demo/`; it does not replace production calibration. The smoke test
checks 2D, single-tag and multi-tag localization through synthetic MJPEG video,
including clearing all targets at shutdown. Desktop CI builds the CPU native module;
Jetson tests additionally exercise available CUDA and TensorRT hardware.
The object demo uses three rendered ball candidates with known coordinates and
always disables NT. It tests geometry and controls, not neural-model accuracy.

## Setup guides

- [Camera modes and exposure](docs/cameras.md)
- [Intrinsic calibration](docs/apriltags.md)
- [Field coordinates, mounting and MultiTag](docs/localization.md)
- [Tag-relative POI aiming, FPS and CUDA PnP](docs/POI_AND_CUDA_POSE.md)
- [September 20 Jetson verification and paired CUDA measurements](docs/CUDA_VERIFICATION_2026-09-20.md)
- [Browser controls](docs/dashboard.md)
- [NetworkTables contract and future robot integration](docs/networktables.md)
- [Object geometry and tracking](docs/object_geometry.md)
- [YOLO26 export and training tools](docs/yolo_exports.md)
- [Performance measurements and live acceptance procedure](docs/PERFORMANCE.md)
- [Latest pose optimization and paired measurements](docs/POSE_OPTIMIZATION_2026-09-20.md)
- [Setup status](docs/SETUP_REPORT.md)

A user service is installed but disabled. After configuring connected cameras:

```bash
systemctl --user enable --now custom-vision
journalctl --user -u custom-vision -f
```

The service normally requires a login. Boot without login needs administrator-enabled
lingering; that has not been configured. Keep one owner per camera. The existing
PhotonVision service has been left running.

Dependencies are isolated in `.venv` with access to JetPack system libraries. Setup
preserves working CUDA/TensorRT/PyTorch. Python 3.10 and the installed compatible
PyNTCore provide NT4; a Python upgrade is unnecessary for this implementation.
Real calibration, datasets, model binaries, builds and credentials are ignored by Git.
[PROJECT_MEMORY.md](PROJECT_MEMORY.md) preserves team decisions for future work.
