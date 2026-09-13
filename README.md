# Team 1086 Custom Vision

AprilTag aiming and robot localization for the Jetson Orin Nano Super, with a
browser setup interface and NetworkTables 4 output. The detector and single-tag
pose solver run in C++; Python manages cameras, configuration, joint localization
and publication. Each camera has an independent worker that takes the newest
frame. Preview rendering runs separately and only when requested.

## Implemented

- **2D:** tag IDs, corners, calibrated bearings (nominal FOV fallback explicitly marked).
- **3D:** calibrated single-tag PnP, ambiguity and reprojection checks, camera- and
  robot-relative tag transforms, and 3D box/axis preview overlays.
- **MultiTag:** a joint solve using uploaded WPILib field coordinates, whole-tag
  outlier rejection and field camera/robot poses. Robot poses require measured
  camera mounting extrinsics. Individual PnP is skipped when a joint solve suffices.
- **Browser setup:** camera mode tuples, driver-reported UVC controls, calibration
  and field JSON uploads, measured mount, 2D/3D, decoder settings, and independent
  preview resolution, FPS, quality and 90° rotation. Saving validates and restarts
  the runtime; old targets are invalidated during reconfiguration.
- **NT4:** coherent versioned JSON per frame, synchronized server timestamps when
  available, pose convenience topics, boot identifiers and freshness watchdogs.

The existing object backends are retained but disabled in this AprilTag profile.
New object development is paused at the user's request; see the
[floor-pickup proposal](docs/OBJECT_ACQUISITION_PROPOSAL.md).

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
CUDA preprocessing and CUDA detector acceleration. See [native build details](native/README.md)
and [the performance report](docs/PERFORMANCE.md) for the measured paths and limits.
CUDA must be selected explicitly and passes capability validation; it never silently
falls back to CPU. Camera JPEG decoding remains on the CPU in the default capture path.

The initial target is **12–24 ms camera-to-robot latency**, subject to measurement.
No physical cameras were available during implementation. Synthetic compute timings
exclude exposure, USB transport, capture decode, networking and robot scheduling.
No claim of outperforming PhotonVision or Limelight is made without the same live
scene, cameras, calibration, quality thresholds and timing method.

## Verify and explore

```bash
.venv/bin/python -m pytest -q
.venv/bin/python scripts/smoke_test.py
.venv/bin/python scripts/benchmark_apriltags.py --backend native --cameras 2
.venv/bin/python scripts/demo_apriltags.py --port 5802
```

The demo serves an explicitly synthetic three-tag field scene at
`http://127.0.0.1:5802`, with NetworkTables disabled. Its files are under ignored
`data/apriltag-demo/`; it does not replace production calibration. The smoke test
checks 2D, single-tag and multi-tag localization through synthetic MJPEG video,
including clearing all targets at shutdown. Desktop CI builds the CPU native module;
Jetson tests additionally exercise available CUDA and TensorRT hardware.

## Setup guides

- [Camera modes and exposure](docs/cameras.md)
- [Intrinsic calibration](docs/apriltags.md)
- [Field coordinates, mounting and MultiTag](docs/localization.md)
- [Browser controls](docs/dashboard.md)
- [NetworkTables contract and future robot integration](docs/networktables.md)
- [Performance measurements and live acceptance procedure](docs/PERFORMANCE.md)
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
