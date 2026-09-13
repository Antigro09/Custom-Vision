# Team 1086 Custom Vision

Fresh AprilTag and object-detection software for the NVIDIA Jetson Orin Nano Super.
The earlier implementation is retained in Git history. This project starts from
scratch with a Python runtime, native AprilTag 3 detection, NT4 output, and a
TensorRT backend for trained game-piece models.

## What is ready

- AprilTag 36h11 IDs and pixels; camera-relative metric poses with valid calibration.
- Monochrome ball-candidate detection for bench experiments, optional color/HSV
  detection, and a GPU TensorRT backend for a future trained model.
- Shared or separate UVC cameras, latest-frame capture, reconnect attempts, and
  empty/invalid results when a camera fails or a processed frame is over 500ms old.
- Read-only preview/status web page, NT4 robot output, calibration/data capture
  tools, repeatable setup, tests, and a user service installer.

The game pieces are undecided. Small Wiffle balls are a possibility, not a confirmed
specification. The supplied baseline finds geometric candidates; it is **not a
trained ball classifier**. No trained FRC weights or real camera calibration are
bundled. Physical cameras were not connected during initial setup.

## Hardware configuration

Team: **1086**. AprilTag camera: Swyft/Arducam OV9281-class, global-shutter monochrome,
1280×800, USB 2.0 UVC **MJPEG only**, up to 120 FPS; 81° horizontal × 52° vertical FOV.
The lens is fixed and described as factory calibrated/glued at a 5ft focus distance.
A measured or vendor-provided intrinsic calibration is still required for distance.
Color cameras for object detection will be added later.

The default configuration requests MJPG at 1280×800/120 FPS. This is a requested
capture mode, **not a measured pipeline throughput claim**. Two pipelines share
camera 0 for initial bench work. Change the intake source when its camera arrives.
Each distinct camera gets its own processing worker; pipelines sharing one camera
run sequentially on its newest frame. JPEG decoding and AprilTags currently run on
CPU; only the TensorRT neural backend uses the GPU.

## Start

```bash
cd /home/jetsonorin/Documents/Custom-Vision
./scripts/setup_jetson.sh
cp config/vision.yaml config/local.yaml  # first time only
.venv/bin/python -m custom_vision.app --config config/local.yaml --check
.venv/bin/python -m custom_vision.app --config config/local.yaml
```

Open [the local preview](http://127.0.0.1:5800). `/api/status` returns the latest
results and `/frame/<pipeline-name>` returns its annotated JPEG. Preview refresh
is 2Hz, independent of detection speed. Set `dashboard.host: 0.0.0.0` in the local
configuration to make the preview accessible from the robot LAN.

NetworkTables defaults to team 1086 at `/CustomVision`. Use `--no-nt` for isolated
bench work. Use `--stdout-json` for per-result output and `--max-frames 100` for a
bounded run. `--check` loads configuration/backends without opening cameras or
contacting the robot; passing it does not certify physical readiness.

## Verify without a camera

```bash
.venv/bin/python scripts/doctor.py
.venv/bin/python -m pytest -q
.venv/bin/python scripts/smoke_test.py
```

The smoke test generates tag 7 and a bright circular candidate, processes a video
through both real pipelines, and checks shutdown clears results. Its images and
calibration are synthetic and stored under ignored `data/smoke/`. GPU integration
tests build a tiny synthetic TensorRT network when the Jetson libraries are present;
these validate the backend plumbing, not game-piece recognition accuracy.

## Camera, calibration, and robot setup

1. Connect the camera and run `v4l2-ctl --list-devices` and
   `v4l2-ctl -d /dev/video0 --list-formats-ext`. Prefer stable
   `/dev/v4l/by-id/...-video-index0` paths in `camera.source`.
2. Ensure only one service owns each camera. A pre-existing PhotonVision service
   was running during setup. Assign different cameras or stop it deliberately
   before using its device with this runtime.
3. Capture checkerboard views across the image at the operating resolution and
   compute calibration using [the calibration guide](docs/apriltags.md). Set the
   corresponding pipeline's `calibration` path in `config/local.yaml`.
4. Verify tag black-border size, tag IDs, pose axes, reprojection error, distance,
   and latency against measured targets. Camera pose is not field/robot pose.
5. Connect the Jetson to the robot LAN and implement the consumer described in
   [the NT4 contract](docs/networktables.md). Verify stale results are rejected on
   robot, including when the Jetson loses power or network connection.
6. Follow [object detection](docs/objects.md) when collecting game-piece data and
   exporting a model. Build each TensorRT engine on this Jetson.

## Service

```bash
./scripts/install_user_service.sh
systemctl --user enable --now custom-vision  # after configuring connected cameras
journalctl --user -u custom-vision -f
systemctl --user stop custom-vision
```

The installer writes a user service and does not enable it automatically. A user
service normally requires login. To run at boot without login, an administrator
can enable lingering using `sudo loginctl enable-linger jetsonorin`. Passwordless
sudo was unavailable during setup, so unattended boot operation has not been
configured or verified.

## Dependencies and repository

Jetson setup preserves installed NVIDIA CUDA/TensorRT/PyTorch. The `.venv` uses
system site packages for JetPack's native libraries. NumPy stays below 2 for the
existing OpenCV stack. PyNTCore 2024.3.2.1 was already installed and provides NT4;
current 2026 PyPI wheels require Python 3.11+, while this Jetson uses Python 3.10.
The package allows the Python-3.10-compatible NT4 releases (2023.4–2024).

`custom_vision/` is the runtime, `config/` holds examples, `scripts/` contains setup
and calibration/data helpers, and `tests/` checks geometry, network behavior, and
inference. Local calibration, datasets, engines, and credentials are ignored by
Git. [PROJECT_MEMORY.md](PROJECT_MEMORY.md) records the user's choices and project
context; [AGENTS.md](AGENTS.md) tells future coding sessions to read it.

Upstream references: [AprilTag 3](https://github.com/AprilRobotics/apriltag),
[WPILib coprocessor vision](https://docs.wpilib.org/en/stable/docs/software/vision-processing/wpilibpi/using-a-coprocessor-for-vision-processing.html),
[NVIDIA TensorRT Python API](https://docs.nvidia.com/deeplearning/tensorrt/10.13.3/inference-library/python-api-docs.html).
