# Persistent project context

Updated: 2026-09-13.

- User: Anthony / GitHub Antigro09; FRC team **1086**.
- Workspace: `/home/jetsonorin/Documents/Custom-Vision`.
- Repository: https://github.com/Antigro09/Custom-Vision . Existing history retained,
  with a fresh implementation requested by the user; old baseline was commit
  `681280be594f3ccd642e36874a21008722be0c4b`.
- Hardware: NVIDIA Jetson Orin Nano Super, 8 GB RAM, aarch64 Ubuntu 22.04.5,
  L4T R36.4.7, CUDA 12.6, TensorRT 10.3, MAXN_SUPER mode observed.
- Cameras: Swyft/Arducam, OV9281-class global-shutter **monochrome** sensors.
  USB 2.0 UVC, MJPEG only, 1280x800 at up to 120 FPS, 81 degree horizontal and
  52 degree vertical FOV; 35g without cable. Lens described as non-distortion,
  factory calibrated and glued at 5ft focus. Intrinsics still need measured or
  vendor-provided calibration; nominal FOV/focus does not supply a camera matrix.
  Supported UVC controls: brightness, contrast, saturation, hue, gain, auto exposure,
  exposure, gamma, sharpness, backlight compensation, auto white balance, white
  balance temperature, convert_rgb. No focus/zoom/pan/tilt/iris controls.
  No `/dev/video*` devices were present at setup.
- Color cameras for object detection will be obtained soon; model and specs unknown.
- Two required pipelines: AprilTag detection for FRC and object detection for
  game-piece acquisition/placement.
- Game pieces are **not known yet**. Small Wiffle balls are a user hypothesis,
  not a confirmed game specification. Do not assume a season or train against
  an invented class definition.
- User explicitly authorized running code/files, installing dependencies, device
  and repository setup, and saving this context to memory. This authorization
  applies to this project and persists across sessions.
- User requested starting from scratch rather than extending the old code.
- Existing `photonvision.service` was already running; leave it alone unless
  coordinating ownership of a connected camera. No custom service should claim
  the same camera simultaneously.
- GitHub connector is authenticated with repository admin access. Local Git
  HTTPS push currently has no credentials; use the connector for publication.
- Passwordless sudo is unavailable. User-level dependency/service setup works;
  do not claim privileged installation or boot startup that has not happened.

Current baseline: native AprilTag 3 via pupil-apriltags; NT4 robot publication;
monochrome contour candidates for bench testing and a TensorRT backend for a
future trained model. Calibration and physical validation remain required.

Setup verification on 2026-09-13: 84 tests passed including native AprilTag,
local NT4, and real GPU execution of a synthetic TensorRT network. End-to-end
synthetic video passed both pipelines. See docs/SETUP_REPORT.md. User service is
installed but disabled, cameras absent; never claim live readiness until measured.
