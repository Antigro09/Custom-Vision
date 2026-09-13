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

Current development scope (user's follow-up on 2026-09-13):
- Prioritize measured low latency, targeting roughly 12–24 ms camera-to-robot;
  prefer C++ and useful CUDA acceleration. Do not claim superiority over
  PhotonVision/Limelight without a fair measured comparison.
- Plan two cameras per Jetson. Likely separate AprilTag and object Jetsons;
  topology may change after measuring combined load.
- AprilTags require 2D/3D, calibrated robot-relative targets, uploaded field
  layouts, joint MultiTag poses, 3D box overlays, and browser camera setup with
  a headless option. Follow familiar PhotonVision/ WPILib concepts, with an
  explicitly documented custom NT4 contract.
- Future robot code is Java; user will request it after testing. Do not implement
  robot Java or autonomous motion during the current AprilTag phase.
- Object work is now **brainstorm only**. Preserve existing code but do not add
  detection, training, ranging or tracking until the user approves the proposal.
  Proposed fixed downward-facing color camera projects detections onto a known
  floor/ball-center plane, with mounting calibration, uncertainty and tracking.
- Dependencies belong in project .venv/private builds. Python 3.10 is sufficient
  for the implemented NT4 runtime; installing 3.11 is authorized if later needed.

Current implementation: C++17 AprilTag 3.4.5 detector and single-tag PnP,
optional CUDA detector/preprocessing, joint calibrated MultiTag localization,
latest-frame independent camera workers, demand-driven browser preview/config,
NT4 schema 2 with synchronized host-read times and stale invalidation. Default
browser port 5801 avoids existing PhotonVision on 5800. No real intrinsic calibration,
measured mount, confirmed field layout or trained game-piece model is bundled.

The initial setup had 84 passing tests; the expanded validation and actual measured
compute results are recorded in docs/SETUP_REPORT.md and docs/PERFORMANCE.md.
Physical cameras remain absent. User service installed but disabled; unattended
boot and on-robot latency/accuracy remain unverified. Do not claim live readiness.
