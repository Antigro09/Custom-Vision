# Persistent project context

Updated: 2026-09-20.

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
  robot Java or autonomous motion until requested.
- User approved full object-pipeline implementation in the latest follow-up:
  YOLO26 likely fine-tuned later, bounding boxes versus instance segmentation,
  calibrated ranging/tracking and NT4 output. Intake camera will face downward.
  Implement both box and segmentation paths; compare actual Jetson latency.
  No team dataset or trained game-piece weights exist yet. Java remains deferred.
- ROS decision: current CUDA detector uses NVIDIA cuAprilTags from Isaac ROS
  directly, without a ROS runtime. Keep it unless measured benefits justify a
  ROS/NITROS graph; ROS itself does not automatically accelerate kernels.
- Dependencies belong in project .venv/private builds. Python 3.10 is sufficient
  for the implemented NT4 runtime; installing 3.11 is authorized if later needed.

Current implementation: C++17 AprilTag 3.4.5 detector, CPU or custom CUDA single-tag
and joint MultiTag PnP, optional CUDA detector/preprocessing, calibrated localization,
latest-frame independent camera workers, demand-driven browser preview/config,
NT4 schema 2 with synchronized host-read times and stale invalidation. Default
browser port 5801 avoids existing PhotonVision on 5800. No real intrinsic calibration,
measured mount, confirmed field layout or trained game-piece model is bundled.

Object implementation now supports YOLO26 NMS-free detection and instance
segmentation plus compatible raw outputs, persistent TensorRT10 CUDA buffers,
calibrated plane ranging with uncertainty rejection, brief tracking/current-frame
selection, intake approach displacement, browser controls and compact NT4 data.
Targets use WPILib robot coordinates at capture time; no motion compensation or
obstacle-checked autonomous path is claimed. Canonical metric targets on the wire
are objects.targets, selected by selected_track_id. Signed target-plane Z must
share the measured robot origin; zero means floor only if that origin is on floor.
Use config/objects.yaml for the future dedicated object Jetson. Its camera mode
and offsets are placeholders and its required trained engine is not supplied.
Ultralytics 8.4.150 training/export tools use separate ignored .venv-export;
COCO benchmark models under models/benchmark are for speed/contract testing only.
The project contains training/export scripts, but no team fine-tuning occurred.

The September 13 object/AprilTag suite passed 326 tests on this Jetson. Actual two-worker
YOLO26n at 640x640 compute: median 16.674 ms / p95 19.491 ms, about 59 FPS per camera;
YOLO26n-seg at 640x640 with 5 masks: 23.577 / 32.723 ms, about 42 FPS per camera. These exclude
camera transport/decode, metric geometry and robot/NT costs. Generic COCO models
matched upstream boxes on one test image; this is not team accuracy. Recommend
boxes first. Detailed methods/results are in docs/SETUP_REPORT.md and docs/PERFORMANCE.md.
Physical cameras remain absent. User service installed but disabled; unattended
boot and on-robot latency/accuracy remain unverified. Do not claim live readiness.

September 20 scope and implementation:
- User supplied `Custom-Vision-POI-CUDA.patch` and the corresponding ZIP in Downloads,
  asked for review, real Jetson CUDA verification/optimization and publication to
  main. The patch was applied to the existing fresh implementation, not restored
  from the old pre-rewrite system. Guided mrcal calibration and object features
  remain in scope for regression testing.
- User explicitly requires CUDA 3D solving for MultiTag and all applicable pose
  paths, not only isolated single tags. `settings.pose_device: cuda` now selects
  both native solvers and deferred single-tag fallback. No CPU PnP seed or hidden
  CPU fitting fallback is permitted in this mode. CPU remains an explicit option.
- CUDA single-tag uses inverse distortion, dual IPPE initialization and nonlinear
  refinement. Joint CUDA uses known field corners, planar/nonplanar initialization,
  complete-tag consensus, robust starts and joint refinement. It handles 2–256
  observed mapped tags; invalid or ambiguous observations can remain rejected.
  Nonplanar finite multistart is not a proof of global optimality/uniqueness.
- Both CUDA pose paths support 4/5/8-coefficient OpenCV pinhole distortion. More
  complex supported CPU lens models require explicit CPU selection. Coordinate
  conversion, bearing/POI geometry, capture, UI and NT remain CPU work; do not say
  every application operation moved to CUDA or that CUDA is always faster.
- POI is current-frame tag-relative aiming, computed before field enrichment,
  independent of odometry and field maps. Tag offsets: +X out of printed front,
  +Y right when viewing upright print, +Z up. Optical tx right+, ty up+. Robot
  coordinates require the measured mount. Verification of the selected calibration
  is required for valid POI output; benchmark calibration stays preview-only.
  Calibration changes through the controller clear that acknowledgement.
- Dashboard now shows processed FPS alongside latency and actual solve devices.
  Preview box depth is configurable: 0.5 is the original shallow box, 1.0 a cube;
  a shallow box alone does not diagnose bad camera intrinsics.
- Typed NT topics include FPS and POI validity/bearings/coordinates; the coherent
  JSON packet includes actual solver devices, joint CUDA timings and residuals.
  Robot Java integration and autonomous motion remain deferred.
- No physical cameras are attached during this verification. Synthetic known-pose
  tests execute on the real Orin GPU. Compute Sanitizer could not instrument it:
  GPU debugging access is disabled for the current user. Do not claim a clean
  memcheck/racecheck/synccheck run; sudo remains unavailable without user input.
  See docs/POI_AND_CUDA_POSE.md for rerun instructions and verification boundaries.
- Final local regression on the optimized September 20 build: 549 pytest tests
  passed, 3 explicit skips (mrcal/mrgingham absent from the main environment and
  the unavailable-CUDA negative case); 15 dashboard client tests passed. Actual
  custom GPU/math coverage includes 83 tests, and 360 additional seeded joint
  stress cases passed. Do not add the focused count to the full-suite count.
- The final paired matrix verified 7,200 rendered-image frames, 14,400 single-tag
  batch calls and 9,600 joint calls. Two-worker full-CUDA compute: four-tag scene
  14.926 ms median / 17.727 ms p95, 67–72 FPS per worker; mapped POI/MultiTag scene
  18.058 / 25.159 ms, 50–56 FPS. CPU detection + CUDA pose was faster on the mapped
  scene (12.406 / 17.952 ms, 78–83 FPS). CUDA is not universally faster; CPU single
  poses had lower p95 at all tested batch sizes. Defaults remain explicit CPU.
  These exclude physical capture/USB/decode/preview/NT/robot receipt. See
  docs/CUDA_VERIFICATION_2026-09-20.md and its machine-readable summary for complete
  paired data and hashes. Actual browser save/restart, cube-depth, 2D/3D, CUDA
  joint-only, POI preview and shutdown/stale clearing were verified locally.

September 20 follow-up optimization:
- User requested continued speed/efficiency work. Added per-detector joint CUDA
  graph replay with an eight-entry LRU cache keyed by tag count, threshold and
  iteration count. Every call uploads fresh corners/field geometry. External
  event nodes preserve completion/timing; new or evicted shapes pay setup cost.
- Joint normal equations now skip only zero padding beyond actual tile corners.
  Host localization reuses joint transforms and removes redundant rotation
  conversions/small NumPy temporaries. Hypotheses, FP64, iterations, quality gates,
  coordinate conventions and NT schema are unchanged.
- Fresh before/after comparison against 3f7f728 on the same device: 7,680 joint
  calls and 4,800 mapped-image frames verified. Two-worker joint medians fell
  15–60% across tested planar/nonplanar 2/4/8/16-tag cases. Full CUDA image compute
  improved 19.994/26.302 to 17.161/23.938 ms median/p95, roughly 49–50 to 56–57 FPS
  per worker. CPU detection + CUDA pose improved 11.583/17.281 to 10.207/16.441 ms.
  CUDA detection + CPU pose median worsened 3.4%; some single-worker joint tails
  worsened. Do not claim universal gains, physical-camera latency, or power savings.
- 551 distinct pytest tests passed across isolated runs (543 with three skips,
  plus all eight actual TensorRT tests), 15 dashboard tests and both runtime
  smokes passed. An initial combined run had three TensorRT fixture setup errors
  from GPU out-of-memory with full swap; isolated runs resolve the test pressure,
  not proof of unrestricted simultaneous workloads. Eight create/solve/close
  cycles showed stable memory; four cached graph shapes cost about 2.5 MiB extra
  process RSS versus baseline. Sanitizer/physical-camera limitations still apply.
- See docs/POSE_OPTIMIZATION_2026-09-20.md and its paired machine summary. Local
  raw artifacts live in ignored data/pose-optimization-2026-09-20/. Verified native
  binary SHA-256: d075a764392c99d4fe592b4f3eaa78ea95c7196b4f0948660920fb0804897c9a.
