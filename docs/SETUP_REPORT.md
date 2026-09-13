# Jetson setup verification — 2026-09-13

Fresh implementation for FRC team 1086 in
`/home/jetsonorin/Documents/Custom-Vision`; previous source is preserved in Git
history at `681280be594f3ccd642e36874a21008722be0c4b`.

## Installed and observed

- aarch64 Ubuntu 22.04.5, Python 3.10.12, L4T R36.4.7.
- CUDA 12.6 and TensorRT 10.3 already installed; preserved unchanged.
- MAXN_SUPER power mode already active; no clock/power/firmware changes made.
- Existing PyTorch 2.3.0 reports CUDA available; runtime does not require it.
- Project `.venv` with system site packages and editable `frc-custom-vision 0.1.0`.
- Built pupil-apriltags 1.0.4.post11 natively for aarch64.
- NumPy 1.26.4, OpenCV 4.10.0, PyYAML 6.0.2, NTCore 2024.3.2.1, pytest 8.4.2.
- Python OpenCV is CPU-only and does not provide GStreamer in this installation.
  UVC/V4L2 MJPEG works at the software interface level; no live camera was available.
- User service installed at
  `/home/jetsonorin/.config/systemd/user/custom-vision.service`, disabled/stopped.
  `config/local.yaml` created for machine-specific edits (ignored by Git).
- Existing PhotonVision service left running. It must not compete for a camera.
- No `/dev/video*` devices detected. No real calibration/dataset/model supplied.
- User lingering is disabled and passwordless sudo unavailable. Unattended
  boot operation is not configured or verified.

## Validation completed

- **84 tests passed** on the Jetson, including actual CPU and GPU execution.
- AprilTag tests detect actual rendered 36h11 patterns, rotate them, verify
  corner ordering, perspective/distortion pose geometry, and invalid-pose cases.
- Camera calibration tests recover known intrinsics from generated checkerboards.
- Object tests cover monochrome round candidates, color ranges, bearings,
  letterbox mapping, class-aware NMS, and rejected output contracts.
- Two TensorRT integration tests build a tiny synthetic, input-dependent engine
  on the GPU and verify CUDA transfers, inference output, cleanup and the complete
  grayscale-to-neural-detection path. This is not a trained object model.
- A real local NT4 client/server test verifies publication and stale-result clearing.
- Runtime tests cover camera failures, slow frames, watchdog, shutdown, latest-frame
  replacement, invalid configurations and preview clearing.
- `scripts/smoke_test.py` processed 10 frames through each actual pipeline, found
  tag 7 and the bright circular candidate, and verified shutdown clears targets.
- Configuration initialization, Python compilation and shell syntax checks passed.
- A bounded real-camera attempt returned failure cleanly because no camera was
  attached. No physical FPS, pose accuracy, or robot-network claim is made.

## Next hardware-dependent work

Connect the OV9281-class camera, confirm its V4L2 MJPG mode, coordinate ownership
with PhotonVision, calibrate it at 1280x800 and validate measured target distances.
Then connect to the roboRIO and check the robot's own receipt-age timeout. Add the
future color camera as the separate intake source. Once the game is known,
collect/label actual images, train/validate the model and build its TensorRT engine.
