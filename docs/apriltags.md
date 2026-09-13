# AprilTags and camera calibration

The default `custom_vision.native_apriltags.NativeAprilTagPipeline` uses C++ detection and single-tag PnP; `custom_vision.apriltags.AprilTagPipeline` is the portable fallback. Both detect `tag36h11` and returns JSON-compatible dictionaries. The default tag side is 0.1651 meters (6.5 inches); verify the tag dimensions for the actual field and any practice print. Measure the side at the detected black/white border, not the outside of the surrounding white margin. [AprilRobotics describes tag sizing and pose conventions](https://github.com/AprilRobotics/apriltag#pose-estimation).

Results contain `id`, `hamming`, `decision_margin`, `center`, `corners`, and `pose_valid`. Corner order preserves the tag's decoded orientation. A valid pose adds `tvec_m`, `rvec_rad`, `distance_m`, and `reprojection_error_px`. `rvec_rad` is an OpenCV Rodrigues rotation vector: its direction is the rotation axis and its magnitude is the angle. The pose transforms tag-local coordinates to the optical camera frame: x right, y down, z forward. Do not send this translation directly to a WPILib field pose; camera mounting transforms and the season's field layout are additional required inputs.

`distance_m` is the Euclidean distance to the tag center. `tvec_m[2]` is forward depth. These values describe the camera, not the robot center. A single planar tag can have ambiguous orientation, especially head-on or far away; low reprojection error is a fit check, not an uncertainty guarantee. The implementation evaluates both IPPE square solutions, refines them, keeps positive-depth solutions, and returns the lowest distorted-pixel reprojection RMS. The runtime adds joint multi-tag field localization and standardized WPILib transforms; see [localization](localization.md). [OpenCV documents the solver and camera axes](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html).

## Calibration

Use the same camera, lens focus, and raw capture resolution during calibration and operation. Capture at least 10 sharp images of a flat chessboard covering different areas, distances, and tilts. `--board-cols` and `--board-rows` count **inner intersections**, not squares; a board with 10 by 7 squares has 9 by 6 inner corners. Measure the square side in meters. The calibration module uses OpenCV's pinhole distortion model; fisheye calibrations are not interchangeable. [OpenCV's calibration tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) explains the capture pattern and lens model.

```bash
.venv/bin/python -m custom_vision.calibration \
  --images 'data/calibration/*.png' \
  --board-cols 9 --board-rows 6 --square-size-m 0.025 \
  --output config/calibration.json
```

The command records width, height, camera matrix, distortion, overall RMS, and individual view errors. It rejects mixed resolutions, unreadable inputs, insufficient distinct views, and overall RMS above 1 pixel by default. Review individual view errors and validate distances against a measured target before robot use. Images without a detected board and near-duplicate views are recorded as skipped. Low calibration RMS alone cannot prove that capture coverage or the measured board dimensions are correct.

Missing calibration permits ID/pixel detection with `pose_valid: false` and `pose_invalid_reason: no_calibration`. A capture resolution different from calibration produces `calibration_resolution_mismatch`, with no metric pose. The pipeline does not silently rescale intrinsics because changed camera modes can crop or otherwise change the image. The legacy example calibration lacks a capture resolution and contains placeholder values; it cannot be used as a measured calibration.

## Detection quality controls

- `min_decision_margin`: default 30; weak decodes below this are omitted.
- `max_hamming`: default 0; corrected-bit detections are omitted unless explicitly allowed (0–2).
- `max_reprojection_error_px`: default 3; a poor fit returns `pose_valid: false`, `pose_invalid_reason: reprojection_error`, and the error without translation or rotation.
- `threads`: default 2; CPU detector worker count.
- `quad_decimate`: default 2; a larger value trades detection range and corner accuracy for speed.

Tune using representative motion, lighting, exposure, distances, and CPU load. The automated tests render genuine tag36h11 pixels, rotate tags through all four orientations, check distorted pose recovery, and calibrate rendered chessboards with known intrinsics.
