# apriltag_vision

A C++ AprilTag detection pipeline for FRC-style vision applications using:

- **apriltag** (official C library, linked as `libapriltag`)
- **OpenCV 4.x** (camera I/O, image processing, pose estimation, visualization)
- **NTCore/CameraServer stack** (`ntcore`, `cscore`, `cameraserver`, `wpiutil`) for dashboard feed + NetworkTables
- **nlohmann/json** (calibration parsing + optional debug stdout JSON)
- **CMake**

The runtime keeps the hot path on raw camera frames for lower latency and better pose correctness:

`capture -> grayscale -> detect -> solvePnP -> annotate -> NetworkTables -> CameraServer stream`

When you also provide an FRC AprilTag field layout JSON, the app can estimate a field-relative camera pose from the observed tags. With multiple known tags visible at once, it solves a combined multi-tag pose on the coprocessor instead of leaving that work for the roboRIO.

All exported transforms now follow the PhotonVision/WPILib coordinate conventions rather than raw OpenCV camera axes.

## Project Layout

- `src/config.hpp`
- `src/calibration.hpp`, `src/calibration.cpp`
- `src/detector.hpp`, `src/detector.cpp`
- `src/pose.hpp`, `src/pose.cpp`
- `src/visualize.hpp`, `src/visualize.cpp`
- `src/publisher.hpp`, `src/publisher.cpp`
- `src/main.cpp`
- `CMakeLists.txt`

## Dependencies

You need installed development packages for:

- OpenCV
- apriltag
- ntcore
- cscore
- cameraserver
- wpiutil
- CMake

### Ubuntu / Linux

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  libopencv-dev
```

Install apriltag and the NTCore/CameraServer packages using your platform's preferred packages or installer. If CMake cannot find them, set `CMAKE_PREFIX_PATH` or the specific `*_DIR` package path variables.

### Windows

- Install OpenCV and apriltag development packages that your compiler/CMake setup can find.
- Install the NTCore/CameraServer C++ packages (`ntcore`, `cscore`, `cameraserver`, `wpiutil`) or point CMake at an existing allwpilib/native install that provides them.
- Make sure `cmake` is on `PATH`.

### macOS (Homebrew)

```bash
brew update
brew install cmake pkg-config opencv apriltag
```

Install the NTCore/CameraServer packages separately, then point CMake at them if needed:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/native-packages
```

`nlohmann/json` is found via `find_package` if installed, otherwise fetched automatically by CMake FetchContent.

## Build

```bash
rm -rf build
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/usr/local \
  -DENABLE_NETWORKTABLES=ON
cmake --build build -j$(nproc)
```

If CMake cannot find one or more native packages, configure with one of:

```bash
cmake -B build -S . -DCMAKE_PREFIX_PATH=/path/to/native-packages
cmake -B build -S . -Dntcore_DIR=/path/to/ntcore
cmake -B build -S . -Dcscore_DIR=/path/to/cscore
cmake -B build -S . -Dcameraserver_DIR=/path/to/cameraserver
cmake -B build -S . -Dwpiutil_DIR=/path/to/wpiutil
```

## Runtime Outputs

- Local debug window via `cv::imshow` unless `--no-display` is set
- CameraServer MJPEG stream with the configured camera name
- NetworkTables topics under `/Vision/<camera_name>`
- Optional per-frame JSON on stdout when `--stdout-json` is enabled

## Usage

```bash
./build/apriltag_vision --calibration calibration.json --camera 0 --tag-size 0.165 --team 1086
```
#### For no GUI
```bash
./build/apriltag_vision --calibration calibration.json --camera 0 --tag-size 0.165 --team 1086 --no-display
```

### CLI flags

- `--camera <int>` camera index (default: `0`)
- `--camera-name <name>` CameraServer and NT camera name (default: `camera0`)
- `--tag-size <double>` tag size in meters (default: `0.1651`)
- `--calibration <path>` calibration JSON file (**required**)
- `--team <number>` connect NT client to a robot team number
- `--nt-server <host>` connect NT client to a specific host, overrides `--team`
- `--width <int>` requested camera input width
- `--height <int>` requested camera input height
- `--fps <int>` requested camera input FPS
- `--family <name>` AprilTag family. Supported: `tag16h5`, `tag25h9`, `tag36h10`, `tag36h11`, `tagCircle21h7`, `tagCircle49h12`, `tagCustom48h12`, `tagStandard41h12`, `tagStandard52h13`
- `--threads <int>` AprilTag detector thread count
- `--quad-decimate <float>` AprilTag detector quad decimation
- `--blur <float>` AprilTag detector Gaussian blur sigma
- `--refine-edges <on|off>` enable or disable edge refinement
- `--pose-iterations <int>` OpenCV pose-refinement iterations in the range `0-100`
- `--max-error-bits <int>` maximum corrected tag bits (`16h5` is typically `0`, `36h11` is typically `<= 3`)
- `--decision-margin <double>` minimum accepted decision margin
- `--decision-margin-cutoff <double>` alias for `--decision-margin`
- `--auto-exposure <on|off>` camera auto exposure control
- `--exposure <double>` manual exposure value, requires `--auto-exposure off`
- `--brightness <double>` camera brightness value
- `--auto-white-balance <on|off>` camera auto white-balance control
- `--white-balance <double>` manual white-balance temperature, requires `--auto-white-balance off`
- `--orientation <normal|cw90|180|ccw90>` rotate the displayed/streamed output without changing detection geometry
- `--stream-width <int>` CameraServer stream width. Must be paired with `--stream-height`
- `--stream-height <int>` CameraServer stream height. Must be paired with `--stream-width`
- `--field-layout <path>` PhotonVision/WPILib-style field layout JSON for field-relative pose and multi-tag solve
- `--no-display` disable the local `cv::imshow` window only
- `--stdout-json` print debug JSON to stdout in addition to NT publishing
- `--record <output.mp4>` save the annotated output video

If neither `--team` nor `--nt-server` is provided, the app starts a local NetworkTables server for desktop testing.

When you request `--width`, `--height`, and `--fps`, the camera backend negotiates the closest mode it supports. The app prints the actual mode it opened at startup so you can verify the camera accepted that combination.

### NetworkTables topics

Under `/Vision/<camera_name>` the app publishes:

- `connected`
- `frame_id`
- `timestamp_us`
- `latency_ms`
- `fps`
- `tag_count`
- `tag_ids`
- `decision_margins`
- `centers_xy`
- `corners_xy`
- `translations_m`
- `rotations_rvec_rad`
- `euler_deg`
- `distances_m`
- `pose_ambiguities`
- `reprojection_errors_px`
- `field_layout_loaded`
- `field_layout_tag_count`
- `field_length_m`
- `field_width_m`
- `field_pose_valid`
- `field_pose_is_multitag`
- `field_pose_used_tag_count`
- `field_pose_used_tag_ids`
- `field_pose_translation_m`
- `field_pose_quaternion_wxyz`
- `field_pose_euler_deg`
- `field_pose_reprojection_error_px`
- `field_pose_ambiguity`

Per-frame array topics keep the same tag ordering across all arrays.

## Coordinate Frames

The published per-tag pose uses PhotonVision/WPILib-style frames:

- Camera frame: `+X` out of the camera, `+Y` left, `+Z` up
- AprilTag frame: `+X` out of the visible tag face, `+Y` right, `+Z` up

That means a camera looking straight at a tag should report mostly positive `x` translation, with the tag rotation close to a 180 degree flip about `z`.

Internally, OpenCV pose estimation still runs in OpenCV's native camera frame for correctness and compatibility with `solvePnP` and `drawFrameAxes`, then the result is converted before publishing.

### Examples

```bash
./build/apriltag_vision --calibration calib.json
./build/apriltag_vision --calibration calib.json --camera 1 --camera-name frontCam
./build/apriltag_vision --calibration calib.json --team 6328
./build/apriltag_vision --calibration calib.json --nt-server 10.0.0.2 --no-display
./build/apriltag_vision --calibration calib.json --width 1280 --height 720 --fps 60 --family tag36h11 --threads 4 --quad-decimate 1.5 --blur 0 --refine-edges on --pose-iterations 15 --max-error-bits 3 --decision-margin-cutoff 30
./build/apriltag_vision --calibration calib.json --field-layout frc-field.json --team 6328
./build/apriltag_vision --calibration calib.json --stdout-json --record output.mp4
./build/apriltag_vision --calibration calib.json --auto-exposure off --exposure -6 --brightness 128 --auto-white-balance off --white-balance 4200 --orientation cw90 --stream-width 640 --stream-height 360
```

## Calibration JSON format

Expected fields:

- `fx`, `fy`, `cx`, `cy` (numbers)
- `dist_coeffs` (array of numbers, any length)

Example:

```json
{
  "fx": 912.4,
  "fy": 910.8,
  "cx": 640.0,
  "cy": 360.0,
  "dist_coeffs": [-0.112, 0.089, 0.0003, -0.0007, -0.021]
}
```

If the file is missing, malformed, or missing required fields, the app exits with a descriptive error.

## Field Layout JSON format

The app accepts the same general structure PhotonVision and WPILib field layouts use: a top-level `tags` array with `ID`, `pose.translation`, and `pose.rotation.quaternion`, plus an optional `field` object with `length` and `width`.

Example:

```json
{
  "tags": [
    {
      "ID": 1,
      "pose": {
        "translation": {
          "x": 11.863959,
          "y": 7.4114914,
          "z": 0.889
        },
        "rotation": {
          "quaternion": {
            "W": 0.0,
            "X": 0.0,
            "Y": 0.0,
            "Z": 1.0
          }
        }
      }
    }
  ],
  "field": {
    "length": 16.518,
    "width": 8.043
  }
}
```

Notes:

- Field-relative pose output represents the camera pose in field coordinates.
- When only one known tag is visible, the app still produces a field-relative camera pose from that tag.
- When two or more known tags are visible, the app switches to a combined multi-tag solve and marks `field_pose_is_multitag=true`.
- Per-tag `pose_ambiguities` is the ratio `best_reprojection_error / alternate_reprojection_error` from the two-solution IPPE square solve, so lower is better and near-zero is best.
- If a meaningful second solution is not available, `pose_ambiguities` is reported as `-1`.
- `field_pose_ambiguity` is populated for single-tag field poses and reported as `-1` for multi-tag results where a single flip ambiguity value is not meaningful.
- Field layout accuracy matters a lot. If the uploaded tag positions are wrong, the field-relative result will be wrong too.
