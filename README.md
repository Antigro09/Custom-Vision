# apriltag_vision

A C++ AprilTag 36h11 detection pipeline for FRC-style vision applications using:

- **apriltag** (official C library, linked as `libapriltag`)
- **OpenCV 4.x** (camera I/O, image processing, pose estimation, visualization)
- **NTCore/CameraServer stack** (`ntcore`, `cscore`, `cameraserver`, `wpiutil`) for dashboard feed + NetworkTables
- **nlohmann/json** (calibration parsing + optional debug stdout JSON)
- **CMake**

The runtime keeps the hot path on raw camera frames for lower latency and better pose correctness:

`capture -> grayscale -> detect -> solvePnP -> annotate -> NetworkTables -> CameraServer stream`

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
cmake -B build -S .
cmake --build build -j
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
./build/apriltag_vision --calibration calib.json [options]
```

### CLI flags

- `--camera <int>` camera index (default: `0`)
- `--camera-name <name>` CameraServer and NT camera name (default: `camera0`)
- `--tag-size <double>` tag size in meters (default: `0.1651`)
- `--calibration <path>` calibration JSON file (**required**)
- `--team <number>` connect NT client to a robot team number
- `--nt-server <host>` connect NT client to a specific host, overrides `--team`
- `--width <int>` requested camera width
- `--height <int>` requested camera height
- `--fps <int>` requested camera FPS
- `--threads <int>` AprilTag detector thread count
- `--quad-decimate <float>` AprilTag detector quad decimation
- `--decision-margin <double>` minimum accepted decision margin
- `--no-display` disable the local `cv::imshow` window only
- `--stdout-json` print debug JSON to stdout in addition to NT publishing
- `--record <output.mp4>` save the annotated output video

If neither `--team` nor `--nt-server` is provided, the app starts a local NetworkTables server for desktop testing.

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

Per-frame array topics keep the same tag ordering across all arrays.

### Examples

```bash
./build/apriltag_vision --calibration calib.json
./build/apriltag_vision --calibration calib.json --camera 1 --camera-name frontCam
./build/apriltag_vision --calibration calib.json --team 6328
./build/apriltag_vision --calibration calib.json --nt-server 10.0.0.2 --no-display
./build/apriltag_vision --calibration calib.json --width 1280 --height 720 --fps 60 --threads 4 --quad-decimate 1.5
./build/apriltag_vision --calibration calib.json --stdout-json --record output.mp4
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
