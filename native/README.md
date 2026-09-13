# Native AprilTag core

Build the CPU detector and optional CUDA grayscale preprocessing:

```sh
.venv/bin/python -m pip install pybind11==2.13.6
scripts/build_native.sh
```

This builds a private static AprilTag 3.4.5 (commit
`94be783968e5091bcc9972c72c84fd63efce2935`) and the C++17 Python extension with
`-O3`, CPU-specific instructions, and hidden symbols. It does not install or
replace system libraries. Initial configuration downloads a SHA-256-verified
source archive. Rebuild on another computer instead of copying the compiled
extension. Build files and binaries stay outside Git.

## Real CUDA detection on this JetPack 6 Jetson

```sh
scripts/build_native.sh -DCUSTOM_VISION_CUDA_APRILTAGS=ON
```

This optional build downloads NVIDIA's standalone cuAprilTags header and 1.26 MB
aarch64 static library from Isaac ROS **release 3.2**, pinned to NITROS commit
`e3a29804b6de38fefbf7040864124d983bfceb6a`. It links the existing CUDA 12.6 and
OpenCV CUDA libraries; it requires no ROS installation, Docker, or JetPack change.
The library SHA-256 is
`e67298e8ee52d6253cd702f4c55b3727bea177f57236aa23797bde50d74dfc24`.
The option remains enabled in that CMake build directory until explicitly set
`OFF`; fresh builds default to CPU detector support.

NVIDIA's binary is under the [NVIDIA ISAAC ROS SOFTWARE LICENSE](NVIDIA_ISAAC_ROS_LICENSE.txt),
not the project's source license. The interface header is Apache-2.0. Retain
these notices and the applicable terms if distributing a compiled extension;
the NVIDIA binary is never committed to this repository. The CPU detector is
BSD-2-Clause; its notice is in [APRILTAG_LICENSE.txt](APRILTAG_LICENSE.txt).

Select the runtime detector under a pipeline's settings:

```yaml
backend: native
detector_device: cuda  # cpu is the default; cuda supports tag36h11 only
preprocess: cpu       # grayscale conversion; independent of detector_device
quad_decimate: 2.0
threads: 2
min_decision_margin: 30
max_hamming: 0
cuda_max_tags: 32
```

The GPU searches and decodes the image. NVIDIA's API does not report decision
margin, so the native adapter passes only its candidate quadrilaterals through
upstream AprilTag's CPU edge refinement and code verification. This preserves
real decision margins, the configured hamming filter, and decoded corner order;
it does **not** repeat the CPU full-image search. Calibrated IPPE and MultiTag PnP
remain CPU work. Native frame work releases the Python GIL.

When valid calibration includes distortion, CUDA rectifies the image before
GPU detection, then candidate corners are mapped back to the original image
before CPU verification. All published corners still describe the raw capture
resolution. Metric poses remain invalid without matching calibration. Detection
without calibration is an uncalibrated 2D result; lens distortion may reduce
detection quality.

`quad_decimate` has an explicit detection-range tradeoff on both devices. The CPU
uses the upstream quad-search decimator and decodes at full resolution. CUDA
linearly resizes its search/decode image, rescales candidate corners to the raw
frame, and verifies/refines them at full resolution on CPU. CUDA's initial
smaller-image decode can reject distant tags that full-resolution CPU decoding
would retain. Test values 1, 2, 3, and 4 against the smallest expected tags;
synthetic large tags alone cannot establish field detection range.
GPU mode requires `refine_edges: true` so raw-image edges determine the final
corners; disabling this refinement is rejected. Coordinate mapping includes
the resize pixel-center offset before applying lens distortion.

`min_white_black_diff` (default **5**) is the CPU search's minimum local tile
contrast. Increasing it can greatly reduce noise-induced candidate quads, but
may reject low-contrast tag boundaries. It does not configure NVIDIA's search.
Keep it at the upstream default until contrast/blur/distance tests justify a
change. `quad_sigma` likewise affects the CPU search only. `cuda_max_tags` caps
NVIDIA's output allocation; `last_profile.capacity_reached` reports when the
buffer may have truncated detections.

## Timing and diagnostics

`native_capabilities()` identifies the compiled backends, versions, GPU count,
and native OpenCV thread limit. OpenCV uses one thread, configured once when the
extension loads; each CPU AprilTag detector defaults to two worker threads.
Change `_native.set_opencv_threads()` only before starting camera workers.

Each pipeline exposes `last_timings` (preprocess, detect, pose, total milliseconds)
and an on-demand `last_profile`:

- CPU: upstream threshold, connected components, clustering, quad fitting,
  decoding/refinement timings, and candidate quad count.
- CUDA: upload/expansion/rectification/resize, GPU detection/decode, CPU candidate
  verification timings, candidate count, and output-capacity status.

CUDA initialization, memory allocation, and resolution changes have first-frame
costs; benchmark after warmup and report those startup costs separately. Native
timing excludes Python result construction. Camera exposure, USB transfer, MJPEG
decode, MultiTag localization, NetworkTables delivery, and robot reception are
additional costs. Use outer wall time and capture/robot timestamps to measure
the complete path. Do not infer live latency from synthetic compute tests.

The optional backend is covered by real-GPU tests for rotated tags, grayscale
and BGR inputs, margin filtering, distorted calibration, resolution changes,
blank images, concurrent cameras, and capacity reporting. No physical camera or
on-robot validation is implied.

## Primary references

- [AprilTag 3 source](https://github.com/AprilRobotics/apriltag/tree/v3.4.5)
- [NVIDIA release 3.2 JetPack 6.1/6.2 platform support](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_apriltag/index.html)
- [Standalone cuAprilTags interface](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/blob/e3a29804b6de38fefbf7040864124d983bfceb6a/isaac_ros_nitros/lib/cuapriltags/cuapriltags/cuAprilTags.h)
- [Official benchmark methodology](https://nvidia-isaac-ros.github.io/v/release-3.2/performance/index.html)

NVIDIA reports 720p Orin Nano Super results for its own graph/node. Those are
different workloads from two 1280x800 MJPEG cameras and do not establish this
project's latency. Current Isaac ROS releases targeting JetPack 7 should not be
installed over this JetPack 6 system.
