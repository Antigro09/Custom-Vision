# Latency engineering and measurement

The target is approximately **12–24 ms camera-to-robot latency**. A synthetic
benchmark can guide implementation choices, but cannot certify that target or
establish superiority over PhotonVision/Limelight. Physical cameras were absent
on 2026-09-13. Results below distinguish detector/pose compute from total age.

## Implemented latency controls

- Private optimized C++17 AprilTag detector and single-tag PnP, plus optional
  custom CUDA single-tag and joint MultiTag fitting; GIL released while
  processing, zero-copy grayscale input when supplied, reusable native buffers.
- Two independent camera workers, two detector CPU threads per camera, one thread
  per OpenCV operation to avoid nested thread-pool oversubscription.
- Live capture continuously drains the driver; inference consumes the latest
  decoded frame. Intermediate frames are counted and dropped rather than queued.
- MultiTag jointly solves mapped corners and derives target poses on the selected
  CPU/CUDA pose path, avoiding redundant individual PnP when the joint solve
  succeeds. POI tracking retains independent single-tag observations.
- Preview overlay/rotation/resize/JPEG work happens on a separate demand-driven
  worker, capped independently. No viewers means no preview compression.
- NT4 requests 10 ms delivery and flushes; one JSON snapshot preserves frame
  coherence. Time synchronization and freshness checks are separate from FPS.
- Optional CUDA full-frame tag search with full-resolution CPU decode verification
  and independently selected CPU/CUDA single-tag and joint MultiTag PnP. This retains actual decision
  margin, hamming and decoded corner order.
  Optional grayscale CUDA conversion is a separate setting, not GPU detection.

Selecting `pose_device: cuda` moves single-tag fitting, joint mapped MultiTag
fitting and pose fallbacks to custom CUDA kernels. Coordinate transforms, bearing
and POI geometry, Python orchestration, networking, previews and default MJPEG
decode remain CPU work. CPU and GPU paths are available for direct comparison.
No ROS installation is required.

The latest before/after optimization measurements are in the
[CUDA graph and host geometry report](POSE_OPTIMIZATION_2026-09-20.md).
The earlier [September 20 Jetson report](CUDA_VERIFICATION_2026-09-20.md) compares
CPU/CUDA backends and records the initial solver verification. Both reports
include tail latency and benchmark provenance; their measurements are separate
runs under dynamic clocks and should not be combined as a single comparison.

The [POI and CUDA pose report](POI_AND_CUDA_POSE.md) covers the later custom
single-tag and joint MultiTag solvers, hardware verification and paired
benchmarks. The September 13 tables below predate both custom pose solvers and
used CPU PnP even with CUDA detection. They do not measure the current CUDA joint
solver.

## Measured on this Jetson — 2026-09-13

Two simultaneous processing workers, **300 frames each** after **30 warm-up
frames per worker**, 1280×800 input, 3D mode, two CPU detector threads per camera.
Warm-up occurs on the same worker as measurement; all workers finish before any
GPU detector is destroyed, avoiding teardown synchronization inside a timed run.
The desktop and existing PhotonVision process remained running; clocks were not
locked. Each row is a separate run, not a statistical confidence interval.

| Path and scene | Median ms | p95 ms | p99 ms | Maximum ms |
|---|---:|---:|---:|---:|
| CUDA, decimation 2 | 10.56 | 15.63 | 18.24 | 20.11 |
| CPU, decimation 2 | 31.35 | 39.07 | 46.61 | 52.42 |
| CPU, decimation 2, contrast 15 | 6.92 | 11.18 | 12.98 | 19.43 |
| CPU, decimation 4 | 10.75 | 15.44 | 17.75 | 20.24 |
| CUDA, decimation 2, noise stress | 15.01 | 18.47 | 20.76 | 27.40 |
| CUDA detection + CPU MultiTag, mapped scene | 16.69 | 24.97 | 26.46 | 28.13 |
| CPU detection + CPU MultiTag, mapped scene | 11.46 | 19.11 | 23.01 | 26.38 |

Every run recovered the expected tag set in **600/600 measured frames**. The mapped
scene additionally required a valid joint pose in every frame. These are repeated
synthetic scenes, not field recall results. [Machine-readable run reports](benchmarks/)
include throughput and cold first-frame times. GPU cold first-frame initialization
was hundreds of milliseconds; the runtime withholds frames over its age limit.

The GPU improved the noisy four-tag workload, while CPU was faster on the clean
mapped scene. CPU contrast 15 and decimation 4 also improve speed but change
low-contrast or small-tag recall. The default remains CPU, decimation 2, contrast 5
until real camera recordings establish the best quality/speed tradeoff. CUDA is
built and selectable immediately in the browser on this Jetson. None of these
measurements certifies a 12–24 ms complete camera-to-robot path; the mapped GPU
p95 already exceeds 24 ms before camera and networking costs.

## Reproduce compute benchmarks

```bash
.venv/bin/python scripts/benchmark_apriltags.py --backend native --detector-device cpu --cameras 2 --frames 300 --warmup 30
.venv/bin/python scripts/benchmark_apriltags.py --backend native --detector-device cuda --cameras 2 --frames 300 --warmup 30
.venv/bin/python scripts/benchmark_apriltags.py --backend native --detector-device cuda --cameras 2 --localization --frames 300 --warmup 30
```

The default deterministic 1280×800 scene has four 36h11 tags (112/160-pixel sides),
25 distractor rectangles and weak background pixel noise. `--noise-stress` uses
stronger pixel noise. `--localization` uses a **different three-tag mapped scene**
and includes joint field localization; do not compare its timing as though only
the solver changed. All metrics are wall time around processing, including Python
boundary overhead; concurrent-camera percentiles pool both cameras' samples.

Each run reports p50/p95/p99/max, per-camera throughput and correct tag-set frames.
Repeated identical synthetic frames do not represent field recall, motion blur,
false-positive rate, distant tag accuracy, or a distribution of real field scenes.
Warm-up samples are excluded. Camera exposure, USB, MJPEG decode, NT transport and
robot receipt are excluded; ordinary runs also exclude field localization.

`quad_decimate` trades detection range for speed. GPU mode resizes its search image
then verifies/refines candidates against full-resolution raw pixels. Increasing
CPU `min_white_black_diff` skips low-contrast tiles and can reduce background work,
but can lose dim/distant tags. Keep representative recorded scenes in the tuning
loop; never pick a threshold solely because the synthetic score improves.

## Current CPU/CUDA pose comparisons

`settings.pose_device` now controls single-tag fitting, joint mapped MultiTag and
pose fallbacks. CUDA supports 4/5/8-coefficient pinhole distortion and up to 256
observed mapped tags in one joint solve. Its deterministic multiple-start search
is not a proof that every geometric solution was found. Compare validated poses
and accepted IDs, not just runtimes. Coordinate transforms, bearing/POI math,
publication and previews remain CPU work.

Use matched rendered-image settings to compare the full detector/enrichment path:

```bash
.venv/bin/python scripts/benchmark_apriltags.py --backend native --detector-device cuda \
  --pose-device cpu --cameras 2 --localization --poi --frames 300 --warmup 30 \
  --output data/apriltags-cpu-pose-new.json
.venv/bin/python scripts/benchmark_apriltags.py --backend native --detector-device cuda \
  --pose-device cuda --cameras 2 --localization --poi --frames 300 --warmup 30 \
  --output data/apriltags-cuda-pose-new.json
.venv/bin/python scripts/benchmark_multitag.py --devices cpu cuda --cameras 2 \
  --tags 2 4 8 16 --layouts planar nonplanar --rounds 2 --frames 200 --warmup 20 \
  --output data/multitag-new.json
```

POI forces independent single-tag poses before the joint solve. Omit `--poi` from
both image runs to measure the runtime's normal skip-single optimization. Keep
all other settings identical and reverse run order on subsequent repetitions.
The separate MultiTag benchmark isolates complete `Localization.enrich` calls
using independently generated known-world corners, twelve oblique camera views,
planar/nonplanar layouts and optional `--noise-px 0.1`. It bypasses tag detection.
It checks every pose against known camera translation/rotation and independent
pixel projection, verifies accepted tag IDs, and rejects invalid or wrong-device
results. It uses the same runtime factory to bind CPU/CUDA solvers.

Both benchmark tools save raw samples, effective settings and source/binary
hashes; they refuse to overwrite output. Input projection/copying and independent
correctness checks are outside measured intervals, with finish barriers preventing
validation or CUDA teardown from contaminating peer timings. Joint
`localization.gpu_timings` separates kernel event time, native solver work and
outer native-call time. Those are nested measurements, not additive costs. Use
full-call wall time and sustained worker FPS for comparisons. No newer GPU
speedup can be inferred from the September 13 tables above.

The current GPU regression command is:

```bash
.venv/bin/python -m pytest -q -ra tests/test_cuda_pose.py tests/test_cuda_multitag.py
```

Compute Sanitizer instrumentation remains blocked by the recorded Jetson debug
access restriction; normal test execution does not establish a successful
sanitizer run. See [verification and rerun details](POI_AND_CUDA_POSE.md#jetson-verification--2026-09-20).

## Live acceptance test

1. Confirm actual negotiated MJPG mode and frame interval for each connected
   camera, stable source paths, USB topology and exposure. Repeat with two cameras.
2. Record raw tag scenes at measured distances, all image regions, oblique angles,
   motion, dim lighting and background clutter. Include empty scenes and false-tag
   distractors. Compare CPU/GPU recall, false detections, reprojection error and
   measured robot pose error at each decimation setting.
3. Measure exposure-to-host delay with a controlled visual time stimulus, and
   validate host-to-roboRIO timestamp conversion. Configure a measured capture
   correction only after this test; a fixed correction cannot remove all jitter.
4. Measure capture-to-pose, publication-to-receipt and pose-estimator age separately.
   Report p50/p95/p99, maximum, dropped frames and stale rejections over sustained
   runs. Include preview open/closed and two-camera load. FPS is not latency.
5. Observe CPU/GPU clocks, thermal throttling, memory and power while running.
   This Jetson is already MAXN_SUPER, but clock locking was not performed because
   administrator access was unavailable. A power mode alone does not lock clocks.
6. Compare PhotonVision and Limelight on a shared measurement method and scene,
   matching resolution, calibration, exposure, tag size and accuracy requirements.
   For different cameras/hardware, report those differences explicitly.

Two cameras are asynchronous; their poses should be fused by capture timestamp on
the robot with appropriate uncertainty, accounting for shared-tag correlation.
Do not average the most recently received poses blindly. Start with a dedicated
AprilTag Jetson; evaluate neural inference contention before combining workloads.

## Reference designs

[PhotonVision MultiTag](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html)
informs joint mapped-tag localization and optional single-tag solving.
[Limelight localization](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization)
and [WPILib axes](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
inform field/robot pose conventions.
[NVIDIA Isaac ROS 3.2 performance methodology](https://nvidia-isaac-ros.github.io/v/release-3.2/performance/index.html)
is a useful GPU reference, but its playback measurements are not measurements of
these USB cameras or this application's complete camera-to-robot path.

## YOLO26 object processing — 2026-09-13

Actual FP16 TensorRT 10.3 engines built on this Jetson from official COCO pretrained
YOLO26n and YOLO26n-seg, exported by Ultralytics 8.4.150. Static batch 1, 640x640
network input, FP32 I/O, workspace 256 MiB. The local Ultralytics `bus.jpg` fixture
was resized to 1280x800 and repeatedly processed; each model found five COCO objects
in every measured frame. These are trained generic models, not team game-piece
models. No physical camera or team accuracy result is implied.

Each worker warmed up for 30 frames, then processed 300 frames; two-worker rows
pool 600 samples. Each worker owns its engine/context/stream. Runs are sequential,
with no engine building, previews or other test jobs during measurement. The
desktop and existing PhotonVision remained running, with unlocked clocks. These
are short runs of one image, not a sustained thermal or varied-scene benchmark.

| Model | Camera workers | Median ms | p95 ms | p99 ms | FPS per camera |
|---|---:|---:|---:|---:|---:|
| YOLO26n boxes | 1 | 16.494 | 18.195 | 19.221 | 61.62 |
| YOLO26n boxes | 2 | 16.674 | 19.491 | 22.363 | 59.12 / 59.13 |
| YOLO26n-seg boxes +5 masks | 1 | 16.735 | 20.196 | 22.766 | 58.86 |
| YOLO26n-seg boxes +5 masks | 2 | 23.577 | 32.723 | 34.774 | 41.87 / 41.52 |

Timing surrounds the complete detector call: letterbox/normalization, CUDA
transfers, TensorRT inference, box decoding and bounded mask contours. It excludes
exposure, camera delivery/MJPEG decoding, metric geometry, NT transport and robot
receipt. The two-camera segmentation p95 already exceeds the 24 ms goal before
those additional costs. Start with boxes; evaluate masks against actual floor
position error and acquisition success before accepting their extra work.

Two-camera median stage timings were 3.092 ms preprocessing /12.577 ms inference /
0.862 ms decoding for detection, and 3.721/12.734/7.303 ms for segmentation. Stage
medians need not sum to the median total. Maxima were 23.609 ms and 35.827 ms.
Cold first calls were roughly 88–202 ms; the runtime's age guard withholds over-age
frames. Live camera rate remains limited by the negotiated sensor mode.

Native channel splitting and contiguous normalization avoid the previous FP64
packing path and reuse scratch storage. CPU-only interleaved checks reduced
letterbox-plus-packing median 3.114→2.063 ms for FP32 and 9.686→2.651 ms for FP16;
FP32 differs by at most one ULP, FP16 is bitwise identical across all 256 values.
The earlier two-camera detection run before this change was 16.680 ms median /
17.963 ms p95. Thus these short unlocked-clock runs do **not** establish an overall
latency improvement; only the isolated preprocessing improvement was measured.

The final decoder was also compared with the upstream PyTorch FP32 predictor on
the same image and NMS-free head. Both models matched all five detections; minimum
box IoU was 0.998765 for detection and 0.997343 for segmentation, with maximum score
difference 0.002496. Our bounded prototype-resolution contours had IoU 0.840–0.923
against the largest component of upstream full-resolution masks. This validates
export/decoder agreement on one fixture, not mask or game-piece accuracy. It also
quantifies the outline approximation rather than treating a 32-point contour as
an exact full mask.

The [machine-readable reports](benchmarks/) include model/input hashes, stage
timings and reference comparisons. Model manifests retain original class order;
weights/ONNX/engines remain in ignored `models/benchmark/`.

```bash
.venv/bin/python scripts/benchmark_objects.py \
  --manifest models/benchmark/detect640/yolo26n.json \
  --engine models/benchmark/detect640/yolo26n.engine \
  --image .venv-export/lib/python3.10/site-packages/ultralytics/assets/bus.jpg \
  --cameras 2 --frames 300 --warmup 30
```

For segmentation use the `segment640/yolo26n-seg` manifest and engine. Before
deployment, repeat on actual color-camera recordings, measured target positions,
empty/confusing scenes and the final fine-tuned model, including sustained
two-camera load and the complete robot reception path.
