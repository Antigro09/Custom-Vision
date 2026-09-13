# Latency engineering and measurement

The target is approximately **12–24 ms camera-to-robot latency**. A synthetic
benchmark can guide implementation choices, but cannot certify that target or
establish superiority over PhotonVision/Limelight. Physical cameras were absent
on 2026-09-13. Results below distinguish detector/pose compute from total age.

## Implemented latency controls

- Private optimized C++17 AprilTag detector and single-tag PnP; GIL released while
  processing, zero-copy grayscale input when supplied, reusable native buffers.
- Two independent camera workers, two detector CPU threads per camera, one thread
  per OpenCV operation to avoid nested thread-pool oversubscription.
- Live capture continuously drains the driver; inference consumes the latest
  decoded frame. Intermediate frames are counted and dropped rather than queued.
- MultiTag jointly solves mapped corners and derives target poses, avoiding
  redundant individual PnP when the joint solve succeeds.
- Preview overlay/rotation/resize/JPEG work happens on a separate demand-driven
  worker, capped independently. No viewers means no preview compression.
- NT4 requests 10 ms delivery and flushes; one JSON snapshot preserves frame
  coherence. Time synchronization and freshness checks are separate from FPS.
- Optional CUDA full-frame tag search with full-resolution CPU decode verification
  and PnP. This retains actual decision margin, hamming and decoded corner order.
  Optional grayscale CUDA conversion is a separate setting, not GPU detection.

GPU acceleration does not imply that MJPEG decode, networking, Python orchestration
or joint localization runs on CUDA. The CPU-native and GPU-native paths are both
available for direct comparison. No ROS installation is required.

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
| CUDA + MultiTag, mapped scene | 16.69 | 24.97 | 26.46 | 28.13 |
| CPU + MultiTag, mapped scene | 11.46 | 19.11 | 23.01 | 26.38 |

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
