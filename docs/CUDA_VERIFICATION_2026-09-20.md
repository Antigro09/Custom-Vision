# CUDA pose verification on Jetson — 2026-09-20

The custom single-tag and joint MultiTag pose solvers execute on the actual
Jetson GPU and pass the software checks below. CUDA mode includes joint field
fitting and deferred single-tag fallback, without CPU PnP seeds or a hidden CPU
fitting fallback. This verifies synthetic geometry and measured compute on this
Jetson; it does not establish real-camera accuracy or camera-to-robot latency.

CUDA improved the two-worker isolated joint localization comparisons, but did not
win every workload. With two workers, CUDA detection and pose achieved 67–72 compute FPS
per worker on the four-tag image; on the mapped POI/MultiTag image it achieved
50–56, versus 77–81 with CPU detection and pose. Small single-tag batches and
single-tag tail latency also favored CPU. Backend choice needs the actual camera
scene and full processing workload, not a blanket assumption that CUDA is faster.

## Hardware and implementation scope

- NVIDIA Jetson Orin Nano Super, 8 GB, aarch64 Ubuntu 22.04, L4T 36.4.7,
  CUDA 12.6; native CUDA build targets Orin `sm_87`.
- CPU and CUDA tests use the same native library and matching inputs. Each
  concurrent worker owns its detector/solver instance and persistent workspace.
- `pose_device: cuda` selects single-tag, joint MultiTag, and deferred fallback
  pose fitting. Supported CUDA camera models have 4, 5, or 8 OpenCV pinhole
  distortion coefficients. A joint call supports 2–256 observed mapped tags.
- Coordinate conversion, bearing/POI geometry, capture, UI, and NetworkTables
  remain CPU work. CUDA detection also retains CPU candidate verification.
  Selecting CUDA pose does not move the entire application onto the GPU.
- Joint initialization and refinement search deterministic candidate starts and
  reject invalid or ambiguous results. Finite multistart is not a proof of a
  global optimum or a unique physical pose.

No physical cameras were connected. Dynamic clocks and the existing desktop and
services were left running. Benchmark configurations ran sequentially, with no
other project GPU tests or benchmarks in parallel. This is not an isolated,
clock-locked laboratory maximum-throughput result.

## Correctness and regression checks

| Check | Observed result | What it establishes |
| --- | --- | --- |
| Full Python suite | 549 passed, 3 skipped | Geometry, runtime, configuration, NT4, object, calibration-workflow, and native/GPU regressions covered by the suite |
| Dashboard client suite | 15 passed | FPS display, actual solver labels, and configuration/POI round trips |
| Focused CUDA/math suite | 83 passed, 1 intentional skip | Real GPU pose coverage and shared single-tag mathematical checks; these passes are already included in the 549 |
| Additional seeded joint stress checks | 360 cases, zero reported failures | Generated planar, nearly planar, and nonplanar layouts with noise, outliers, and changed field coordinates |
| Persistent joint workspace capacity | 256 tags, then reused with smaller/invalid/valid inputs | Maximum supported joint input and reuse without stale consensus or mutation of earlier results |
| AprilTag runtime smoke | 10 frames per mode; 2D, single PnP, MultiTag passed; shutdown cleared | Synthetic video/runtime integration using the native backend; this smoke alone is not CUDA verification |
| Object runtime smoke | 16 valid and 4 empty frames; stale/lost and shutdown outputs cleared | Synthetic ball-candidate geometry/tracking/output behavior with NT disabled; not trained game-piece accuracy |
| Final benchmark matrix | 36 raw artifacts, 72 pooled configurations; all checks passed | 14,400 single-tag batch calls, 9,600 joint calls, and 7,200 rendered-image frames checked across both worker counts |
| Live browser and HTTP verification | Passed against the synthetic CUDA demo | FPS/latency, actual solver devices, POI, cube depth, save/restart, 2D clearing, and shutdown state |

The three full-suite skips are the real `mrgingham` command and `mrcal` Python
integration unavailable in the main environment, plus the CUDA-unavailable
negative test because CUDA was available. Guided calibration remains implemented;
those two skipped external integrations were not validated by this run.

GPU tests independently reproject returned poses, including alternate solutions,
with OpenCV and compare against known camera transforms. Coverage includes
distortion, tag orientation, ambiguous/frontal views, noisy and degenerate inputs,
mixed valid/invalid batches, concurrent instances, single-tag chunking, whole-tag
outliers, duplicate IDs, and joint per-tag residuals. CPU equivalence and analytic
Jacobian tests exercise the shared **single-tag** `native/pnp_math.hpp`; joint
math in `native/cuda_multitag_math.hpp` is covered by real GPU integration tests,
not a separate CPU equivalence/Jacobian suite.

Stress fitting success is not permission to use every generated pose on a robot:
configured ambiguity and quality gates can still reject a result. POI tests cover
tag-relative axes, mounts, priority, calibration replacement, validity, and NT4
publication/clearing; no physical aiming mechanism was tested.

The browser showed processed FPS, latency, and actual CUDA field solving on the
synthetic preview. Saving box depth 0.5 then 1.0 persisted both values; disabling
POI let the single-tag solve become “not run” while joint CUDA remained valid.
Switching to 2D retained the three decoded IDs, reset the pose setting to CPU,
and cleared metric poses; restoring 3D/CUDA/POI restored both CUDA solve labels.
Each save changed the runtime boot ID, confirming restart, and the existing
session could continue saving. The synthetic calibration remained preview-only.
No browser console errors appeared before intentional shutdown; afterward the
UI showed unavailable, zero FPS/tags, no valid POI, and removed the preview.

## Benchmark method

The final matrix compares one and two independent workers. CPU/CUDA order is
reversed in the second round. All measured results are stored during timing and
checked after **all** peer workers finish; correctness checks cannot occupy the
Python interpreter while another worker is still being timed. Warmup, fixture
generation, validation, initialization, and teardown are excluded from the
reported warmed timings. Worker FPS is measured from each worker's elapsed loop,
not calculated by inverting a pooled latency percentile.

| Workload | Inputs | Timed calls per worker per round | Warmup | Rounds |
| --- | --- | ---: | ---: | ---: |
| Single-tag pose batches | 1, 2, 4, 8, 16, or 32 independently projected tags | 200 | 30 | 2 |
| Joint field localization | 2, 4, 8, or 16 mapped tags; planar and nonplanar layouts | 100 | 20 | 2 |
| Rendered-image AprilTag pipeline | Four-tag scene, or three mapped tags with POI and MultiTag; all four detector/pose device combinations | 150 | 20 | 2 |

Single-tag measurements include the complete native pose API call. Joint
measurements include the complete `Localization.enrich` call, using fresh
deferred-pose detections, eight-coefficient distortion, and twelve moderately
oblique known camera poses. POI is disabled in that isolated joint test. Both the
joint and rendered-image benchmarks use the same `app.make_detector` wiring as
the runtime, so requested CUDA field solving is checked as CUDA rather than
silently substituted with CPU fitting.

The clean joint fixture uses a benchmark-specific ambiguity limit of 0.5; it does
not change the production default. Its independent limits are 0.5 mm translation,
0.02° rotation, and 0.001 px reprojection RMS. Accepted tag IDs and actual device
must also match. Single-tag limits are 1 mm, 0.05°, and 0.001 px. Noise/outlier
robustness is separately exercised by tests/stress checks; the final latency
matrix uses clean pose fixtures, not the optional noisy joint benchmark mode.

Rendered-image timings include detection, pose fitting, and, in the mapped
scene, tag-relative POI followed by joint localization. Images are 1280×800;
quad decimation is 2 and preprocessing remains CPU for every device combination.
The nominal zero-distortion image fixture omits calibrated CUDA detector remap
cost. These repeated rendered images do not reproduce real lighting, blur,
occlusion, or exposure changes.

All 7,200 rendered-image frames passed their expected-ID, pose validity, and
device checks. The mapped scene additionally checks valid joint localization,
selected POI and finite bearings, with 0.1 m gross-error limits for the camera
and POI positions to allow for rasterized tag edges. These image checks are less
strict than the independent clean-corner pose tolerances above.

**Every timing excludes physical camera exposure, USB transport, MJPEG capture/
decode, preview rendering/encoding, NetworkTables, and robot receipt.** Pose-only
tables also exclude tag detection. Native/CUDA kernel times are nested portions
of full call time, not extra stages to add to it. Joint kernel event time excludes
input transfers and host/Python work; comparing it alone to CPU wall time would
misstate the benefit.

## Two-worker results

The tables pool raw samples across both workers and both rounds; percentiles
are not averages of per-run percentiles. The companion
[machine-readable summary](benchmarks/2026-09-20-cuda-summary.json) records the
one-worker results too, p99/maxima, source and native-binary SHA-256 hashes,
accuracy maxima, and the checksums of all 36 local raw artifacts.

The verified native binary SHA-256 is
`402245c4ec3fe3e7e70053121a72ca91a10e076cf168736a8c9c50bdd4705a84`.
The summary, generated at `2026-09-20T17:44:04Z`, records hashes for 16 source
files. All matrix artifacts used that same binary and consistent source hashes.

### Single-tag batches

Full pose-call milliseconds, **p50 / p95**. Each device/count has 800 timed batch calls.

| Tags per batch | CPU | CUDA |
| ---: | ---: | ---: |
| 1 | 0.113 / 0.170 | 0.167 / 2.193 |
| 2 | 0.244 / 0.341 | 0.216 / 2.693 |
| 4 | 0.388 / 0.549 | 0.239 / 3.475 |
| 8 | 0.742 / 0.947 | 0.297 / 3.795 |
| 16 | 1.478 / 2.004 | 0.390 / 3.588 |
| 32 | 2.867 / 3.442 | 0.657 / 4.234 |

CPU was faster at one tag, and its p95 was lower at every tested batch size.
CUDA reduced the median from two tags onward, but that median improvement did
not guarantee higher throughput for small batches. For example, four-tag CPU
workers processed 2,095–2,437 batches/s, versus 1,611–2,113 for CUDA. These are
pose-only calls, not achievable camera frame rates. CUDA kernel medians ranged
from 0.110 to 0.286 ms; the full-call tail above remains the relevant cost to the
application. This experiment does not isolate the cause of those tail delays.

### Joint field localization

Complete localization-call milliseconds, **p50 / p95**. Each device/layout/count has 400 timed calls.

| Layout | Mapped tags | CPU | CUDA |
| --- | ---: | ---: | ---: |
| Planar | 2 | 8.510 / 10.854 | 4.447 / 7.615 |
| Planar | 4 | 8.012 / 15.336 | 5.861 / 8.997 |
| Planar | 8 | 15.971 / 19.996 | 9.407 / 15.056 |
| Planar | 16 | 23.051 / 32.385 | 16.531 / 20.625 |
| Nonplanar | 2 | 5.457 / 8.545 | 5.197 / 7.650 |
| Nonplanar | 4 | 9.717 / 12.468 | 5.253 / 8.047 |
| Nonplanar | 8 | 12.866 / 19.308 | 8.400 / 13.464 |
| Nonplanar | 16 | 23.276 / 31.731 | 14.315 / 20.561 |

CUDA lowered both p50 and p95 in these eight two-worker joint comparisons.
Across all one/two-worker joint configurations, independent errors stayed below
3.13 × 10⁻⁷ m translation, 5.54 × 10⁻⁶ degrees rotation, and 2.40 × 10⁻⁶ px
reprojection RMS, all below the declared clean-fixture limits. Such tiny errors
describe exact synthetic inputs; they are not a real-camera accuracy estimate.
Every clean joint frame accepted all expected IDs without single-tag fallback.

### Rendered-image pipeline

Full compute milliseconds, **p50 / p95**, followed by measured per-worker FPS
range across both rounds. Each row has 600 verified frames. “Pose CUDA” includes
single-tag fitting, and joint field fitting when enabled.

| Scene | Detector | Pose | p50 / p95 ms | FPS per worker |
| --- | --- | --- | ---: | ---: |
| Four tags | CPU | CPU | 33.476 / 41.584 | 28.4–29.6 |
| Four tags | CPU | CUDA | 33.987 / 43.271 | 28.1–28.7 |
| Four tags | CUDA | CPU | 13.906 / 17.926 | 73.7–78.5 |
| Four tags | CUDA | CUDA | 14.926 / 17.727 | 67.0–72.1 |
| Three tags + POI + MultiTag | CPU | CPU | 12.610 / 18.415 | 77.1–80.6 |
| Three tags + POI + MultiTag | CPU | CUDA | 12.406 / 17.952 | 77.6–82.5 |
| Three tags + POI + MultiTag | CUDA | CPU | 17.135 / 24.759 | 55.3–59.3 |
| Three tags + POI + MultiTag | CUDA | CUDA | 18.058 / 25.159 | 50.5–55.6 |

On the four-tag image, CUDA detection produced the largest improvement. Adding
CUDA pose to CUDA detection increased the median from 13.906 to 14.926 ms and
reduced throughput, although p95 was slightly lower. On the mapped image, CPU
detection was faster; CPU detection plus CUDA pose was only slightly ahead of
CPU/CPU in this run. The fully CUDA configuration was slower than both CPU
detector configurations for that scene.

The isolated joint speedup therefore does **not** imply a faster complete image
pipeline. These fixtures differ in tag size, image content, and enabled work;
compare device choices within the same scene. The data does not justify an
“always use CUDA” policy, a sustained camera FPS guarantee, or a camera-to-robot
latency claim. Keep both explicit backend choices and repeat the comparison with
calibrated cameras and representative field footage.

## Reproduction and remaining checks

Use distinct output paths: benchmark scripts refuse to overwrite an existing
report. Run each command sequentially on the target Jetson, outside other GPU
testing or profiling. Repeat with `--cameras 1` for the one-worker comparisons.

```bash
.venv/bin/python -m pytest -q -ra
node --test tests/dashboard_client.test.cjs
.venv/bin/python -m pytest -q -ra tests/test_cuda_pose.py tests/test_cuda_multitag.py tests/test_pnp_math.py

.venv/bin/python scripts/benchmark_pose.py --cameras 2 --tags 1 2 4 8 16 32 \
  --frames 200 --warmup 30 --rounds 2 --output logs/pose-new-run.json
.venv/bin/python scripts/benchmark_multitag.py --cameras 2 --tags 2 4 8 16 \
  --layouts planar nonplanar --frames 100 --warmup 20 --rounds 2 \
  --output logs/multitag-new-run.json
.venv/bin/python scripts/benchmark_apriltags.py --backend native --mode 3d \
  --decimate 2 --preprocess cpu --detector-device cuda --pose-device cuda \
  --cameras 2 --frames 150 --warmup 20 --poi --localization \
  --output logs/pipeline-cuda-new-run.json
```

For the pipeline matrix, repeat the last command with detector/pose CPU/CPU,
CPU/CUDA, CUDA/CPU, and CUDA/CUDA, then reverse the order for the second round.
Also repeat without `--poi --localization` for the four-tag scene. Use a new
output path for every configuration and round. A source or binary change makes
that a different experiment; preserve the recorded hashes with the results.

**Compute Sanitizer remains blocked.** NVIDIA's tool refused instrumentation
with “GPU debugging features are disabled.” The current account lacks the
required debug access, and noninteractive sudo requires a password. Normal CUDA
execution passing does not establish clean memcheck, racecheck, or synccheck
results. After an administrator enables access and refreshes the session, run
these separately from performance measurements:

```bash
compute-sanitizer --tool memcheck --leak-check full --error-exitcode 99 \
  .venv/bin/python -m pytest -q tests/test_cuda_pose.py tests/test_cuda_multitag.py
compute-sanitizer --tool synccheck --error-exitcode 99 \
  .venv/bin/python -m pytest -q tests/test_cuda_pose.py tests/test_cuda_multitag.py
compute-sanitizer --tool racecheck --error-exitcode 99 \
  .venv/bin/python -m pytest -q tests/test_cuda_pose.py tests/test_cuda_multitag.py
```

Real camera calibration, measured mounts and POI offsets, live two-camera USB
load, power/thermal behavior over a match, end-to-end timestamp validation, and
robot acceptance remain to be tested. No comparison against PhotonVision or
Limelight was performed. Robot Java integration and autonomous motion remain
deferred.
