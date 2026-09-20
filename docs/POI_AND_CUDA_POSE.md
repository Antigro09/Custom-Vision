# Tag-relative POI, processed FPS, and custom CUDA PnP

This change adds an aiming observation, not robot actuator code. No odometry,
field layout, shooter model, or robot motion is invented. Native CPU PnP remains
the default; the optional CUDA PnP backend has executable Jetson tests and still
requires physical camera/robot acceptance before competition use.

## The flattened box is intentional

The existing overlay has a square base of `tag_size_m` and a depth of half that
size. It is a short rectangular prism, not evidence by itself of bad intrinsics.
The new `preview.box_depth_ratio` is 0.5 by default; set 1.0 for a cube. Its range
is 0.1–2.0. This only changes preview geometry. Incorrect intrinsics can still
skew either representation. The colored axes retain their half-tag length.

## POI: aim at an offset from a currently visible tag

The concept is the same as Limelight's point-of-interest tracking: configure a
3D offset from the tag and report angular offsets to that point. A field map or
`.fmap` is not needed for this tag-relative operation. This implementation does
not import Limelight `.fmap` files or claim wire compatibility with Limelight.

In each AprilTag pipeline:

```yaml
poi:
  enabled: true
  calibration_verified: false
  max_ambiguity: 0.2
  max_reprojection_error_px: 3.0
  targets:
    - name: aim_target
      tag_id: 7  # EXAMPLE: replace with your actual tag ID.
      offset_m: [0.0, 0.0, 0.30]  # EXAMPLE: 30 cm above this tag's center.
preview:
  box_depth_ratio: 1.0
```

Offsets are meters in this repository's **WPILib TAG frame**, not its robot or
OpenCV camera frame: X points out of the tag's printed front toward the viewer;
Y points right when looking at the upright printed tag; Z points up. The origin
is the tag center, with orientation determined by its decoded bits. Rotating the
printed tag rotates its offset. For example, six inches left and two inches
behind the front of the tag is `[-0.0508, -0.1524, 0.0]` in THIS convention.

The computation is `p_camera = R_camera_tag * p_tag + t_camera_tag`, using the
current single-tag pose, before field localization enrichment. POI enabled
therefore prevents skipping individual PnP just because a MultiTag solve exists.
`tx_deg = atan2(x_camera, z_camera)` is positive right. `ty_deg =
atan2(-y_camera, z_camera)` is positive up. These are planar optical-axis angles;
`ty` is not spherical elevation when horizontal offset is nonzero. They are
angles to the POI, not the tag's yaw/pitch orientation.

With a measured `robot_to_camera`, additional robot-coordinate translation,
robot yaw (positive left), and elevation are supplied. Camera bearings do not
require that mount, but cannot correct camera-to-shooter parallax or provide a
robot-center steering angle by themselves. No ballistic/drop calculation,
velocity compensation, occlusion detection, obstacle checking, or control loop
is included. If the tag disappears there is no prediction or retained aim target.
Multiple configured entries are prioritized in explicit configuration order,
never averaged across cameras or unrelated physical POIs.

### Calibration and pose validity

`calibration_verified` is an operator acknowledgement after physical validation,
not a programmatic calibration certificate. It defaults to false. A calibration
marked `benchmark_only: true` NEVER becomes a valid aiming calibration, even if
this checkbox is selected. A nominal calibration can show **orange PREVIEW ONLY**
geometry but leaves `poi.valid` and the typed NT aim flag false. Valid observations
are magenta. Finite pose, positive POI depth, matching image/calibration resolution,
unique visible tag ID, finite known ambiguity and a separate reprojection threshold
are required. The POI threshold defaults to 3 px and is bounded at 10 px. Increasing
the detector's threshold to 1000 px for a nominal-intrinsics visualization does
not bypass this separate check. Low residual alone never proves true accuracy.
Uploading a changed calibration or selecting a different calibration path clears
the previous acknowledgement. Uploading identical normalized calibration preserves
it. A manually overwritten file at the same path is outside the controller;
clear and recheck its acknowledgement after physical validation.

The JSON contains `poi.targets`, `selected_name`, `valid`, `invalid_reason`,
`capture_monotonic_us`, projected pixels, camera angles/translation and optional
robot coordinates. Unverified entries explicitly have `valid: false` even when
geometric numbers are available for preview. A POI outside the image can retain
valid geometry if it is in front of the camera; its crosshair is not drawn.

New convenience topics under each existing pipeline's NT table:

| Topic | Type | Meaning when `poi_valid` is true |
|---|---|---|
| `fps` | double | Smoothed processed FPS, not sensor/preview FPS |
| `poi_valid` | boolean | Selected current POI passed the checks |
| `poi_name` / `poi_tag_id` | string / integer | Selected configured point and observed tag |
| `poi_tx_deg` / `poi_ty_deg` | double | Right-positive / up-positive camera angles |
| `poi_camera_xyz` | double[] | OpenCV camera x-right, y-down, z-forward, meters |
| `poi_robot_xyz` | double[] | Robot x-forward, y-left, z-up, meters; requires mount |
| `poi_robot_yaw_deg` | double[] | One positive-left bearing, or empty without mount/defined bearing |

Loss/rejection/disconnect/shutdown clears the flag, names, IDs and numeric/vector
convenience topics. Values of zero are not themselves validity indicators. As
with existing topics, use the coherent per-frame JSON and its capture time for
cross-field consistency; separate NT topics are convenience values, not one atomic
multi-topic transaction. Robot consumers must reject stale observations and require
validity/time synchronization as appropriate for their application. The top-level
`pose_valid` topic is still field-robot pose validity, distinct from POI validity.

## Processed FPS and stage times

The dashboard replaces the prominent frame ID with processed FPS and keeps latency.
It uses a smoothed measured interval between processed frames, not `1000/latency`
and not the preview's 10 FPS. Frame IDs remain in raw JSON for debugging. Disconnect
or stale runtime data displays zero processed FPS. The stage strip shows tag detect,
pose, enrichment and queue milliseconds; pose device is separate from detector device.
`pose_device: none` means no single-tag solve ran that frame. Deferred single-tag
fallback calls are timed and identified under `localization.single_tag_fallback`;
`single_tag_pose_ms` includes that work. Joint field solving is separately labeled
with its actual `localization.pose_device`, CPU or CUDA. Skipped single-tag work
can therefore coexist with a valid CUDA field solve. The configured `pose_device`
controls both fitting paths and any deferred single-tag calls.
The UI includes pose-device selection, POI settings and the box-depth ratio.

## Optional custom CUDA PnP

On the Jetson checkout:

```bash
bash scripts/build_native.sh \
  -DCUSTOM_VISION_CUDA_APRILTAGS=ON \
  -DCUSTOM_VISION_CUDA_POSE=ON \
  -DCMAKE_CUDA_ARCHITECTURES=87
.venv/bin/python -c 'from custom_vision.native_apriltags import native_capabilities; print(native_capabilities())'
```

This builds project code with existing CUDA and OpenCV libraries. It does not
install/upgrade JetPack or NVIDIA runtime packages. CPU-only CI can compile CUDA
for Orin without a GPU; that is not CUDA execution. Build on the Jetson itself,
not by copying an x86 extension from CI. CMake remembers options in its build directory.

Select independently:

```yaml
settings:
  backend: native
  mode: 3d
  detector_device: cuda  # existing cuAprilTags search; may also be cpu
  preprocess: cpu
  pose_device: cuda      # single-tag, joint field and deferred PnP; cpu is default
  cuda_pose_iterations: 30
  multitag: true         # uploaded field coordinates enable the joint solve
```

The full square-tag GPU solver includes parallel inverse rational distortion, homography,
both IPPE initial hypotheses, translation solving, and damped nonlinear refinement
of the best hypothesis. No CPU PnP seed is supplied. The original-hypothesis ambiguity
and alternate solution are retained; refinement never turns two branches into a
false unique-pose claim. Each tag uses one cooperative 32-thread block; Jacobian
rows and normal-equation entries are parallel, small pivoted solves use one lane.
A closed-form square homography replaces an 8-by-8 elimination, and a centered
translation least-squares solve avoids another elimination without reducing precision.
Tag batches share one launch per camera frame (chunks of at most 256). Each detector
owns a persistent nonblocking stream and events. Orin uses persistent mapped
pinned host input/output buffers, sharing its physical memory directly with CUDA
and eliminating two transfers per batch. Inputs are loaded once into shared memory
before iterative reuse. Discrete GPUs use pinned asynchronous transfers and device
buffers instead. Per-event completion waits avoid changing process-wide CUDA
scheduling; they actively wait for the short solve and can occupy a CPU core.
Float64 and ordinary finite-value checks are retained; no fast-math is enabled.

### Joint mapped MultiTag on CUDA

With `multitag: true` and a field layout, the same pose-device setting also selects
custom CUDA joint fitting. The runtime factory connects the detector's native
`estimate_multitag` entry point to localization; it never supplies a CPU PnP seed
or invokes CPU `solvePnP` as a hidden fallback. A joint call supports **2–256
observed mapped tags**, each represented by its four original image corners and
matching known field corners. This limit is separate from field-file size and
CUDA detector output capacity. No joint field pose is fabricated from unknown
or contradictory tags.

The GPU computes inverse distortion, tag-derived dual IPPE seeds, shared planar
initialization, whole-tag consensus, robust starts and joint nonlinear refinement.
Accepted corners constrain one field-to-camera pose; individual tag poses are
not averaged. Planar ambiguity preserves the two initial plane solutions.
Nonplanar fitting compares distinct refined candidates. The deterministic finite
set of starting poses **does not prove that all possible minima were found**;
quality gates and independent physical validation remain necessary.

Joint fitting uploads observations and field corners once per call into persistent
device buffers, so repeated reads across hypotheses and iterations can use GPU
caches. Its small result remains mapped on Orin. This differs from the single-tag
path's one-time shared-memory load: pinned host memory is GPU-uncached on Orin.
See NVIDIA's [Tegra memory table](https://docs.nvidia.com/cuda/cuda-for-tegra-appnote/index.html#memory-management).
Each nonplanar refinement block checks its own initial depths and residuals;
the initialization stage does not repeat the same projections serially for every
hypothesis. Neither optimization reduces the starting poses or quality checks.

All accepted tag corners must have positive camera depth, and each accepted
tag's RMS reprojection error must pass the configured bound. Native results
include accepted/rejected indices and `per_tag_reprojection_error_px` in input
order. Localization exposes accepted errors as `tag_reprojection_errors_px`,
keyed by tag ID, and reuses these GPU-computed errors for field-derived target
poses without CPU reprojection or additional fitting. Degenerate or inconsistent
input and failed fits remain invalid. Any separate single-tag observations needed
for POI, unknown tags or permitted fallback use the selected CUDA solver too.

Both GPU pose paths support OpenCV pinhole distortion with **4, 5 or 8
coefficients**; 12/14-coefficient models require explicit CPU selection. Missing
CUDA build/device, calibration or mode 3D fails explicitly. A rejected completed
single-tag pose is not redundantly re-solved by the localization layer. Numerical
tests independently reproject GPU outputs with OpenCV.

The CPU still performs coordinate/basis transforms, Rodrigues/quaternion output
conversion, finite-result checks, calibrated bearing and POI geometry, camera
management, NetworkTables and UI/preview work. Default MJPEG decoding and CUDA
detector candidate quality verification also remain CPU work. Selecting CUDA
PnP accelerates fitting rather than moving every application operation to the GPU.

`native_timings.pose_ms` includes host orchestration, copies, synchronization,
finite checks and conversion inside the native pose stage. `pose_kernel_ms` is
CUDA-event kernel time only. Do not compare that number alone to CPU wall time.
The entire Python/native boundary remains included in the outer detector timing.
Joint solves report a separate `localization.gpu_timings` object:

| Field | Scope |
|---|---|
| `pose_kernel_ms` | CUDA event time around the joint kernel sequence |
| `pose_ms` | Native joint solver work, including buffer handling and synchronization |
| `call_ms` | Outer native joint call, including Python/native conversion and waiting |

These measurements overlap; do not add them. The complete `Localization.enrich`
call also performs coordinate and target-output work beyond the native call.

## Measure rather than assume a speedup

Run actual hardware tests before trusting this experimental backend:

```bash
.venv/bin/python -m pytest -q -ra tests/test_cuda_pose.py tests/test_cuda_multitag.py tests/test_poi.py tests/test_pnp_math.py
.venv/bin/python scripts/benchmark_pose.py --cameras 2 --tags 1 2 4 8 16 32 \
  --frames 300 --warmup 30 --output logs/pose-$(date +%Y%m%d-%H%M%S).json
```

The benchmark uses known synthetic corners, not camera images. It calls the real
selected native backend, reverses CPU/GPU test order on alternating rounds and
includes full pose-call wall time. Invalid known-pose results abort rather than
produce favorable timing. Every measured output is checked against known translation,
rotation and independent OpenCV projection after all workers finish timing, so
validation cannot delay another timed worker. Native-stage, kernel, outer-call and
outer-minus-native times are separate. The first call and warmed p50/p95/p99/max are
separate; the first call is not a cold-process/context startup measurement.
Two workers own independent instances and wait for each other before teardown.
`--devices cpu` is available without GPU support. Camera transport, detection and
NT costs are intentionally absent from this pose-only benchmark.


Joint localization has a separate benchmark using the same `app.make_detector`
factory as the runtime:

```bash
.venv/bin/python scripts/benchmark_multitag.py --devices cpu cuda --cameras 2 \
  --tags 2 4 8 16 --layouts planar nonplanar --rounds 2 --frames 200 --warmup 20 \
  --output logs/multitag-new-run.json
# Optional deterministic corner noise; use a different output file:
.venv/bin/python scripts/benchmark_multitag.py --devices cpu cuda --cameras 2 \
  --tags 2 4 8 16 --layouts planar nonplanar --noise-px 0.1 \
  --output logs/multitag-noisy-new-run.json
```

Inputs are independently projected from known field layouts and twelve modestly
varying oblique camera poses with eight-coefficient distortion. No test module
supplies fixtures or truth. Each frame starts with fresh deferred-pose detections;
POI is disabled to isolate joint localization. Complete call wall time and nested
GPU/native times are reported with raw samples, source/binary hashes, accepted
IDs, independently checked camera translation/rotation and pixel residuals.
Corner generation and input copies occur before timing; correctness checks wait
until all workers finish. CPU/GPU order reverses each round. Existing output files
are refused. These are corner-input compute measurements, not detector throughput
or physical camera accuracy.

For real-camera comparison, copy a working local YAML to separate new test files,
keep detector device/cameras/resolutions/exposure/quality settings identical, and
change **only `pose_device`**. Keep NetworkTables disabled during nominal testing.
Run CPU and CUDA sequentially with the same visible tags; stop one before the other.
Use the existing `--headless --no-nt --stdout-json --max-frames 3000` flags to save
JSONL, then:

```bash
.venv/bin/python scripts/summarize_vision_log.py logs/YOUR-RUN.jsonl
```

The log tool counts errors/stale/shutdown observations, reports rejected poses,
excludes the first 30 connected frames per pipeline/boot from timing summaries,
and reports p50/p95/p99. Logged FPS is not reconstructed camera delivery throughput.
Host-read-to-publication latency does not include exposure/USB/MJPEG read time or
robot receipt; JSON logging adds overhead. Zero-distortion nominal intrinsics omit
the CUDA detector's distortion-remap work, so they do not represent final calibrated
performance. Changed calibration can also change rejection and iteration behavior.

Two cameras at 25–30 processed FPS do not establish that PnP is the bottleneck.
Small tag batches may be faster on the CPU, especially with Float64 arithmetic,
launch/sync cost and result conversion. Inspect detection, decode/capture, queue,
USB sharing and power/thermal behavior before promising a speed target. Preserve
quality while comparing, and never select a backend solely to prove a preferred
hardware choice. Improvements must be demonstrated on the same real workload.

## References and verification scope

- Limelight POI: https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d#point-of-interest-tracking
- OpenCV IPPE/planar PnP and coordinate order: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
- IPPE implementation reference: https://github.com/opencv/opencv/blob/4.10.0/modules/calib3d/src/ippe.cpp
- NVIDIA profiling and transfer-overhead guidance: https://docs.nvidia.com/cuda/cuda-c-best-practices-guide/index.html
- NVIDIA Tegra memory/synchronization guidance: https://docs.nvidia.com/cuda/cuda-for-tegra-appnote/index.html

The shared single-tag header `native/pnp_math.hpp` has executable CPU
equivalence, Jacobian and degeneracy tests. The joint math in
`native/cuda_multitag_math.hpp` is covered through actual GPU integration tests,
not a separate CPU equivalence/Jacobian suite. Joint GPU tests cover
planar/nonplanar layouts, known field poses, whole-tag outliers, duplicate IDs,
ambiguity, mapped-tag capacity, concurrent instances and per-tag residuals.
GPU integration tests require actual hardware and explicitly skip without it. The CUDA CI workflow compiles the .cu source with nvcc for Orin and
tests the CPU runtime; hosted CI does not supply a GPU. POI tests cover axes,
priority, calibration/quality gates, absent/duplicate tags and fresh-state semantics;
UI tests cover FPS and configuration round trips. Retain IPPE/OpenCV notices when
redistributing a compiled build. Physical calibration, measured offsets, field
accuracy, sustained camera load and robot-side acceptance remain separate.

The attached draft's earlier container-only results are superseded by the Jetson
verification recorded below. No physical cameras are connected during this work;
the measurements are synthetic compute, not camera-to-robot latency.

## Jetson verification — 2026-09-20

See the [complete verification report](CUDA_VERIFICATION_2026-09-20.md) for paired
one/two-worker measurements, accuracy limits and source/binary checksums. CUDA
improves some workloads; small batches and some complete image pipelines favor
CPU execution, so the explicit CPU option remains available.

The final CUDA 12.6 build for Orin `sm_87` includes detection, single-tag PnP and
joint MultiTag kernels. The full local suite passed **549 tests, with 3 explicit
skips**; dashboard client tests passed **15/15**. The skips are the real mrcal and
mrgingham integrations unavailable in the main environment, and the negative
CUDA-unavailable test because CUDA was actually available. Existing guided
calibration code and its isolated installer are preserved.

The focused custom CUDA/mathematical suite passed **83 tests plus one intentional
skip**, included within the full count. Another **360 seeded stress cases** passed
against independent known camera poses and OpenCV reprojection/reference fits.
They combine planar, nearly planar and nonplanar layouts, noise, whole/partial-tag
outliers and transformed field coordinates. Capacity tests also exercise 256 tags
followed by smaller, invalid and valid inputs on the same persistent workspace.
These runs execute real CUDA kernels; they are not emulated GPU results.

Numerical coverage includes noisy and arbitrary-orientation poses, distorted
projection, frontal/rotated ambiguity, analytic Jacobian checks, degenerate
quadrilaterals, invalid mixed batches, 300-pose chunking, concurrent camera
instances and same-instance serialization. GPU outputs are independently
reprojected with OpenCV, including their alternate solutions. POI tests exercise
decoded tag orientation, axes, mounts, calibration replacement, priority, rejection
and live NT4 loopback publication/clearing. These tests validate software geometry,
not a measured real camera or a robot aiming mechanism.

**CUDA sanitizer instrumentation remains unverified.** The installed NVIDIA
Compute Sanitizer refused instrumentation with “GPU debugging features are
disabled.” The current user is outside the `debug` group, the relevant device
nodes are `root:debug`, and noninteractive sudo requires a password. The GPU
single-tag tests still ran during that earlier attempt, but execution without
instrumentation is not a successful memcheck or proof of no memory races/leaks.
The newer joint kernels likewise remain unverified by sanitizer instrumentation.
NVIDIA documents the debug-group requirement in its [Jetson/Tegra sanitizer instructions](https://docs.nvidia.com/compute-sanitizer/ComputeSanitizer/index.html#using-the-compute-sanitizer-on-jetson-and-tegra-devices).
After an administrator enables that access and the login/session is refreshed,
run separately from performance measurements:

```bash
compute-sanitizer --tool memcheck --leak-check full --error-exitcode 99 \
  .venv/bin/python -m pytest -q tests/test_cuda_pose.py tests/test_cuda_multitag.py
compute-sanitizer --tool synccheck --error-exitcode 99 \
  .venv/bin/python -m pytest -q tests/test_cuda_pose.py tests/test_cuda_multitag.py
compute-sanitizer --tool racecheck --error-exitcode 99 \
  .venv/bin/python -m pytest -q tests/test_cuda_pose.py tests/test_cuda_multitag.py
```
