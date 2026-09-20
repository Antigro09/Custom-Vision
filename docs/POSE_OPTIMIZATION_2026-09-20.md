# Pose throughput optimization — September 20 follow-up

The second optimization pass reduces repeated CPU geometry work and CUDA launch
overhead. On this Jetson, two-worker joint MultiTag median processing times fell
15–60% across the eight tested layouts/counts. The rendered three-tag scene with
POI and full CUDA detection/pose improved from 19.994 to 17.161 ms median, with
p95 improving from 26.302 to 23.938 ms. These are synthetic compute measurements,
not camera-to-robot latency or an accuracy claim about physical cameras.

## Changes retained

- Joint CUDA fitting replays its transfers, nine kernel stages and timing events
  through a per-detector CUDA graph. An eight-entry LRU cache supports changing
  tag counts. Each call supplies fresh image and field corners; poses and
  consensus are never cached. New/evicted shapes require graph setup.
- Joint normal-equation accumulation omits zero-filled padding beyond the actual
  corners in a tile. It preserves the order of every real corner's contribution.
- Host localization reuses the joint camera transform, avoids converting target
  rotations to Rodrigues vectors and back just to format relative poses, and
  uses scalar arithmetic instead of small NumPy temporaries for quaternion/RPY
  output. The stable Rodrigues conversion around pi remains.

The solver still uses the same FP64 math, candidate starts, iteration limit,
robust consensus, ambiguity tests and rejection thresholds. CUDA single-tag and
joint fallback behavior, POI independence, and the NetworkTables schema remain.
The native graph uses external event-record nodes so host completion and GPU
timings refer to the executed graph; a first attempt using internal capture
events failed hardware tests and was corrected before the measurements below.
See the [native implementation notes](../native/README.md#timing-and-diagnostics)
for the CUDA documentation and lifecycle details.

## Paired measurements

Baseline: commit `3f7f72859334cfacdd6965309f22a049c4e299f2`, including its original
native binary. The comparison ran separate, sequential baseline/updated Python
processes, reversing revision order in the second round. Both revisions used
the same project environment, benchmark scripts, synthetic fixtures and settings.
The device remained in its existing configuration with dynamic clocks, desktop
and other services running. No project benchmark or test overlapped another.

Joint tests used 2/4/8/16 tags, planar/nonplanar geometry, one/two workers,
20 warmup calls and 80 measured calls per worker per round. They time the complete
`Localization.enrich` call, including host output formatting. The image tests used
1280×800 BGR input, quad decimation 2, CPU grayscale preprocessing, two workers,
20 warmup frames and 150 measured frames per worker per round, with POI and joint
localization enabled. Validation ran after both workers finished timing.

All **7,680 joint calls and 4,800 rendered-image frames** passed the existing
independent geometry, expected-ID, quality and actual-device checks. Percentiles
below pool raw samples across workers and rounds. They are not averages of
per-run percentiles. FPS is measured from each worker's elapsed processing loop.

Two-worker joint localization, milliseconds **median / p95**:

| Layout | Tags | Before | After |
| --- | ---: | ---: | ---: |
| Planar | 2 | 3.655 / 7.765 | 2.624 / 6.455 |
| Planar | 4 | 6.340 / 10.000 | 4.261 / 7.229 |
| Planar | 8 | 9.616 / 15.590 | 5.514 / 10.528 |
| Planar | 16 | 16.338 / 20.908 | 13.868 / 17.884 |
| Nonplanar | 2 | 4.382 / 7.602 | 1.765 / 6.580 |
| Nonplanar | 4 | 6.142 / 10.630 | 3.198 / 6.574 |
| Nonplanar | 8 | 8.451 / 12.350 | 5.662 / 11.864 |
| Nonplanar | 16 | 14.205 / 19.794 | 10.489 / 16.693 |

Two-worker mapped-image pipeline, including detection, POI and localization:

| Detector / pose | Before median / p95 ms | After median / p95 ms | Before FPS/worker | After FPS/worker |
| --- | ---: | ---: | ---: | ---: |
| CPU / CPU | 12.555 / 19.560 | 10.558 / 14.520 | 70–81 | 89–103 |
| CPU / CUDA | 11.583 / 17.281 | 10.207 / 16.441 | 81–86 | 84–99 |
| CUDA / CPU | 17.530 / 25.423 | 18.121 / 24.965 | 55–58 | 52–58 |
| CUDA / CUDA | 19.994 / 26.302 | 17.161 / 23.938 | 49–50 | 56–57 |

CUDA detection with CPU pose had a 3.4% higher median in this paired comparison;
its p95 was slightly lower. Some one-worker joint p95 values also increased even
though all joint medians decreased. GPU event times include concurrent GPU work
and do not all improve. This is evidence for the measured complete-call gains,
not a claim that every stage, configuration or tail improved. Defaults remain
explicit CPU, and both detector/pose choices remain available.

## Verification and resource limits

New tests exercise graph-cache eviction with changed observations/field geometry,
invalid-to-valid replay, and rotation serialization at zero, pi and gimbal lock.
Existing tests cover noisy/outlier geometry, ambiguity, large field origins,
256-tag capacity, concurrent cameras and same-instance serialization.

The initial complete run produced 548 passes, three expected skips and three
setup errors from one synthetic TensorRT FP16 segmentation-engine fixture. Its
GPU allocation failed while the 8 GB device's swap was full. All eight TensorRT
tests passed in a separate fresh process. The remaining suite then passed
**543 tests with three expected skips**, giving **551 distinct passing tests**
across the two isolated runs. The **15 dashboard tests** and both AprilTag/object
runtime smokes passed too. Splitting avoids combining offline engine-building
memory with all other integration-test state; object GPU tests were not skipped.

```sh
.venv/bin/python -m pytest -q --ignore=tests/test_tensorrt_integration.py
.venv/bin/python -m pytest -q tests/test_tensorrt_integration.py
node --test tests/dashboard_client.test.cjs
```

An eight-cycle create/solve/close memory probe showed stable resident memory after
warmup for both revisions. With four cached graph shapes, the updated process
used about 2.5 MiB more resident memory than baseline. This small cache cost is
not a memory reduction; the optimization reduces repeated computation and host
submissions. System-wide GPU memory counters include other desktop processes,
and this probe is not a formal leak or sanitizer check.

No physical cameras were connected. Capture, exposure, USB, MJPEG decode, preview,
NetworkTables and robot receipt are excluded. Power/energy efficiency was not
measured in a controlled comparison. GPU sanitizer instrumentation remains
unverified because this account lacks device-debugging access. A graph cache miss
has setup cost; these warmed tables do not bound first-frame or worst-case latency.

## Provenance and reproduction

The [machine-readable summary](benchmarks/2026-09-20-pose-optimization.json)
contains all 20 comparisons, one-worker results, p99/maxima, first-call samples,
accuracy maxima, source/native hashes and checksums for 24 local raw reports.
The verified updated native binary SHA-256 is
`d075a764392c99d4fe592b4f3eaa78ea95c7196b4f0948660920fb0804897c9a`.
Local raw reports, test logs, memory probes, thermal samples and sequential
orchestration are under ignored `data/pose-optimization-2026-09-20/`.

Build both revisions using the same documented native CUDA options. Run each
command in its corresponding trusted checkout with the same Python environment,
using a fresh output path each time, and reverse revision order for round two:

```sh
.venv/bin/python scripts/benchmark_multitag.py --devices cuda --cameras 2 \
  --frames 80 --warmup 20 --rounds 1 --output data/joint-comparison.json
.venv/bin/python scripts/benchmark_apriltags.py --backend native --mode 3d \
  --detector-device cuda --pose-device cuda --cameras 2 --frames 150 --warmup 20 \
  --poi --localization --output data/image-comparison.json
```

Repeat joint tests with one worker and image tests with all four detector/pose
combinations. Do not run the two revisions concurrently or substitute old timing
tables for a fresh baseline under the current device load.
