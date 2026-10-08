# Native synthetic calibration check on this Mac

The requested native integration test passed without a skip in the isolated
CPython 3.12.14 / mrcal 2.5.2.post1 environment. All four real CLI fits completed:
OPENCV8 training/full and spline training/full. The output remains a calibration
candidate with status `needs_review`; no runtime activation occurred.

Before running, the adapter was corrected for mrcal 2.5's `rt_ref_frame` field.
It still accepts numeric legacy `frames_rt_toref` from older mrcal, rejects invalid
pose arrays and indices, and preserves board-to-camera coordinates. mrcal 2.5's
deliberately poisoned legacy key is never treated as numeric data. The 22 focused
tests pass and are included in the full ordinary regression count below.

## Solver and validation evidence

The deterministic fixture rendered 60 monochrome 640x480 views of a 7x5-corner
board with 0.03 m spacing. Forty-eight views train each model and twelve views
are held out by the existing deterministic time-block split. This is synthetic
holdout data, not an independent physical recapture. All 60 views were accepted,
with no rejected corners and 93.75% corner-cell coverage.

| Diagnostic | OPENCV8 | Spline |
| --- | ---: | ---: |
| Held-out RMS, px | 0.120281 | 0.134952 |
| Held-out p95 corner error, px | 0.227046 | 0.239054 |
| Sampling uncertainty p95, px | 0.360814 | 2.574821 |
| Sampling uncertainty maximum, px | 0.733790 | 4.513403 |

Both held-out RMS values pass the test's 0.6 px threshold. The 24x16 sampling
uncertainty grids are finite. Spline p95 uncertainty exceeds the 0.5 px review
threshold. Rotation-aligned spline/OPENCV8 disagreement is 2.736904 px p95 and
8.417048 px maximum, also exceeding the 0.5 px p95 review threshold. These two
findings correctly keep the candidate under review; no thresholds were relaxed.
Model disagreement measures a diagnostic, not ground-truth physical error.

Exact OPENCV8 export produced `fx=600.4989186`, `fy=605.5030649` against fixture
values 600 and 605, with eight rational OpenCV distortion coefficients.
The exporter checked mrcal/OpenCV projection agreement at its existing tolerance.
Both native models retain optimization inputs. Sixty board-to-camera poses were
exported, with `robot_to_camera=null` and `robot_mount_calibrated=false`.

## Bounds and measured resources

The supervisor checked 65,104,543,744 free RAM bytes and 818,352,336,896 free disk
bytes before launch, exceeding the reserved 2 GiB RAM / 256 MiB artifact estimates.
It requested one compute thread, with a 300-second timeout for each fit and a
1,800-second overall deadline. OpenCV parallel regions/OpenCL were disabled;
loaded OpenBLAS and OpenMP pools reported one thread. Apple Accelerate received
`VECLIB_MAXIMUM_THREADS=1`. This is a thread request, not a CPU-affinity guarantee.

| Measurement | Observed |
| --- | ---: |
| Supervised wall time | 5.444397 s |
| pytest test time | 4.99 s |
| wait4 user CPU | 4.868561 s |
| wait4 system CPU | 0.302249 s |
| Sampled aggregate group peak RSS | 239,779,840 bytes (228.67 MiB) |
| wait4 maximum RSS | 189,759,488 bytes |
| Artifacts including summary/verification receipts | 1,938,852 logical bytes |

RSS was sampled at approximately half-second intervals and may miss brief peaks.
The wait4 resource scope is the direct child and descendants it waited for; its
maximum RSS is not the aggregate group peak. Artifact bytes exclude symlink
targets to avoid counting the same generated image repeatedly. No bound was
exceeded. Native group 72591 exited normally, required no termination signals,
had its direct child reaped and had no remaining group members.

Four short sleeping-process checks separately passed deadline, early-exit,
cancellation and injected monitoring-error cleanup. The supervisor sends TERM
only to its owned process group, waits up to five seconds, then sends KILL if
needed. The self-checks use a shorter half-second grace. A monitoring error does
not prevent cleanup. JUnit must show exactly one unskipped passing test, along
with a completed model report and four solver logs, before native success is
reported.

The preserved `.venv` regression suite passed **672 tests with 67 skips** in
30.38 seconds. Those skips still include native dependencies unavailable in that
working environment; the separate native test above actually ran. All 21 working
environment distributions, interpreter and prefix match the earlier manifest.
Existing previews remained listening under their original PIDs: combined vision
preview on `127.0.0.1:5844` (69952) and the separate desktop preview on
`127.0.0.1:3001` (14847). No camera/GPU stack or system setting changed.

## Reproduction and retained artifacts

After the [hash-pinned environment setup](MRCAL_MAC_SETUP.md):

```sh
.venv/bin/python tools/native_calibration_check.py --self-test
.venv/bin/python tools/native_calibration_check.py
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 \
OPENCV_FOR_THREADS_NUM=1 VISION_TEST_GUI=0 .venv/bin/python -m pytest -q
```

Process monitoring requires the host's permitted process access. If unavailable,
the supervisor fails preflight before launching a child. Each run creates a new
directory under ignored `data/native-calibration-runs/`. This run is
`981ae811-2a56-4915-a2a0-b032be2cbd75`, containing `pytest.log`, `junit.xml`,
`preflight.json`, `resources.json`, `evidence-summary.json`, `verification.json`
and the generated session/models/diagnostics under `pytest-tmp/`.
The cleanup self-check run is `b902ba76-0ee5-41f2-a1ab-ba55952f3e8d`.
Setup/import/manifest evidence remains under `data/native-mrcal-setup/`.

Resource receipt SHA-256:
`e8f921752d983254ac239bc55acbb0ce09f9b7adcbf11cad15f9a75040996368`.
Calibration report SHA-256:
`8f9b97754eabb759f3a689140d8f838f694aa56a0f1bd3d2207fb7d2cb40bcb1`.
Source and artifact hashes are retained in the verification/summary receipts.
Generated images, models and native environment remain ignored and local.

The mrgingham path remains unrun; this check uses OpenCV-SB. Physical camera,
board survey, mount, capture correction, motion, worst-case load and on-robot
accuracy remain unverified. No controller integration, deployment, camera access,
GPU work, AGI scheduling changes, package publication or new GUI push occurred.
