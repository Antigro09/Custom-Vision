# Offline calibration desktop workspace

The browser workflow reuses `calibration_session.py` acquisition/selection and the
existing isolated `calibration_solver.py`. It is a separate local desktop
application. It starts no camera, vision pipeline, NetworkTables instance or
runtime controller, and requires no WPILib/controller libraries. The Jetson's
installed Python/CUDA stack is unchanged.
Its dedicated page loads only the calibration UI; the desktop dependency list is
`tools/requirements-calibration-desktop.txt`, separate from producer and browser
test dependencies. Use an isolated desktop environment for any future setup.

From the repository's existing desktop environment:

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python tools/calibration_dashboard.py
```

Open `http://127.0.0.1:5845`. `--data` selects the local saved workspace; its default
is ignored `data/calibration-browser`. `--solver-python` selects an **existing**
isolated mrcal interpreter. The application probes that interpreter and installs
nothing. Stop it with Ctrl-C; owned jobs are canceled and saved inputs remain.
The normal vision dashboard provides a launch hint without starting this service.

The local `vision/calibration-workspace` branch includes the field visualization
commits; no branch switch is needed to review both. To open both workspaces on
one page with explicitly synthetic field observations:

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python tools/field_dashboard_preview.py \
  --port 5844 --calibration-data data/workspace-preview
```

Open `http://127.0.0.1:5844`. The visible synthetic notice applies to field fixture
playback only; imported calibration media and jobs are separate. This preview
disables camera discovery/acquisition, NT and hardware activation. No calibration
solve runs merely by opening it. Ctrl-C stops the server and cancels its own jobs;
saved imports remain. This optional desktop preview starts no normal vision runtime.

## Import, review, solve and export

1. Describe the physical camera and recording history. OV9281 and OV2311 are
   examples of labels, not assumed devices or mode profiles. Enter the measured
   board's inner-corner counts and square spacing in meters. Keep physical camera
   identity, delivered resolution, crop, binning and focus together as one profile.
2. Import untouched images or a prerecorded video. Optional host-read timestamp
   JSONL follows the existing video sidecar format. Native decoded dimensions are
   authoritative; a declared dimension mismatch fails selection. Unknown OBS
   resizing/cropping history remains a warning. No camera is auto-selected.
3. Select views, inspect the actual saved preview, coverage grid, blur/contrast
   metrics and rejection reasons. Change board pose, distance and tilt in a future
   recording when coverage is poor. Coverage measures sampling, not accuracy.
4. Choose a backend explicitly. OpenCV produces a baseline candidate with training
   residuals; held-out validation, sampling uncertainty, spline comparison and
   native mrcal models are **not computed**. It is never silently substituted for
   an unavailable mrcal job. mrcal uses the existing OPENCV8 and separate spline
   workflow only when its real modules/CLI are available. Its temporal path
   requires an aligned recorded host-time sidecar; image order or nominal FPS is
   not relabeled measured capture time.
5. Review diagnostics and export a candidate. Preserve the saved session and native
   `.cameramodel` files with their optimization inputs for later uncertainty
   analysis. Runtime JSON alone cannot retain those inputs. A candidate does not
   establish physical camera accuracy, mount, timing or robot-pose trust.

The mrcal runtime exporter retains exact OPENCV8 `fx, fy, cx, cy` and rational
`k1, k2, p1, p2, k3, k4, k5, k6` coefficients after projection parity checks.
Spline coefficients remain in their native reference model; they are never
renamed OpenCV coefficients. Blocked-time held-out views still come from one
recording. An independent recapture remains a physical validation step. Sampling
uncertainty does not include every systematic error; see the upstream
[projection uncertainty explanation](https://mrcal.secretsauce.net/uncertainty.html).

## Bounds, saved jobs and activation

One calibration process group runs at a time, with CPU thread counts set to one.
Jobs have bounded wall time (maximum 600 seconds), persisted progress, cancellation
and saved terminal outcomes. Cancellation targets only the manager's own group,
with a bounded terminate/kill grace. A service restart marks interrupted jobs
failed and retains their data; it does not adopt or kill unrelated processes.

Uploads are sequential and limited to 12 MiB per image, 32 MiB per video and 2 MiB
per sidecar. Image sets contain at most 100 inputs; the asset store is limited to
256 MiB. Selected views are limited to 180 and video scanning to 10,000 frames.
Opaque asset/session/job/candidate IDs resolve only within the configured store.
Saved candidate artifacts are allowlisted. Uploaded media, real calibrations and
private history remain outside Git.

The offline application exposes export only. When a separately hosted dashboard
has both this manager and a writable `RuntimeController`, activation additionally
requires a reviewed candidate, selected pipeline and explicit confirmation. The
controller validates resolution and available identity/crop/binning/focus
provenance before atomic configuration replacement. Unknown provenance remains
unverified. It saves the previous configuration/artifact references and clears
POI physical verification. Restore changes only that pipeline's prior calibration
reference and requires that its original geometry is unchanged and no later
calibration has replaced the activated one;
unrelated current configuration changes remain intact. Restore also requires
renewed physical verification. Existing dashboard upload behavior is preserved.

The existing same-origin and setup-token checks apply to job writes. Only the
asset-upload route has a larger bounded JSON allowance; configuration writes
retain their 1 MiB bound. The standalone application binds localhost and disables
camera discovery, including device-refresh requests.

## Reproducible checks and native requirements

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest -q -rs
.venv/bin/python tools/calibration_dashboard_browser_qa.py --help
.venv/bin/python tools/calibration_dashboard_browser_qa.py \
  --url http://127.0.0.1:5845 --fixture-store data/calibration-browser \
  --chromium /path/to/existing/chromium-headless-shell
```

Browser QA uses local synthetic chessboard media and real page controls. Synthetic
process adapters verify lifecycle/cancellation only; they do not test mrcal or
calibrate a physical camera. Existing native mrcal integration tests use
`importorskip`; generic CI without mrcal does not establish native integration.
The working desktop environment lacks mrcal, SciPy, mrgingham and
`mrcal-calibrate-cameras`; native solves and uncertainty/model-comparison checks
are unrun. A suitable isolated environment
needs importable mrcal, SciPy, OpenCV, NumPy and the matching mrcal CLI; mrgingham is
required only when explicitly selected. Verify capability output and run the
existing native integration test before claiming that backend passed.
This is not a blanket Mac platform limitation: the
[Mac setup report and exact lock](MRCAL_MAC_SETUP.md) identifies the matching
upstream wheel and the separate workspace-local environment now prepared. Its
native import and doctor checks passed; the single 60-view native test remains
on hold pending review of its CPU/memory/stopping bounds. The working environment
and running preview were preserved.
`--fixture-store` must match the desktop application's `--data` directory. The
harness creates and removes only its own temporary, explicitly invalid calibration
artifacts to test the real HTTP download. Candidate/native diagnostic display
fixtures stay visibly synthetic. Browser test dependencies use the separate
`tools/requirements-field-browser-test.txt`; the harness downloads no browser.

The inspected PhotonVision
[v2026.3.4 build](https://github.com/PhotonVision/photonvision/blob/v2026.3.4/build.gradle)
pins its own mrcal wrapper dependency. That source pin does not identify Anthony's
installed binary or establish an accuracy defect. This application reports
measured interpreter/package/tool capabilities rather than a vague latest label.

## Physical validation remains separate

- Confirm physical camera identity, exact capture mode/crop/binning and locked
  focus. Check independent board dimensions, flatness, printer scale and corner
  ordering; retain original media and capture history.
- Recapture independently, including image edges, corners, near/far distances and
  tilted views; compare residuals, richer-model disagreement and sampling
  uncertainty. Investigate blur, lighting, rolling shutter and systematic error.
- Survey the robot mount separately. Intrinsic board-relative poses are not a
  robot-to-camera mount. Validate independent POI signs and geometry physically.
- Measure capture correction and clock alignment separately, including uncertainty.
  Host-frame read timestamps are not exposure timestamps.
- Validate moving-target/robot behavior, source and power loss, reconnection,
  independent receipt expiry and worst-case CPU/network load on actual hardware.
  No such hardware evidence is provided by these software checks.
