# Guided chessboard calibration

The desktop calibration workspace adds an explicitly selected chessboard preview,
saved snapshots, image review and solved board diagnostics to the existing offline
workflow. Opening the page starts no camera or solver. The standalone launcher
binds localhost, publishes no NetworkTables data and cannot activate a runtime
calibration. Synthetic and recorded sources stay labeled throughout the workflow.

## Start the synthetic preview

Run from the repository root using the existing desktop environment:

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 \
.venv/bin/python tools/calibration_dashboard.py \
  --guided-capture --synthetic-preview --port 5846 \
  --data data/guided-calibration-preview
```

Open `http://127.0.0.1:5846`. This separate port preserves the combined field and
offline preview on 5844 and the other desktop preview on 3001. The saved workspace
is under ignored `data/`; original images, real calibrations, native models and
job artifacts must remain out of Git. Ctrl-C stops this application's owned
preview and jobs while retaining saved observations.

If the existing isolated native environment is available, select it explicitly:

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 \
.venv/bin/python tools/calibration_dashboard.py \
  --guided-capture --synthetic-preview --port 5846 \
  --data data/guided-calibration-preview \
  --solver-python "$PWD/data/native-mrcal-mac-env/bin/python"
```

Use one launcher at a time on 5846. The application preserves the specified
interpreter and prepends its adjacent `bin` directory to worker/probe `PATH`, so
the matching `mrcal-calibrate-cameras` CLI can be discovered. It installs nothing.
Missing modules or CLI programs leave the native backend unavailable; they do
not trigger an OpenCV substitution. See the [isolated Mac setup](MRCAL_MAC_SETUP.md)
and [prior native synthetic evidence](MRCAL_MAC_NATIVE_CHECK.md). Those recorded
results do not qualify a new camera, board or guided recording.

Uploaded local videos can supply explicitly selected recorded previews. Physical
Linux V4L2 sources require explicit launcher registration and an ownership check;
HTTP requests cannot choose arbitrary camera indices, device paths or URLs. No
Mac webcam is selected implicitly.

On Linux, a deliberately selected camera can be registered without opening it:

```sh
.venv/bin/python tools/calibration_dashboard.py --guided-capture \
  --camera-device /dev/video0 --camera-physical-id team-camera-serial \
  --camera-width 640 --camera-height 480 --camera-fps 10
```

This command is a configuration example, not hardware evidence. Mac registration
is rejected. Crop, binning and focus remain unknown in this simple registration;
unknown declarations permit preview/export but block physical activation. A trusted
backend with verified mode metadata can register the complete source through
`CaptureSource`; a camera label never supplies those measurements.

## Collect and inspect observations

1. Enter the board's **inner-corner** columns and rows. A 7 by 5 configuration
   expects 35 detected corners from an 8 by 6 square chessboard. Keep the complete
   board in view; this implementation does not use partial-board detections.
   Measure printed square spacing, board flatness and printer scale independently.
2. Enter square spacing in **millimeters** in the GUI. A value of `30` is stored
   as `square_size_m: 0.03`; session and solver geometry remain in meters. Declare
   camera identity, delivered mode, crop, binning and fixed/manual focus when
   known. Labels such as OV9281 or OV2311 supply no measured intrinsics or identity.
3. Choose a source, confirm it and start the preview explicitly. The synthetic
   source renders a chessboard and uses the actual OpenCV corner detector; its
   corners and motion are synthetic evidence. A recorded preview is playback,
   with no claim of live sensor exposure timing.
4. Inspect detected/expected corner count, actual detected-corner overlay,
   edge-transition quality and contrast. Save a snapshot only after its exact
   displayed frame is ready and the board passes selection. Missing/incomplete,
   stale, duplicate or poor-quality observations are rejected. A saved snapshot
   keeps its lossless original image separately from the corner overlay.
5. Vary board location, distance, scale and tilt, including image edges and
   corners. Approximately 25 varied views for an initial parametric assessment
   and 50 or more for richer sampling are collection guidance, not acceptance
   thresholds. The selected minimum-view setting, holdout requirements and
   diagnostic gates still apply; more near-identical images do not establish
   better calibration.
6. Stop preview, inspect saved observations and acknowledge the selection before
   requesting a solve. Acquisition and solving cannot run together. Changing the
   board, camera, mode or selection declaration creates a new capture session;
   old observations remain available rather than being silently relabeled.

The **calibration mosaic** is a contact sheet of actual saved images with their
detected corners. It supports visual review and downloading originals. The
**accumulated 2D corner coverage** is a separate normalized plot of measured
image coordinates and a coverage grid. It contains no stitched image and
measures sampling rather than calibration accuracy.

## Time provenance and solver diagnostics

Guided acquisition records host monotonic time in nanoseconds, normally at
`host_frame_read_complete`. That includes read/transport/decode effects; it is
not an exposure timestamp. Synthetic and recorded playback times describe the
current host preview, not original sensor acquisition or synchronized robot time.
These local session fields do not change the producer protocol's JSON `_us`
microsecond fields.

An optional physical backend may supply exposure start/duration and a validated
mapping into host monotonic time. Only that trusted backend metadata, together
with the registered source's verified mapping, enables
`hardware_exposure_midpoint = exposure_start + floor(duration / 2)`. An exposure
duration alone, a GUI declaration or a nominal frame rate cannot enable it.
Rolling-shutter assumptions remain explicit, and a midpoint does not establish
row exposure times, capture correction or clock synchronization. The supplied
OpenCV V4L2 reader does not invent that metadata.

Native guided solves use explicit `image_groups` holdout: consecutive selected
images are grouped in fives, and whole groups are held out deterministically.
This is lens validation using grouped images, with **no temporal independence
claim**. The split requires at least five groups and sufficient remaining
training/held-out observations. Recorded, aligned host timestamp sidecars support
the separate existing `time_blocks` workflow; neither method is an independent
physical recapture.

The native workflow fits OPENCV8 and, unless explicitly disabled, a separate
spline reference. It reports held-out residuals, sampling uncertainty and model
disagreement. Runtime export retains the exact OPENCV8 intrinsics and rational
distortion coefficients after projection checks. Spline coefficients remain in
their native model. OpenCV baseline output provides training residuals; native
holdout, uncertainty and spline comparison remain marked not computed.

The 3D diagnostic view transforms the configured chessboard by retained solved
board poses in **camera optical coordinates**: +X right, +Y down, +Z forward,
meters. Orbit/zoom, camera axes, board outlines and per-image visibility help
inspect those poses. This is board projection geometry, not arbitrary scene
reconstruction, a field pose or a robot-to-camera mount. Missing or invalid
native poses produce an unavailable diagnostic rather than an invented pose.

## Review, activation and limits

Default, imported custom file and latest result are displayed separately from an
active runtime calibration. Import, solve, review and export do not activate a
camera. Quality that is unknown, conflicting, stale or `needs_review`, or a report
with unresolved issues, blocks software review/activation. A review marker is
bound to the current geometry revision and accepted quality. Additional captured
observations invalidate a candidate tied to the older capture revision.

The standalone desktop workspace has no runtime activation controller. Where a
separately hosted writable controller is explicitly provided, activation requires
an eligible reviewed result, exact camera identity and matching resolution, crop,
binning and locked focus, plus an explicit pipeline confirmation. Synthetic
evidence cannot activate a physical camera. Activation retains prior references
for constrained restore and clears physical/POI calibration acknowledgement.
Review and activation never certify a physical camera, mount or timing model.

Default guided preview is capped at 5 FPS and one megapixel, retains at most ten
recent frames and expires live status after 1.5 seconds. Snapshot selection is
bounded to 180 views. One solver job runs at a time, with a configured wall limit
of at most 600 seconds and owned-process cancellation. Upload limits remain
12 MiB/image, 32 MiB/video, 2 MiB/timestamp sidecar and 100 imported images; local
input/captured-image budgets are bounded. CPU worker environment variables request
one compute thread; they are not a hardware resource or timing qualification.

Physical camera leases coordinate cooperating runtime/calibration readers. Linux
foreign-reader checking must succeed for guided physical acquisition. Advisory
locks and a reader probe cannot reserve a camera against every unrelated process.
Failed reader release retains ownership and blocks a new acquisition; no other
application or service is killed to obtain a camera.
Arbitrary runtime GStreamer pipelines have no parsed cooperative device lease and
are excluded from guided acquisition; the foreign-reader probe remains a limited
check rather than an exclusive camera reservation.

## Reproduce software checks

Run the required CPU suite and the pure UI contracts:

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 \
  .venv/bin/python -m pytest -q
node --test tests/calibration_dashboard.test.cjs \
  tests/calibration_capture_ui.test.cjs tests/calibration_diagnostics.test.cjs
```

The guided native check is opt-in and uses the existing isolated Mac environment:

```sh
.venv/bin/python tools/native_calibration_check.py --guided --deadline 180
```

It retains synthetic images, native models, logs and resource receipts under
ignored `data/native-calibration-runs/`. The monitor samples the launched process
and ancestry-observed descendant groups, including the separate worker group,
with 2 GiB RSS and 256 MiB artifact limits. Sampling may miss brief descendants or
new sessions whose ancestry vanishes between samples; these are observed bounds,
not kernel resource isolation. `--self-test` exercises cleanup without fitting.

On 2026-10-08 the actual guided native test passed, including four real fits,
held-out checks, uncertainty, exact OPENCV8 export, sixty matched board poses and
downloaded provenance. The monitored run completed in 5.81 seconds with sampled
aggregate RSS 401.64 MiB and no remaining observed process groups. Its synthetic
candidate stayed `needs_review`: spline uncertainty and lens-model disagreement
exceeded existing gates. This is successful diagnostic execution, not an eligible
physical calibration. Native guided validation used `image_groups` explicitly.

The final Mac CPU suite passed 887 tests with 68 skips; the separate UI contract
suite passed 31 tests. Actual Chromium passed 40 guided workflow checks, including
snapshot storage, millimeter conversion, exact-frame interruption, contact-sheet
download, changed-board isolation, import gates and retained native 3D boards.
A separate retained-board browser check passed 14 orbit/zoom/visibility and review
checks without capture or fitting. Missing native/GPU/hardware capabilities are
skips in the ordinary suite; the opt-in native test was separately run without a
skip. All physical checklist items below remain unrun.

## Physical acceptance checklist

- Verify the physical camera/lens identity, exact delivered resolution, crop and
  binning, fixed focus or locked manual focus, and original recording history.
- Independently measure board spacing, flatness, printer scale and corner ordering;
  recapture diverse edge/corner, near/far and tilted views with the same camera mode.
- Investigate held-out residuals, uncertainty, model disagreement and systematic
  error. Independently validate aiming signs and geometry; survey the robot mount
  separately from board-to-camera poses.
- Measure exposure/capture correction and clock mapping with uncertainty. Check
  unavailable offsets, disconnects and clock changes; host read time is a fallback.
- Validate motion, blur, shutter behavior, exposure and lighting on real imagery.
- Coordinate camera ownership with the production runtime and other services;
  test cancellation, reader release, reconnect, source loss and power loss.
- Measure worst-case CPU, memory, USB/network and preview load on the target
  hardware, including receipt expiry and recovery. Software synthetic checks
  establish none of these physical results.
