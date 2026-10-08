# Native solver setup on this Mac

The separate workspace-local CPython 3.12 environment is now prepared at
`data/native-mrcal-mac-env`. Native imports, the real doctor entry point, CLI help
and dependency consistency checks pass. The bounded 60-view native integration
test now passes; its calibration candidate remains `needs_review`. The working
`.venv` package manifest is unchanged, and both existing previews remain running.
No VM/container launch, camera access or AGI scheduling change was made.
See [native results and evidence](MRCAL_MAC_NATIVE_CHECK.md).

The inspected host is macOS 26.6.2, Apple Silicon arm64, with CPython 3.12.14.
Its working desktop environment has OpenCV 4.10.0, NumPy 1.26.4, PyYAML 6.0.3
and pytest 8.4.2. In that working environment, missing importable modules remain `mrcal`, `scipy`, `numpysane`,
`gnuplotlib`, `shapely` and `fltk` (provided by `pyfltk`). Missing commands are
`mrcal-calibrate-cameras`, `mrgingham` and `gnuplot`.

This is an installation gap. Upstream documents macOS arm64 wheels and lists
`mrcal-2.5.2.post1-cp312-cp312-macosx_14_0_arm64.whl`, matching this host and
interpreter. Its SHA-256 is
`1f36ab4bec94a4f0cfd4b102df497a919846b71c5fb728f5e945682c0f511247`.
The maintainer describes bundled native libraries/CLI, gnuplot and vnlog, but
cautions that the wheel may have omissions and recommends Debian/Ubuntu packages
when possible. Availability of a compatible wheel does not establish that our
integration test passes. See the [official installation guidance](https://mrcal.secretsauce.net/install.html)
and [pinned release files](https://pypi.org/project/mrcal/2.5.2.post1/).

The pinned package declares NumPy, numpysane >=0.35, SciPy >=0.18,
opencv-python-headless, gnuplotlib >=0.38, Shapely, PyYAML and pyfltk. The
published [package metadata](https://pypi.org/pypi/mrcal/2.5.2.post1/json)
is authoritative for this version. pyfltk also has a
[CPython 3.12 macOS arm64 wheel](https://pypi.org/project/pyfltk/1.4.5.0/).
The existing solver test does not need FLTK interaction or a displayed window.
The Mac mrcal wheel excludes mrgingham. Our native integration test explicitly
uses `opencv-sb`, so mrgingham is unnecessary for that test.

## Exact isolated lock and completed checks

[requirements-mrcal-macos-cp312.lock](../tools/requirements-mrcal-macos-cp312.lock)
pins all 15 solver/test/inspection dependencies and their exact official registry
artifact hashes. It keeps NumPy 1.26.4 and OpenCV 4.10.0.84 aligned with the tested
desktop algorithms, with SciPy 1.16.3 and mrcal 2.5.2.post1. All 13 binary artifacts
have wheel tags supported by the inspected CPython 3.12/macOS arm64 host. Only
numpysane 0.45 and gnuplotlib 0.47 use small pure-Python source distributions.
No compiled source dependency was built.

The pure-Python packaging backend actually used was setuptools 84.0.0, pinned in
[requirements-mrcal-python-build.lock](../tools/requirements-mrcal-python-build.lock).
The new environment's bootstrap pip is 25.0.1. The complete installed inventory,
install/resolution reports, artifact manifest and logs are retained under ignored
`data/native-mrcal-setup/`. The installed environment occupies approximately
454 MiB, before caches; this is a measured disk figure, not a memory requirement.

Ten native/analysis module imports passed, including SciPy optimization/linear
algebra and the actual mrcal extension. The doctor located the isolated CLI and
imported mrcal, OpenCV and SciPy. mrcal does not expose `__version__`; its doctor
fallback prints `distro-package`, but distribution metadata verifies the installed
wheel version is exactly 2.5.2.post1. `pip check` reports no broken requirements.
The real CLI help accepts all nine flags used by the current adapter, and
`pytest --collect-only` collected exactly the requested native integration test.
Collection generated no views and performed no calibration.
mrgingham remains absent and is unnecessary for the existing `opencv-sb` test.

## Reproduction in a fresh checkout

Create a new isolated solver environment inside ignored `data/`, using the
existing interpreter. Require published binaries for compiled dependencies;
only the small pure-Python numpysane and gnuplotlib packages may use source
distributions. If a compiled wheel is unavailable or dependency resolution fails,
stop and inspect the error instead of launching a source build.

```sh
.venv/bin/python -m venv data/native-mrcal-mac-env
data/native-mrcal-mac-env/bin/python -m pip --isolated install --require-hashes \
  -r tools/requirements-mrcal-python-build.lock
data/native-mrcal-mac-env/bin/python -m pip --isolated install --require-hashes \
  --no-build-isolation -r tools/requirements-mrcal-macos-cp312.lock
PATH="$PWD/data/native-mrcal-mac-env/bin:$PATH" \
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 \
data/native-mrcal-mac-env/bin/python calibration.py doctor \
  --mrcal-python "$PWD/data/native-mrcal-mac-env/bin/python"
data/native-mrcal-mac-env/bin/python -m pip --isolated freeze --all \
  > data/native-mrcal-mac-env/installed-versions.txt
```

This leaves the existing `.venv`, Jetson camera/GPU stack and system Python
unchanged. Use a workspace-local pip cache when reproducing; no cache or package
needs to be written outside this checkout. The doctor check imports modules and
finds the real CLI; it does not solve or acquire images. `calibration.py` is the
entry point; invoking the module alone does not call its `main()`.

## Approved bounds and completed native test

The approved slot requested one compute thread, a 30-minute exclusive slot,
2 GiB available RAM and 256 MiB artifact space, excluding the environment/cache.
Memory and runtime were conservative scheduling estimates. The supervisor samples
aggregate owned-group RSS and artifacts and stops if either exceeds the estimate;
this is not a kernel-enforced memory cap or an exact RSS peak measurement.
The test has 60 synthetic 640x480 grayscale views, 35 corners/view, four sequential
native fits with 300-second per-fit deadlines, then held-out pose fitting and
24x16 diagnostic grids. The four fits can therefore consume up to 20 minutes;
the other stages currently have no total deadline of their own.

The [supervisor](../tools/native_calibration_check.py) applies a total
1,800-second wall deadline. Timeout, cancellation, unexpected resources or
monitoring errors terminate the owned process group with SIGTERM, followed by
SIGKILL after a five-second grace if needed, and reap the direct child. It also
cleans up descendants after an early child exit. Short sleeping-process checks
verified deadline, early-exit, cancellation and monitoring-error cleanup. It
signals no preview or unrelated group. A passing result requires exactly one
unskipped JUnit test, a completed model report, four solver logs and empty group.

```sh
.venv/bin/python tools/native_calibration_check.py --self-test
.venv/bin/python tools/native_calibration_check.py
```

The Mac OpenCV wheel uses GCD. Its `getNumThreads()` returns the pool/CPU count
after a positive setting, so a naive `setNumThreads(1)` assertion is misleading.
`setNumThreads(0)` disables parallel regions and was checked to report one here;
OpenCL was also checked disabled. The loaded NumPy OpenBLAS and mrcal OpenMP pools
each report one thread under the listed environment. SciPy's Apple Accelerate
thread request uses `VECLIB_MAXIMUM_THREADS=1`; its peak under a solve has not been
separately measured. This requests serial computation, not a macOS CPU-affinity guarantee.
See the pinned [OpenCV 4.10 parallel implementation](https://github.com/opencv/opencv/blob/4.10.0/modules/core/src/parallel.cpp).

This test uses 60 synthetic views and exercises actual OPENCV8/spline fits,
held-out residuals, sampling uncertainty, model comparison, retained optimization
inputs and exact runtime export. It passed with four real fits on this Mac;
held-out RMS was 0.120281 px for OPENCV8 and 0.134952 px for spline. The candidate
stays under review because spline sampling uncertainty and model disagreement
exceed their review thresholds. These are synthetic software results; independent
physical camera, board, mount, timing and motion validation remains required.

For later desktop launches, put the isolated solver's `bin` directory on `PATH`
and pass its interpreter to `tools/calibration_dashboard.py --solver-python`.
The interpreter option alone does not expose adjacent CLI programs: current CLI
discovery and workers inherit `PATH`.

To test the mrgingham path, the upstream-preferred alternative is an isolated
Debian 12+ or Ubuntu 22.04+ arm64 environment with matching distro mrcal and
mrgingham packages. Do not install those packages on the Mac or replace the
Jetson's active environment. No such container or VM was launched for this assessment.
