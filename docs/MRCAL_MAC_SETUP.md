# Native solver assessment on this Mac

Read-only assessment, 2026-10-08. No dependency installation, native build,
solver job, VM/container launch, camera access or AGI scheduling change was made.

The inspected host is macOS 26.6.2, Apple Silicon arm64, with CPython 3.12.14.
Its working desktop environment has OpenCV 4.10.0, NumPy 1.26.4, PyYAML 6.0.3
and pytest 8.4.2. Missing importable modules are `mrcal`, `scipy`, `numpysane`,
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

## Minimal next step, not executed

Create a new isolated solver environment inside ignored `data/`, using the
existing interpreter. Require published binaries for compiled dependencies;
only the small pure-Python numpysane and gnuplotlib packages may use source
distributions. If a compiled wheel is unavailable or dependency resolution fails,
stop and inspect the error instead of launching a source build.

```sh
.venv/bin/python -m venv data/native-mrcal-mac-env
data/native-mrcal-mac-env/bin/python -m pip install \
  --only-binary=:all: --no-binary=numpysane,gnuplotlib \
  'mrcal==2.5.2.post1' 'pyfltk==1.4.5.0' 'pytest==8.4.2'
PATH="$PWD/data/native-mrcal-mac-env/bin:$PATH" \
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 \
data/native-mrcal-mac-env/bin/python -m custom_vision.calibration_session doctor \
  --mrcal-python "$PWD/data/native-mrcal-mac-env/bin/python"
data/native-mrcal-mac-env/bin/python -m pip freeze \
  > data/native-mrcal-mac-env/installed-versions.txt
```

This leaves the existing `.venv`, Jetson camera/GPU stack and system Python
unchanged. The doctor check imports modules and finds the real CLI; it does not
solve or acquire images. Run the one native test in a separately approved CPU
window, after the doctor succeeds:

```sh
PATH="$PWD/data/native-mrcal-mac-env/bin:$PATH" \
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 \
data/native-mrcal-mac-env/bin/python -m pytest -q -rs \
  tests/test_guided_calibration.py::test_real_mrcal_full_solve_validation_uncertainty_and_runtime_export
```

This test uses 60 synthetic views and exercises actual OPENCV8/spline fits,
held-out residuals, sampling uncertainty, model comparison, retained optimization
inputs and exact runtime export. It currently remains unrun here. Passing would
establish software integration with synthetic data; independent physical camera,
board, mount, timing and motion validation would still be required.

For later desktop launches, put the isolated solver's `bin` directory on `PATH`
and pass its interpreter to `tools/calibration_dashboard.py --solver-python`.
The interpreter option alone does not expose adjacent CLI programs: current CLI
discovery and workers inherit `PATH`.

To test the mrgingham path, the upstream-preferred alternative is an isolated
Debian 12+ or Ubuntu 22.04+ arm64 environment with matching distro mrcal and
mrgingham packages. Do not install those packages on the Mac or replace the
Jetson's active environment. No such container or VM was launched for this assessment.
