# Producer contract implementation report

This change extends the raw Custom-Vision producer contract. Robot-code integration,
controller deployment and hardware qualification are outside its implementation.
The vision pipelines and Jetson runtime dependency range are preserved. The
recorded desktop checks used an isolated Mac test environment.

| Item | Result |
| --- | --- |
| Repository | Existing `Antigro09/Custom-Vision`, downloaded to an empty task workspace |
| Inspected HEAD / audit baseline | `2aee0fc1794b31539b02000d16791d4eec8df29a`; identical and initially clean |
| Local branch | `vision/schema2-producer-contract` |
| Core producer commit | `425e5b43b594f65ce6db4038d8c3bf3c5039a6fd` |
| Profile | `custom-vision-schema2-2026.1`, additive to schema 2 |
| Protocol corpus | 27 producer fixtures, matched runtime; 6 malformed consumer examples separately |

The implementation adds publication order under the publisher lock, nullable public
geometry revisions, independent correction verification/uncertainty, a fail-closed
version-aware NT clock adapter, complete actionable error clearing and suppression
of results from captures already invalidated. Reload/shutdown retain the original
capture identity. Packet sequences start at zero per pipeline/boot, include repeated
same-frame invalidations, preserve frame identity, and fail rather than wrap above
2^53-1. Compact references and typed object selection resolve canonical targets.

The deployed schema, additive profile, units/validity/revision rules, source hashes,
exact fixture-byte hashes and expected typed outputs are in [protocol/](../protocol/README.md).
Fixtures run actual CPU localization, independent POI, calibrated object geometry
and bounded segmentation decoding before the real publisher serialization path.
Their clocks/transport are explicit stubs; [native loopback evidence](NT4_INTEROP.md)
separately checks real NT4. Float serialization preserves the baseline six-decimal
precision bound and dashboard containers stay full precision. Review caught and
fixed schema parity for valid `segmentation:null` outputs.

| Final checks | Result |
| --- | --- |
| Full pytest CPU suite, including approved localhost dashboard/NT4 checks | 565 passed, 67 skipped, 0 failures; 13.61 s |
| Protocol checks (included in full suite) | 48 passed; exact corpus reproduction and semantic gates |
| Clock adapter checks (included in full suite) | 74 passed; installed private `_now` binding confirms integer microseconds |
| Dashboard JavaScript checks | 15 passed, 0 failures |
| Real Python2024 producer ↔ released alpha-7 Java/native NT4 | 7 checks each on single-tag, empty-tag and compact-object fixtures; passed |
| File-only synthetic smoke | Passed: 2 shared-input pipelines, 3 empty frames each, same-frame shutdown; 8 packets |
| Manifest/source/27 fixture hash verification | Passed |
| Python compile and `git diff --check` | Passed |

Skipped checks are unavailable on this Mac: native AprilTag extension (including
CUDA pose/MultiTag execution), TensorRT, mrgingham and mrcal. Physical capture,
calibration, mount accuracy, source/power loss, motion, controller clock alignment,
Jetson worst-case load and actual roboRIO/Systemcore hardware remain unrun. Use the
[separate physical checklist](PHYSICAL_VALIDATION.md) under later hardware approval.
No hardware performance/readiness claim follows from software passes.

WPILib 2026 + roboRIO is the immediate consumer target; this producer remains
controller-independent. The separate desktop NT server pin is
`org.wpilib.ntcore:2027.0.0-alpha-7` with Java 25. The official
[SystemcoreTesting compatibility matrix](https://github.com/wpilibsuite/SystemcoreTesting#software-compatibility)
checked on 2026-10-08 lists only 2027 alpha toolchains. WPILib 2026 + Systemcore has
no official supported target in that matrix; no HAL/toolchain compatibility was
invented. Game year does not select the controller toolchain.

JSON `_us` remains integer microseconds across both consumer versions. The pinned
Python2024 `_now`/offset uses microseconds; alpha-7 Java sample metadata uses
nanoseconds. Real loopback verified the distinct epochs/units (including a negative
server offset), whole queued strings, disconnect/reconnect and retained old data.
This proves desktop transport interoperability with the selected server, not
Systemcore hardware qualification or exposure-time synchronization.

Reproduce from this checkout using an isolated Python 3.12 environment (the original
producer supports Python>=3.10; the isolated Java bootstrap uses Python3.12 safe
archive extraction). Do not apply the desktop test lock to the installed Jetson:

```sh
UV_CACHE_DIR="$PWD/.cache/uv" uv venv --python 3.12 .venv
UV_CACHE_DIR="$PWD/.cache/uv" uv pip install --python .venv/bin/python -r tools/requirements-protocol-test.txt
.venv/bin/python tools/generate_protocol_fixtures.py --contract-status matched_runtime
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest -q -rs
node --test tests/dashboard_client.test.cjs
.venv/bin/python tools/protocol_smoke.py
.venv/bin/python tools/nt4_interop/bootstrap.py --download --download-jdk
.venv/bin/python tools/nt4_interop/run.py --fixture protocol/fixtures/single_tag.json
```

Loopback tests require localhost TCP socket access. NT4 downloads are optional,
pinned and checksum-verified; JNI/JDK artifacts remain ignored in
`tools/nt4_interop/.cache`. Re-running fixture generation records the checkout's
current revision; fixture bytes and source hashes identify the exact corpus.

Consumers may continue parsing deployed schema-2 geometry while accepting additive
fields. To use this profile, adopt packet-sequence eligibility and apply accepted
invalidation before measurement frame deduplication. Clear missing/invalid families,
expire each source independently, reject retained/retired-session data and resolve
compact object IDs. Revisions identify configuration, not physical verification.
Configuration reload invalidates pending observations and creates a new source boot.
See [networktables.md](networktables.md) and [FIELDS.md](../protocol/FIELDS.md).
