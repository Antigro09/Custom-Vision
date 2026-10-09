# Pinned, camera-free NT4 interoperability

The primary immediate target is **WPILib 2026 + roboRIO**. The separately pinned
Systemcore profile is **WPILib 2027.0.0-alpha-7**. Game year does not select the
controller toolchain. This harness tests the Python producer against the released
alpha-7 desktop Java NT server; it is neither robot-code integration nor a
Systemcore hardware test.

As checked on 2026-10-08, the official [SystemcoreTesting compatibility
matrix](https://github.com/wpilibsuite/SystemcoreTesting#software-compatibility)
lists 2027 alpha-1/2, alpha-5/6 and alpha-7 toolchains. Systemcore image >=14 is
paired with >=2027.0.0-alpha-7. **WPILib 2026 + Systemcore has no official supported
target in that matrix.** No 2026 Systemcore adapter, rewritten HAL, compatibility
claim or deployment is supplied. The 2026 consumer profile and its version pin
belong to the separate Java consumer component.

## Versions and artifact provenance

| Component | Harness selection |
| --- | --- |
| Python producer | `pyntcore==2024.3.2.1`, within existing `>=2023.4,<2025` range |
| Java NetworkTables | `org.wpilib.ntcore:ntcore-java:2027.0.0-alpha-7` |
| Supporting Java/native libraries | `org.wpilib` ntcore, wpiutil, wpinet, datalog, all `2027.0.0-alpha-7` |
| Native platform | Released `osxuniversal` C++ archives, including JNI shared libraries |
| Compiler/JVM | Java 25; optional isolated Temurin `25.0.4.1+1` macOS ARM64 download |
| Build | Direct `javac --release 25` / `java`; no Gradle, GradleRIO, WPILibJ or HAL dependency |

[artifacts.lock.json](../tools/nt4_interop/artifacts.lock.json) records exact
HTTPS URLs, byte counts and SHA256 hashes. The WPILib artifacts were checked
against their official Maven `.sha256` files. The optional JDK hash came from
[Adoptium's release metadata](https://api.adoptium.net/v3/assets/feature_releases/25/ga?architecture=aarch64&heap_size=normal&image_type=jdk&jvm_impl=hotspot&os=mac&vendor=eclipse&page_size=1)
and is pinned to the specific release URL in the lock, rather than resolving
"latest" during subsequent runs. [PyPI publishes the exact producer
release](https://pypi.org/project/pyntcore/2024.3.2.1/). Downloaded binaries,
runtime, compiled classes and transient persistence file stay under ignored
`tools/nt4_interop/.cache/`. No installed Jetson dependencies are changed.

Alpha-7 [upstream build settings](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/build.gradle)
use Java release 25. The harness uses the released
`PubSubOption.SEND_ALL` / `KEEP_DUPLICATES` constants, and the alpha-7 single-port
`startServer(persistence, listenAddress, mdnsService, port)` API. It does not reuse
2026 `edu.wpi.first` imports or older option factory assumptions.

## Timestamp boundary

The pinned Python native `_now()` and `getServerTimeOffset()` return integer
**microseconds**. The actual producer's version-aware `NtClockAdapter` owns
conversion from host read-completion time into synchronized NT server time.
JSON `capture_server_us`, `capture_monotonic_us` and `publish_unix_us` retain
their microsecond meaning. The Unix value remains logging metadata.

Alpha-7 [`TimestampedString.timestamp` and `serverTime`](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/ntcore/src/generated/main/java/org/wpilib/networktables/TimestampedString.java)
and [`NetworkTableInstance.getServerTimeOffset()`](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/ntcore/src/generated/main/java/org/wpilib/networktables/NetworkTableInstance.java)
use **nanoseconds**. The Java test emits raw metadata; Python compares JSON
microseconds multiplied by 1000 against that metadata. `serverTime` may be 0/1
for a server-local value, so the harness does not assume every nonempty value
has a positive server timestamp. The measured loopback values had positive
remote server times with 1000 ns granularity.

The native Python and Java clocks can have different epochs. The observed Python
offset was a large negative integer; it is applied before comparing clocks.
Two-second assertion windows only catch gross unit/clock errors in this bounded
smoke test. They are not latency budgets, clock-accuracy measurements, exposure
measurements, capture-correction verification or worst-case-load results.

## Reproduce on a Mac

Use an isolated Python 3.12+ environment. The checked run used `.venv`, Python
3.12.14 and the pinned producer. Install only into this environment:

```sh
python3.12 -m venv .venv-interop
.venv-interop/bin/python -m pip install -r tools/nt4_interop/requirements.txt
.venv-interop/bin/python tools/nt4_interop/bootstrap.py --download --download-jdk
.venv-interop/bin/python tools/nt4_interop/run.py \
  --fixture protocol/fixtures/single_tag.json \
  --report tools/nt4_interop/.cache/single_tag-result.json
```

Alternatively set `JAVA_HOME` to an existing Java 25 JDK and omit
`--download-jdk`. Once downloads are present, `bootstrap.py` verifies their
hashes and builds offline with no download flags. The optional bundled JDK is
macOS ARM64 only; the native artifact lock is macOS universal. Linux/Windows
native runs are **unrun**, rather than silently selecting other artifacts.

The test uses an ephemeral unprivileged port and binds the Java server only to
`127.0.0.1`, with mDNS advertising disabled. It creates a pinned Python NT
instance with the same explicit loopback address/port before starting the client
and attaches it to the actual `Publisher`. All serialization and clock logic
execute through `Publisher.publish()` / `wire_packet()`. It deep-copies a golden
producer fixture, refreshes capture/log times and frame identity, and adds two
harness-only probes: an exact safe-range integer and a float rounded to six
decimal places. These are synthetic additive fields, not new protocol contracts.

The JVM has a 96 MiB heap limit and a 45-second watchdog. Polls and subprocess
operations have separate short bounds. Processes, subscriptions and clients are
closed after the run. No persistent service, camera, controller, GPU workload
or motion command is involved. Running this test requires permission to bind
localhost TCP sockets in the execution environment.

## Results and remaining checks

Three native loopback runs passed with canonical producer fixtures:

| Fixture | Saved receipt | Result |
| --- | --- | --- |
| Single tag | [single_tag-macos.json](../tools/nt4_interop/results/single_tag-macos.json) | Passed |
| Empty tags | [empty_tags-macos.json](../tools/nt4_interop/results/empty_tags-macos.json) | Passed |
| Compact objects, covariance and selection | [objects_compact-macos.json](../tools/nt4_interop/results/objects_compact-macos.json) | Passed |

Each receipt records fixture, producer-file and artifact-lock hashes, profile
`custom-vision-schema2-2026.1`, and producer revision
`425e5b43b594f65ce6db4038d8c3bf3c5039a6fd`. The producer commit mapping was added
after the runs by matching the recorded producer hashes to the committed files;
the transport runs were not repeated to add this provenance. Each run
checks connection; real native clock-offset units; exact complete serialized
strings and integer timestamps; queued whole publications; loss of synchronized
capture time on disconnect; clock reacquisition after reconnect; and delivery of
the retained final packet to a late subscriber after producer closure.

The server explicitly enables retention for this test. Production `Publisher`
does not automatically enable it. A retained value preserves its old
`packet_seq` and capture time: delivery to a new subscriber does not prove source
liveness, freshness or a new measurement. Consumers still require independent
receipt expiry and publication/session eligibility.

No Systemcore or roboRIO hardware, real Jetson networking, physical camera,
2026 Java native server, other versions in the Python dependency range, Linux or
Windows native environment, worst-case load, exposure/correction accuracy, power
loss, calibration or motion was tested here. Use the separate
[physical checklist](../protocol/PHYSICAL_CHECKLIST.md) before any hardware
qualification. Actual robot-code integration remains on hold.
