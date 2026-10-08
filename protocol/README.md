# Custom Vision producer protocol

This directory owns the raw observation contract and producer fixtures. Persistent
field tracking, robot pose estimation, planning, and motion belong to consumers.
The protocol is independent of game year and controller. Python payload timestamps
remain integer microseconds even when a consumer's NT library uses nanoseconds.

* [schema2.json](schema2.json) describes the deployed schema-2 JSON string. Existing
  fields retain their meanings; optional diagnostics may be absent on failures.
* [custom-vision-schema2-2026.1.json](custom-vision-schema2-2026.1.json) is the additive
  named profile. It keeps `schema_version: 2` and requires `protocol_profile`,
  `packet_seq`, and three nullable public geometry revisions. `timing` is optional
  for consumers, and emitted by this producer.
* [FIELDS.md](FIELDS.md) defines units, validity, compact references, revisions,
  and acceptance ordering that cannot all be expressed in JSON Schema.
* [fixture-manifest.json](fixture-manifest.json) lists producer revision, actual
  source hashes, exact fixture byte hashes, typed topic values and expected
  outcomes. [fixtures/](fixtures/) contains exact coherent publisher strings.
* [PHYSICAL_CHECKLIST.md](PHYSICAL_CHECKLIST.md) separates required physical
  measurements from these camera-free software checks.

The JSON Schemas use Draft 2020-12 and stable URN identifiers. Resolve
`urn:custom-vision:schema2` from the local `schema2.json`; no network resolution is
necessary. Unknown additional fields are allowed so diagnostic additions do not
break a conforming reader. A reader must enforce the semantic gates in FIELDS.md,
including resolving selected IDs and checking quaternion/covariance quality.

Generate the golden corpus with:

```sh
.venv/bin/python tools/generate_protocol_fixtures.py --contract-status matched_runtime
.venv/bin/python -m pytest tests/test_protocol.py
```

The generator runs CPU geometry on small synthetic observations, injects an
explicit microsecond clock, and sends results through the implemented
`Publisher.publish -> wire_packet -> json.dumps` path with stub NT topics. It does
not infer that physical calibration or capture correction is verified. A measured
zero-correction fixture is a synthetic metadata case, not a measured camera claim.
The manifest must say `matched_runtime` before fixtures are handed to consumers as
the agreed profile. Provisional generation uses `pending_runtime_alignment`.

The first four manifest entries are a coherent replay: one valid frame, same-frame
watchdog invalidation, repeated invalidation, and a new boot. The remaining entries
are independent contract examples. Their counters still come from the publisher.
Malformed consumer input belongs in [consumer-invalid/](consumer-invalid/) and
must never be labelled a valid producer fixture.

Migration from deployed schema 2 is additive: readers that ignore unknown fields
can keep parsing existing geometry. Readers should adopt `packet_seq` for
publication order; `frame_id` remains capture/processing identity. Do not dedupe
invalidation by frame ID. The profile gives no new 3D capability, motion
compensation, synchronized exposure measurement, or physical qualification.

The NT4 producer dependency remains `pyntcore>=2023.4,<2025`. Reproducible desktop
checks pin the supported adapter to 2024.3.2.1 without replacing Jetson libraries.
WPILib 2026 + roboRIO is the primary current consumer target. A separately pinned
WPILib 2027 alpha + Systemcore target requires its own API time adapter. An
official WPILib 2026 + Systemcore toolchain must be verified separately; the
offseason's game year does not establish support.
