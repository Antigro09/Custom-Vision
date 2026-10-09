# Shared synthetic field-map fixture

A* owns `frc-field-map/1`. These two assets are mirrored byte-for-byte from its
`fixtures/field-map/synthetic-approved.json` and `synthetic-top-down.png` for an
independent Custom-Vision consumer check; neither is an official FRC field.
Image attribution/license are recorded in JSON (`CC0-1.0`).

- Image SHA-256: `9f8ad23da16dcf864327ca31558ad9e1cae8864a8355e9c2f9feba48a828ebeb`
- Canonical approval SHA-256: `e022b3c0b20e83f71f8a6519946d635bc2e730c38d20f65392dae404d76f0163`

Canonical approval excludes only the root `approval` object, sorts object keys,
preserves arrays, encodes every numeric value as a JSON string `f64:` followed by
16 lowercase big-endian IEEE-754 hex digits, and normalizes negative zero to zero.
Strings use UTF-8 JSON escaping. Approval is not proof of physical verification.

The affine `[.02,0,0,0,-.02,4,0,0,1]` maps pixel-right/down into fixed NWU XY meters.
X extent is 8 m, Y extent 4 m. Outer rings are closed CCW, holes closed CW, and the
obstacle's vertical range is unknown (`null`). The renderer applies no inflation.
