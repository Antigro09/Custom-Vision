# Browser field visualization

The dashboard includes **2D plan** and **3D perspective** views of the selected
pipeline's available vision localization. Choose **All camera estimates** to draw
separate labeled sources. The inspector continues to describe the selected source;
no source is fused with another or with drivetrain odometry.

Green robot and amber camera markers use `localization.field_to_robot` and
`localization.field_to_camera` independently. Missing mounting geometry leaves the
robot unavailable while a valid camera pose can still be drawn. Configured mounting
translation appears in the inspector; the connector between accepted camera and
robot poses illustrates their actual transform. Markers, axis lengths and tag icons
are display aids, not measured robot footprints, tag sizes or camera frustums.
The existing camera preview and independent POI controls remain available.

## Frames, freshness and limits

All geometry stays in the fixed WPILib NWU frame: X forward, Y left, Z up, meters,
WXYZ quaternions. Named field poses have already been converted from optical
coordinates; the renderer does not apply that conversion again. Canvas projection
uses Z-up directly. In plan view X goes right and Y goes up; perspective preserves
actual Z and quaternion axes. No alliance flip, origin reset or season default is
introduced. Field dimensions and configured tag poses come from the active validated
WPILib layout, never from image appearance. If absent, only axes and a metric grid
are shown, with **Field geometry unavailable**.

`GET /api/field-view` is a read-only snapshot of unchanged dashboard result packets,
server-monotonic receipt ages, validated layout and nullable configured mounts. It
adds no fields to `/api/status` or the producer wire contract. Public geometry is
cached at runtime setup; paths and credentials are not included in that geometry.
The browser projects each server receipt age into its own monotonic clock rather
than comparing clocks from different domains. Independent measurement expiry is
1.5 seconds, and repeated polls do not refresh it. Higher publication sequence can
invalidate the same capture frame; it cannot create a second measurement or extend
its lifetime. Dashboard data with disabled NT/stdout also works without `packet_seq`.

Invalidation clears current poses. Expired estimates become labeled, subdued stale
markers; historical trails are never current observations. Trails break across
invalid intervals, stale gaps and source/configuration epochs. The model caps eight
sources and 180 history points per pose; the renderer draws at most 160 per trail,
512 tag markers, 15 frames per second and eight million canvas pixels. Image
textures use a bounded triangle mesh. Hidden tabs back off, and HTTP polling has
one request outstanding. No WebGL, CDN, model download or GPU is required.

## Field picture and metric geometry

Open **Field image, reviewed geometry & replay**, import an A* field-map JSON and
choose its matching local PNG/JPEG/WebP. Imports stay in this browser session and
do not write camera configuration, restart workers or change localization revisions.
The image's filename, dimensions and SHA-256 must match. Pictures are limited to
12 MiB and 16 megapixels; JSON is limited to 1 MiB. No URL, SVG or embedded code is
loaded from the document.

The shared document is `schema_version: "frc-field-map/1"`, owned by the A* image
importer. `map.width_m` is the fixed-frame X extent and `map.height_m` the Y extent,
adapted to WPILib `field.length` and `field.width`. Its row-major affine 3×3
`calibration.image_to_field` maps pixel coordinates (right/down) to field XY meters.
Perspective/homography import and lens-distortion correction are unsupported here;
the recorded distortion state and metric check errors remain explicit. A photograph
alone does not establish its transform, physical obstacle shape or height.

Boundary/obstacle outer rings are explicitly closed counterclockwise, holes closed
clockwise. Validation checks finite bounds, winding, intersections, hole containment,
transform invertibility and review revisions. Canonical metric rings describe
physical geometry; display applies no planner footprint or clearance inflation.
Approval requires exact map revision and canonical content digest plus reviewed
polygons and independent metric checks distinct from fitted controls. Both declared
errors are RMS Euclidean affine-projected residuals in meters over their respective
point sets; the viewer recomputes them within 1e-8 m. Image/geometry edits make old approval
stale. Artwork/map revision stays separate from tag-layout/calibration revisions.
Dimension mismatch with the runtime tag layout suppresses the imported overlay;
matching dimensions alone do not prove origin, season/variant or physical alignment.
Check these against surveyed geometry before using a map.

Draft/suggested polygons are visibly uncertain. A null `vertical_range_m` stays an
outline in 3D; only an approved explicit vertical interval can become a solid.
Holes are retained. Suggested image contours cannot establish vertical geometry.
The A* importer supplies reviewed contour drafts and vertex editing; this viewer
consumes those exports and does not claim automatic obstacle extraction.

The mirrored [synthetic fixture](../tests/fixtures/field-map/synthetic-approved.json)
and [picture](../tests/fixtures/field-map/synthetic-top-down.png) are byte-identical
to A*'s owned test assets. They are an 8×4 m diagram, not an official FRC field. Its
CC0 attribution, approval digest and image hash are preserved. Importing them into
a differently sized runtime layout deliberately reports a mismatch.

## Replay and local checks

**Load replay JSON** accepts one producer packet, an ordered array, or
`{packets:[...],field_layout:<WPILib layout>,mounts:<pipeline map>}`. Optional geometry
is replay context, not a runtime write. Replay is labeled and isolated from live
sources. **Next packet** steps manually; old packets expire. **Return to live**
restores the selected live pipeline. Maximum 512 packets / 2 MiB. Malformed or
retired-source packets are rejected. Clear trails affects display history only.

Run a camera-free, read-only localhost preview (no camera, detector or NT instance):

```sh
.venv/bin/python tools/field_dashboard_preview.py --port 5844
node --test tests/dashboard_client.test.cjs tests/field_model.test.cjs tests/field_renderer.test.cjs tests/field_scene.test.cjs
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest -q -rs
```

Use the separate browser-test requirements and harness for actual headless browser
QA; its command-line help describes local browser executable and artifact options.
The preview explicitly replays synthetic fixture poses with synthetic identities.
It does not qualify calibration, physical capture, Jetson worst-case load, controller
hardware or robot integration.

## Research basis

Limelight documents 3D camera/robot target-space and robot field-space transform
visualizers and mounting-pose adjustments in its [3D AprilTags guide](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d).
Its [MegaTag2 guide](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2)
shows separate estimation methods and recommends a fixed blue origin. Its
[software changelog](https://docs.limelightvision.io/docs/docs-limelight/software-change-log)
documents PNG map-builder upload. These references establish useful visualization
concepts; they do not establish automatic obstacle extraction or a separate built-in
2D Limelight field-map mode. Our 2D view is complementary, and our solver/protocol
remain Custom-Vision's existing implementation.
