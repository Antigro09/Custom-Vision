# Object detection for team 1086

The current OV9281-class global-shutter Arducams are monochrome. The default
`contour` backend finds bright, approximately round **ball candidates**. It is a
starting point for camera, targeting, and robot integration, and has no trained
knowledge of a game piece. A white mark, light, or another round object can also
be a candidate. The game and final object classes are not yet known.

## Monochrome candidate baseline

The object pipeline's `settings` mapping in `config/local.yaml` supports:

```json
{
  "backend": "contour",
  "label": "ball_candidate",
  "threshold": 180,
  "polarity": "bright",
  "min_area_px": 150,
  "max_area_fraction": 0.25,
  "min_circularity": 0.55,
  "morphology_kernel": 3,
  "max_detections": 100
}
```

Thresholding uses grayscale intensity 0–255. `polarity: "dark"` selects dark
objects instead. Opening and closing remove isolated pixels and fill small
gaps; set `morphology_kernel` to zero to disable them. Positive kernel sizes
must be odd. Contours below `min_area_px`, above `max_area_fraction` of the
image, or below `min_circularity` are rejected. Circularity is `4*pi*area /
perimeter^2`; elongated marks score poorly. Small wiffle-ball holes are ignored
when measuring the outer contour. Partly hidden or touching balls may not form
separate round contours.

The published `confidence` is the circularity heuristic, accompanied by
`confidence_kind: "shape_circularity"`. It is not a probability that the object
is a ball. Candidates are ordered by bounding-box area, largest first. Tune
exposure and thresholds using the real field surface and lighting before
using this output to guide an intake.

## Optional color baseline

When the planned color camera arrives, `hsv` can select a colored game piece:

```json
{
  "backend": "hsv",
  "label": "game_piece_color",
  "hsv_lower": [5, 100, 80],
  "hsv_upper": [30, 255, 255],
  "min_area_px": 150,
  "morphology_kernel": 3
}
```

These example values select orange tones; they are not tuned for an actual
game piece. OpenCV hue uses 0–179 and saturation/value use 0–255. Hue ranges
wrap when lower hue exceeds upper hue, e.g. `[170,100,80]` to `[10,255,255]`
selects red. The HSV baseline has no useful color signal on a monochrome
camera. Its `confidence` is foreground pixel occupancy inside the bounding
box (`confidence_kind: "color_fill_fraction"`), also a heuristic.

## Trained model contract

`tensorrt` runs trained detectors directly through TensorRT 10 and JetPack's
CUDA runtime. It does not require PyTorch for inference. `opencv_onnx` is a
CPU fallback for inspecting the same ONNX model; it is not the recommended
performance backend for the Jetson.

Supported models have exactly one static input `[1,3,H,W]` and one raw
YOLOv8/YOLO11 detection output `[1,4+number_of_classes,N]`. Output coordinates
are pixel-space center-x, center-y, width, height followed by per-class
probabilities. Labels must exactly match training class order. The runtime
performs confidence filtering and class-aware NMS. YOLOv5, embedded-NMS,
end-to-end, segmentation, pose, dynamic, and batched output contracts are not
supported. TensorRT I/O must be linear FP32 or FP16 device tensors.

The input image is letterboxed with gray value 114, converted BGR to RGB,
transposed to NCHW, and normalized to `[0,1]`. Monochrome images are replicated
into three channels; train and validate on equivalent monochrome imagery if
deploying to the present cameras. Boxes are restored to the capture image
coordinates using the actual resize and padding, clipped, and filtered.

Export a **trained** `.pt` checkpoint on a training workstation with an
Ultralytics version that supports YOLOv8/YOLO11:

```bash
yolo export model=best.pt format=onnx imgsz=640 batch=1 dynamic=False nms=False opset=17
```

Copy `best.onnx` to the Jetson's `models/` directory. Build a raw engine on the
target Jetson using its own TensorRT installation:

```bash
/usr/src/tensorrt/bin/trtexec --onnx=models/best.onnx --saveEngine=models/game-piece.engine --fp16 --skipInference
```

The direct backend reads raw TensorRT plans. Some Ultralytics `.engine`
exports contain a metadata header; use the ONNX plus `trtexec` procedure above
to create a compatible file. Rebuild an engine after changing JetPack,
TensorRT, GPU, or the model. Use trusted model files. The sample project does
not include a trained game-piece model.

Configure:

```json
{
  "backend": "tensorrt",
  "model_path": "../models/game-piece.engine",
  "labels": ["ball"],
  "input_size": 640,
  "confidence_threshold": 0.35,
  "iou_threshold": 0.45,
  "max_detections": 100
}
```

`input_size` may also be `[width,height]` and must match the engine. Missing or
unsupported model files fail explicitly. They never silently fall back to a
different detector. TensorRT execution reuses device buffers and serializes
calls through one context. Synchronous host/device copies keep buffer lifetime
straightforward; benchmark on the actual camera resolution before setting a
robot loop expectation. Up to 3,000 highest-scoring candidates enter NMS to
bound CPU work.

The model export and runtime contracts follow the primary documentation for
[Ultralytics export](https://docs.ultralytics.com/modes/export),
[Ultralytics detection output formats](https://docs.ultralytics.com/guides/end2end-detection),
and [NVIDIA TensorRT Python inference](https://docs.nvidia.com/deeplearning/tensorrt/10.x.x/inference-library/python-api-docs.html).

## Results and camera calibration

Every detection contains JSON-native `class_id`, `label`, `confidence`,
`bbox_xyxy`, `center`, and `area_fraction`. Bounding boxes are `[left,top,right,bottom]`
in capture-image pixels, with right/bottom edges exclusive for the contour and
HSV backends. `center` is the bounding-box center; `area_fraction` is bounding
box area divided by image area for all backends. No distance or 3D pose is
invented from a 2D detection.

With calibration, `yaw_deg` is positive to camera right and `pitch_deg` is
positive up. These are individual horizontal/vertical bearing angles from
undistorted normalized image coordinates, not robot or field coordinates.
They require `camera_matrix` (3x3), `dist_coeffs`, `width`, and `height` in the
calibration JSON. Frame size must exactly match the calibration. Without
calibration, angle fields are omitted. Camera mounting transforms, intake
geometry, timestamps, and stale-target rejection belong in robot integration.

## Collect a useful training dataset

Capture the actual game piece once the game is known, including small wiffle
balls if relevant. Use the production camera, lens, resolution, exposure, and
mounting height. Collect near/far, partial occlusions, multiple touching balls,
motion blur, varying field lights, and empty-field frames with plausible
confusers. For the current camera, retain monochrome images; collect a separate
representative color set when the color camera arrives.

Label consistent object bounds and split training/validation/test sets by
recording session so neighboring frames do not inflate evaluation scores.
Measure false positives on negative backgrounds and recall at intake distance.
Validate latency, precision/recall, and target stability on held-out field
recordings before connecting detections to autonomous robot actions.

## Python and verification

```python
from custom_vision.objects import ObjectPipeline

pipeline = ObjectPipeline({"backend": "contour"}, calibration=None)
detections = pipeline.process(frame_bgr)  # uint8 grayscale or BGR NumPy image
pipeline.close()
```

Run CPU synthetic checks with `.venv/bin/python -m pytest tests/test_objects.py`.
They cover monochrome round candidates and rejected backgrounds, hue wrapping,
calibrated bearings, letterbox inversion, per-class NMS, invalid model contracts,
RGB preprocessing, and JSON serialization. They do not establish real-game
accuracy. Run the real GPU smoke test with
`.venv/bin/python -m pytest tests/test_tensorrt_integration.py -q -rs`. It builds
a tiny input-dependent TensorRT network with input `[1,3,32,32]` and output
`[1,5,1]` in a temporary directory. It verifies actual device inference,
host/device copies, buffer ownership, cleanup, model validation, and grayscale
input through the object pipeline. It skips on hosts without TensorRT/CUDA.
Its detections are explicitly synthetic and do not measure trained accuracy.
