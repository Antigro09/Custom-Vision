# YOLO model export and runtime contract

Use **YOLO26n bounding-box detection first** for the intake camera. It has less
mask work and needs box labels instead of traced object boundaries. Instance
segmentation is available when testing shows touching/partly hidden objects or
contact-point quality justify its extra latency and labeling effort. A ball's
circular outline is a segmentation mask, not an oriented bounding box. A mask
still does not measure depth or reveal an occluded floor contact.

The Jetson runtime imports TensorRT and CUDA directly. It does not import ROS,
PyTorch or Ultralytics during inference. Model training/export belongs in the
separate export environment or a training workstation; preserve JetPack's GPU
libraries. Model weights, ONNX files, engines and captured training data stay out
of Git. An engine must be built on this Jetson/TensorRT installation; it is not a
portable interchange format.

## Export contracts

The runtime selects an explicit format; it never guesses class order or silently
interprets an unsupported tensor shape. Static batch-one FP32/FP16 linear device
I/O is required. The input is `[1,3,H,W]`, normalized RGB, letterboxed with 114-gray
padding. `input_size` is `[width,height]` or a square integer. Postprocessing uses
the exact rounded letterbox transform to recover capture-image coordinates.

| `output_format` | Task | Predictions | Other output |
|---|---|---|---|
| `yolo26_end2end` | `detect` | `[1,N,6]`: x1,y1,x2,y2,score,class ID | none |
| `yolo26_end2end` | `segment` | `[1,N,6+nm]`: above + mask coefficients | `[1,nm,Hproto,Wproto]` |
| `yolov8_raw` | `detect` | `[1,4+nc,N]`: cx,cy,w,h, class probabilities | none |
| `yolov8_raw` | `segment` | `[1,4+nc+nm,N]`: above + mask coefficients | `[1,nm,Hproto,Wproto]` |

`yolov8_raw` also describes compatible YOLO11 and YOLO26 one-to-many outputs.
End-to-end predictions are confidence-filtered and sorted without repeating NMS.
Raw predictions use class-aware NMS. NaN/Infinity detections are removed; invalid
class IDs, logits instead of probabilities, and incompatible contracts produce
an error instead of robot targets. Full `labels` must match training class order;
optional `allowed_class_ids` selects useful classes without changing that order.

Current official YOLO26 uses `nms=False` to request the one-to-one export. An
omitted `nms` selects the one-to-many head in current releases. Export behavior was
checked against Ultralytics **8.4.150**; reverify shapes when upgrading. The official
[YOLO26 guide](https://docs.ultralytics.com/models/yolo26/) and
[head implementation](https://github.com/ultralytics/ultralytics/blob/v8.4.150/ultralytics/nn/modules/head.py)
document the contracts above. Prototype/coefficients follow the official
[segmentation implementation](https://github.com/ultralytics/ultralytics/blob/v8.4.150/ultralytics/models/yolo/segment/predict.py).

For team-trained `best.pt`, export **ONNX** with `batch=1`, `dynamic=False`,
`nms=False`, `imgsz=640`, `opset=17`, then build a raw plan with JetPack's
`/usr/src/tensorrt/bin/trtexec --onnx=models/best.onnx --saveEngine=models/game-piece.engine --fp16 --skipInference`.
The runtime accepts raw TensorRT plans, not metadata-prefixed Ultralytics `.engine`
containers. Match `task` to the trained detection or segmentation model and use
`output_format: yolo26_end2end` for this export. Never rename a general COCO model
class as the team's game piece: fine-tuning and field validation remain required.

## Bounded processing and results

Each camera owns a TensorRT context, CUDA stream, persistent GPU buffers and pinned
host buffers. Input copies, inference and output copies share that stream. A
protected decoder consumes reusable outputs before another call can overwrite
them. No GPU device allocation occurs per frame. RGB tensor storage is reused;
letterboxing and compact mask decoding currently run on the CPU. Native channel
splitting reuses RGB scratch planes; contiguous FP32 normalization avoids a
temporary FP64 path. FP16 inputs use an exact 256-value normalization lookup.
Engine inference
is serialized within one instance; camera workers use separate instances.

`max_detections` defaults to 16 and is limited to 32. Segmentation defaults to the
best 8 masks (`max_masks`, range 1–16), with at most 32 contour points each
(`max_contour_points`, range 4–64). Only the prototype crop inside the selected box
is combined with coefficients; no capture-resolution full mask is allocated or
published. `mask_threshold` defaults to 0.5. Prototype channels are limited to 128
and total prototype values to 13,107,200. Empty masks retain their box with an
explicit `segmentation_status`, and targets past the mask budget report `mask_limit`.

Every neural detection contains `bbox_xyxy`, `center`, `class_id`, `label`,
`confidence`, `confidence_kind: model_score`, and `area_fraction`. Segmentation
adds:

```json
{
  "segmentation_status": "valid",
  "segmentation": {
    "contour_px": [[100.5, 200.5], [110.5, 220.5], [120.5, 200.5]],
    "centroid_px": [110.5, 207.2],
    "bottom_px": [110.5, 220.5],
    "area_px": 180.0,
    "approximate": true,
    "resolution": [160, 160]
  }
}
```

The contour describes the largest external component, omitting small interior
holes. Anchors and area are approximate because decoding stays at prototype
resolution. `bottom_px` is an image-outline anchor, not a proven physical contact;
the ranging layer must select an appropriate calibrated target plane and reject
unusable geometry. Full masks are kept off NetworkTables to bound bandwidth.

Pipeline timings expose `preprocess_ms`, `inference_ms`, `decode_ms`, `total_ms`.
Inference time is host wall time including transfers and stream synchronization.
These timings do not include camera exposure, USB delivery or robot reception.


## Pinned tools on this Jetson

```bash
./scripts/setup_yolo_tools.sh
.venv-export/bin/python scripts/train_yolo.py --data data/team/dataset.yaml --weights models/yolo26n.pt
.venv-export/bin/python scripts/export_yolo.py --weights data/training/gamepiece/weights/best.pt --output-dir models/gamepiece --build-engine
```

Training requires the team's actual train/validation dataset; it is not run by
setup or deployment. Segmentation requires the corresponding segmentation
checkpoint and polygon/mask labels. The export helper pins Ultralytics 8.4.150,
uses batch 1/static shapes, an explicit NMS-free head, and validates ONNX outputs.
Cloudpickle 3.1.2 and Ultralytics-thop 2.1.6 are pinned in this export environment
to satisfy the export/training package's current requirements.
It writes original class labels and model hashes to a JSON manifest. Copy these
labels, task, input_size and output_format into the camera's settings exactly.
Build each engine on the target Jetson/TensorRT version. Scripts do not install
or replace NVIDIA torch/CUDA/TensorRT. `.venv-export` is separate from `.venv`.

The default TensorRT build workspace is 256 MiB to leave room for other applications
on the 8 GB Jetson; `VISION_TRT_WORKSPACE_MB` can set 64..2048. Run export/build tasks
sequentially and close completed test processes before profiling. Builder workspace
is not a guarantee of total memory use. The scripts leave working system services
and other projects alone.

Ultralytics code/models have their own upstream license; see
[the official license information](https://docs.ultralytics.com/models/yolo26).
Generic COCO pretrained weights used for benchmarking are not a trained detector
for the team's unconfirmed game piece and are not enabled in production config.
