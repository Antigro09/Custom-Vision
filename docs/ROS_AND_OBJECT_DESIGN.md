# ROS and object detector choices

## Keep the direct NVIDIA library integration

ROS is a general robotics framework; NVIDIA Isaac ROS supplies Jetson-accelerated
components. The AprilTag CUDA path already links the standalone cuAprilTags
library distributed with Isaac ROS3.2. It does not run ROS nodes or publish ROS
topics. Adding ROS does not change the library's detector kernels automatically.

NITROS can avoid copies between compatible accelerated nodes composed in one
process. That is valuable for a larger ROS camera/rectification/inference graph.
Our current UVC camera → detector → NetworkTables application already owns its
buffers and workers directly, and would require an additional ROS-to-NT bridge.
There is no measured benefit that justifies replacing the working application
with ROS now. We keep the direct implementation and can revisit NITROS if the
system grows to stereo/depth, multiple accelerated graph stages or other ROS
components. This is an architectural judgment, not an unperformed ROS benchmark.
[NVIDIA NITROS architecture](https://nvidia-isaac-ros.github.io/v/release-3.2/concepts/nitros/index.html),
[the pinned cuAprilTags integration](../native/README.md).

## Bounding boxes first; instance segmentation supported

The traced cutout is **instance segmentation**: a separate pixel mask for each
object. It is not simply a circle detector. Bounding boxes require simpler labels
and postprocessing. For a ball with known radius, a reliable center plus a known
camera mount gives a useful initial range estimate. Segmentation can improve
visible centers and silhouettes for overlapping/irregular pieces, but partial
occlusion can still bias its center and apparent contact point.

Both detection and instance segmentation are implemented. Start with YOLO26n
bounding boxes and an explicitly measured ball-center plane. Compare a YOLO26n-seg
model on the same held-out team images and measured floor positions before
switching. Segmentation should earn its extra processing through lower position
error or better acquisition success. Bounding-box bottom center and mask bottom
are approximate contact points, not guarantees of true floor contact.

YOLO26's NMS-free head exports detection rows `[x1,y1,x2,y2,score,class]`.
Segmentation adds mask coefficients and a prototype tensor. Our runtime decodes
these explicitly; it also retains raw YOLOv8/YOLO11-style exports. No post-NMS
model tensor is silently interpreted as a raw tensor. The model's labels and
export format must agree with configuration.
[Ultralytics YOLO26](https://docs.ultralytics.com/models/yolo26),
[instance segmentation](https://docs.ultralytics.com/tasks/segment).

Inference uses TensorRT10/CUDA with persistent buffers and a stream per engine.
Python handles camera orchestration, postprocessing and geometry; TensorRT,
OpenCV and NumPy perform numerical operations in native code. Training/export
uses a separate pinned `.venv-export`; Ultralytics and PyTorch are not imported
by the live TensorRT runtime. No system GPU packages were replaced.

## What is implemented versus data-dependent

Implemented: box/mask inference, class filtering, bounded mask contours,
calibrated plane ranging with uncertainty limits, brief track association,
current-frame target selection, intake-offset approach displacement, NT4 output,
browser controls and explicit synthetic tests. Robot Java and autonomous motion
remain deferred. The approach displacement is not an obstacle-checked path.

Actual team fine-tuning requires the confirmed game piece, color-camera footage
and held-out validation data. No trained team model is fabricated. Official COCO
pretrained YOLO26n and YOLO26n-seg engines are stored under ignored
`models/benchmark/` solely for interface and speed testing, with original class
labels. They are not activated on the robot. The supplied object profile leaves
calibration, measured camera mount and target height unset until provided.

The intake camera's downward direction is accepted as planned. The remaining
calibration values are numerical inputs required by the implemented geometry;
nominal tilt alone cannot determine distance. See [geometry](object_geometry.md),
[export contracts](yolo_exports.md) and [object setup](objects.md).
