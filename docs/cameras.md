# Cameras and tuning

The supplied monochrome camera is UVC over USB2 and accepts MJPEG only. Runtime
selects V4L2 and requests FOURCC MJPG, 1280x800, 120 FPS. Inspect actual modes:

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
v4l2-ctl -d /dev/video0 --list-ctrls-menus
```

Use the exact names/ranges returned by the device when setting controls. The
user lists brightness, contrast, saturation, hue, gain, auto exposure, exposure,
gamma, sharpness, backlight compensation, auto white balance, white balance
temperature and convert_rgb; these may use different V4L2 names. Focus/zoom,
pan/tilt and iris are unsupported and are not set by the runtime.

For AprilTags, measure exposure under competition-like lighting. Shorter exposure
reduces motion blur; excessive gain can damage tag edges. Fix exposure/gain once
a useful operating point is found. Do not infer numeric exposure units from the
product description. Factory focus at 5ft and a non-distortion lens do not provide
intrinsic calibration values. Calibrate at 1280x800 with the installed lens and
orientation, and keep resolution matched at runtime.

The capture thread continuously drains decoded live frames, discarding intermediate
frames when inference cannot keep up. This bounds application backlog but does
not establish sensor exposure timestamps. Camera/driver/USB buffering and JPEG
decode latency must still be measured. 120 FPS capture capability does not mean
120 FPS pose or neural processing. Avoid saturating one USB2 hub with multiple
high-rate cameras; validate each negotiated mode and throughput.

The browser groups resolution, frame rate and format into actual driver-advertised
mode tuples. Preview rotation is applied after detection and overlay projection;
calibration always refers to the raw image. Changing camera mounting requires
updated extrinsics even when the preview looks upright.

The second tag camera has a disabled `rear_tags` profile. Enable it only after
selecting its own stable source and calibration. Start with two detector threads
per camera on this six-core Jetson and measure combined load.

The future color camera and floor-pickup design are described in
[the object proposal](OBJECT_ACQUISITION_PROPOSAL.md). New object work is paused;
no game-piece class or camera specification is assumed.
