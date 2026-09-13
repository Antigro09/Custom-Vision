#!/usr/bin/env python3
"""Capture unannotated calibration/training images from a UVC camera."""
import argparse
import time
from pathlib import Path
import cv2
from custom_vision.app import open_camera


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default='0')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=800)
    parser.add_argument('--fps', type=int, default=120)
    parser.add_argument('--count', type=int, default=30)
    parser.add_argument('--interval', type=float, default=1.0)
    args = parser.parse_args()
    if args.count <= 0 or args.interval <= 0:
        parser.error('count and interval must be positive')
    args.output.mkdir(parents=True, exist_ok=True)
    source = int(args.source) if args.source.isdigit() else args.source
    cap = open_camera(dict(source=source, backend='v4l2', fourcc='MJPG', width=args.width, height=args.height, fps=args.fps))
    next_save = time.monotonic()
    count = 0
    try:
        while count < args.count:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('Camera disconnected')
            if time.monotonic() >= next_save:
                path = args.output / f'{time.time_ns()}.png'
                if not cv2.imwrite(str(path), frame):
                    raise RuntimeError(f'Could not save {path}')
                print(path)
                count += 1
                next_save = time.monotonic() + args.interval
    finally:
        cap.release()


if __name__ == '__main__':
    main()
