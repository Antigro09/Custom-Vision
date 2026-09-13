#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m venv --system-site-packages .venv
# Keep NVIDIA system packages; do not install torch/CUDA/TensorRT from PyPI.
export PIP_CONFIG_FILE=/dev/null
.venv/bin/python -m pip install --upgrade pip 'setuptools>=68,<80' wheel packaging 'setuptools-scm>=8,<9' scikit-build 'cmake<4'
# pupil-apriltags builds natively on aarch64. Limit parallel builds on an 8GB Jetson.
export CMAKE_BUILD_PARALLEL_LEVEL=2
.venv/bin/python -m pip install --no-build-isolation 'pupil-apriltags==1.0.4.post11'
if ! .venv/bin/python -c 'import cv2; assert hasattr(cv2, "aruco")'; then
  .venv/bin/python -m pip install 'opencv-contrib-python==4.10.0.84'
fi
.venv/bin/python -m pip install -e '.[test]'
.venv/bin/python scripts/doctor.py
.venv/bin/python -m custom_vision.app --check
