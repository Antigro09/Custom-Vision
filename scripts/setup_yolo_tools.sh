#!/usr/bin/env bash
# Separate export/training tools; preserve JetPack torch/CUDA and the live runtime.
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m venv --system-site-packages .venv-export
mkdir -p .cache/ultralytics
PIP_CONFIG_FILE=/dev/null .venv-export/bin/python -m pip install --no-deps 'ultralytics==8.4.150' 'cloudpickle==3.1.2' 'ultralytics-thop==2.1.6'
YOLO_CONFIG_DIR="$PWD/.cache/ultralytics" YOLO_AUTOINSTALL=false .venv-export/bin/python -c 'import ultralytics, torch, onnx; print("export tools:", ultralytics.__version__, "torch:", torch.__version__, "onnx:", onnx.__version__)'
