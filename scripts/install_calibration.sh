#!/usr/bin/env bash
# Keep mrcal/OpenCV calibration tools isolated from Jetson CUDA/TensorRT/PyTorch.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [[ ! -x /usr/bin/python3 ]] || ! command -v apt-get >/dev/null; then
  echo 'This installer supports Debian/Ubuntu (including JetPack Ubuntu). See docs/CALIBRATION.md for other hosts.' >&2
  exit 1
fi
if [[ $EUID -eq 0 ]]; then
  privilege=()
else
  privilege=(sudo)
fi
packages=(mrcal python3-mrcal mrgingham python3-scipy python3-yaml python3-venv)
# Do not replace NVIDIA's working Python OpenCV if it is already installed.
if ! /usr/bin/python3 -c 'import cv2; assert hasattr(cv2,"findChessboardCornersSB"); assert hasattr(cv2,"estimateChessboardSharpness")' >/dev/null 2>&1; then
  packages+=(python3-opencv)
fi
printf 'Installing distro packages only: %s\n' "${packages[*]}"
"${privilege[@]}" apt-get update
plan=$(mktemp)
trap 'rm -f "$plan"' EXIT
"${privilege[@]}" apt-get -s --no-remove install "${packages[@]}" > "$plan"
if grep -E '^(Inst|Remv) (cuda|nvidia|libnvinfer|libcudnn|tensorrt|python3-(torch|tensorrt))' "$plan"; then
  echo 'Refusing an APT plan that changes NVIDIA GPU packages. Inspect dependencies manually.' >&2
  exit 1
fi
"${privilege[@]}" apt-get --no-remove install -y "${packages[@]}"
if [[ ! -d .venv-calibration ]]; then
  /usr/bin/python3 -m venv --system-site-packages .venv-calibration
fi
if [[ ! -x .venv-calibration/bin/python ]]; then
  echo '.venv-calibration exists but is not a usable venv. Rename it, then rerun this script.' >&2
  exit 1
fi
.venv-calibration/bin/python calibration.py doctor --mrcal-python /usr/bin/python3
printf '\nReady: .venv-calibration/bin/python calibration.py --help\n'
printf 'No pip installs or service/camera/config changes requested; NVIDIA-package changes were checked before installation.\n'
