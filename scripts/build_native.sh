#!/usr/bin/env bash
# Build project code only; preserve the Jetson's installed GPU libraries.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
vision_python="${VISION_PYTHON:-$project_dir/.venv/bin/python}"
if [[ ! -x "$vision_python" ]]; then
  echo "Create the project .venv first (scripts/setup_jetson.sh)." >&2
  exit 1
fi
pybind_cmake="$($vision_python -m pybind11 --cmakedir)"
cmake -S "$project_dir" -B "$project_dir/build/native" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython_EXECUTABLE="$vision_python" \
  -Dpybind11_DIR="$pybind_cmake" "$@"
cmake --build "$project_dir/build/native" --parallel "${VISION_BUILD_JOBS:-2}"
"$vision_python" -c 'from custom_vision._native import capabilities; print(capabilities())'
