#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -ne 2 ]; then printf '%s\n' 'Usage: scripts/build_engine.sh models/gamepiece.onnx models/gamepiece.engine' >&2; exit 2; fi
trtexec_path=/usr/src/tensorrt/bin/trtexec
if [ ! -x "$trtexec_path" ]; then trtexec_path="$(command -v trtexec)"; fi
workspace_mb="${VISION_TRT_WORKSPACE_MB:-256}"
if [[ ! "$workspace_mb" =~ ^[0-9]+$ ]] || (( workspace_mb < 64 || workspace_mb > 2048 )); then
  echo "VISION_TRT_WORKSPACE_MB must be an integer in64..2048" >&2; exit 2
fi
# Input must already have static shape [1,3,H,W]; see docs/objects.md.
"$trtexec_path" --onnx="$1" --saveEngine="$2" --fp16 --memPoolSize="workspace:$workspace_mb" --skipInference
