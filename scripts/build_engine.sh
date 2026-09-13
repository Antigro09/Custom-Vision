#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -ne 2 ]; then printf '%s\n' 'Usage: scripts/build_engine.sh models/gamepiece.onnx models/gamepiece.engine' >&2; exit 2; fi
trtexec_path=/usr/src/tensorrt/bin/trtexec
if [ ! -x "$trtexec_path" ]; then trtexec_path="$(command -v trtexec)"; fi
# Input must already have static shape [1,3,H,W]; see docs/objects.md.
"$trtexec_path" --onnx="$1" --saveEngine="$2" --fp16 --memPoolSize=workspace:1024 --skipInference
