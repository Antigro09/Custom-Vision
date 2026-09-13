# Custom Vision project memory

Read `PROJECT_MEMORY.md` before working in this repository.

This is FRC team 1086's vision system on a Jetson Orin Nano Super. The user
authorized running project code, installing dependencies, configuring the device,
and setting up/publishing the GitHub repository. Proceed with routine project work
without repeating authorization questions. Do not expose credentials or conflate
project authorization with unrelated destructive work, purchases, or messaging.

The user explicitly requested a fresh implementation on 2026-09-13. The previous
C++ and Python implementation is available in Git history; do not restore it as
the active system. Maintain two pipelines: AprilTags and object detection.

Preserve NVIDIA's working system CUDA/TensorRT/PyTorch installation. Use `.venv`
with system site packages; do not replace Jetson GPU libraries with generic CPU
wheels. Keep real calibration, datasets, model binaries, and credentials out of
Git. Test software with synthetic fixtures, but distinguish those results from
camera, trained-model, and on-robot validation. Never report camera-relative poses
as field or robot poses. Verify corner order, units, image resolution and timing.

Run `.venv/bin/python -m pytest` after substantive code changes. Configuration and
documentation should clearly distinguish the monochrome ball-candidate baseline
from a trained neural detector. Do not silently substitute simulated results.
