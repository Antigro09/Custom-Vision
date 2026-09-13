#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
project_dir="$PWD"
mkdir -p "$HOME/.config/systemd/user"
if [ ! -f config/local.yaml ]; then cp config/vision.yaml config/local.yaml; fi
cat > "$HOME/.config/systemd/user/custom-vision.service" <<UNIT
[Unit]
Description=FRC Team 1086 Custom Vision
After=network-online.target

[Service]
Type=simple
WorkingDirectory=$project_dir
ExecStart=$project_dir/.venv/bin/python -m custom_vision.app --config $project_dir/config/local.yaml
Restart=on-failure
RestartSec=3
TimeoutStopSec=8
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=default.target
UNIT
systemctl --user daemon-reload
printf '%s\n' 'User service installed. Start after connecting and configuring cameras:' 'systemctl --user enable --now custom-vision' 'For boot without login, an administrator can run: sudo loginctl enable-linger jetsonorin'
