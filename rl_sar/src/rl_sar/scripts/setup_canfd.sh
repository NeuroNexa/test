#!/bin/bash
set -euo pipefail

sudo systemctl stop tita-bringup.service >/dev/null 2>&1 || true

if sudo ip link show can0 >/dev/null 2>&1; then
    sudo ip link set can0 down || true
else
    echo "[setup_canfd] can0 interface not present; attempting to create via socketcan module" >&2
fi

sudo ip link set can0 up type can bitrate 1000000 sample-point 0.80 \
    dbitrate 8000000 dsample-point 0.80 fd on restart-ms 100
sudo ifconfig can0 txqueuelen 1000

echo "[setup_canfd] can0 configured for Titati (1M/8M CAN-FD)."
