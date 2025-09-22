#!/bin/bash
set -euo pipefail

CAN_IFACE="${1:-can0}"

sudo ip link set "${CAN_IFACE}" down || true
sudo ip link set "${CAN_IFACE}" up type can bitrate 1000000 sample-point 0.80 \
    dbitrate 8000000 dsample-point 0.80 fd on restart-ms 100
sudo ifconfig "${CAN_IFACE}" txqueuelen 1000

echo "[titati_can_setup_master] ${CAN_IFACE} configured for CAN-FD 1M/8M."
