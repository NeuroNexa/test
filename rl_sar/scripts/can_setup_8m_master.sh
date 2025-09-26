#!/bin/bash
set -e

sudo systemctl stop tita-bringup.service >/dev/null 2>&1 || true
sudo ip link set can0 down >/dev/null 2>&1 || true
sudo ip link set can0 up type can \
  bitrate 1000000 sample-point 0.80 \
  dbitrate 8000000 dsample-point 0.80 fd on restart-ms 100
sudo ifconfig can0 txqueuelen 1000

echo "[Titati] Master CAN-FD interface can0 configured to 1 Mbps / 8 Mbps."
