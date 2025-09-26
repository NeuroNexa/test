#!/bin/bash
# Configure CAN interface on the master Titati Jetson so rl_sar can take over the motors.
# Usage: ./titati_can_setup_master.sh [can-interface]
set -e

CAN_IFACE=${1:-can0}
BITRATE=${TITATI_CAN_BITRATE:-1000000}
DBITRATE=${TITATI_CAN_DBITRATE:-8000000}
SAMPLE_POINT=${TITATI_CAN_SAMPLE_POINT:-0.80}
DSAMPLE_POINT=${TITATI_CAN_DSAMPLE_POINT:-0.80}
QUEUE_LEN=${TITATI_CAN_QUEUE_LEN:-1000}

sudo systemctl stop tita-bringup.service >/dev/null 2>&1 || true
sudo ip link set "${CAN_IFACE}" down >/dev/null 2>&1 || true
sudo ip link set "${CAN_IFACE}" up type can bitrate "${BITRATE}" sample-point "${SAMPLE_POINT}" \
    dbitrate "${DBITRATE}" dsample-point "${DSAMPLE_POINT}" fd on restart-ms 100
sudo ifconfig "${CAN_IFACE}" txqueuelen "${QUEUE_LEN}"

echo "[Titati] ${CAN_IFACE} configured (master)."
echo "Export TITATI_CAN_INTERFACE=${CAN_IFACE} before running rl_real_titati if you use a custom interface."
