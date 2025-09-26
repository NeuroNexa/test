#!/bin/bash
# Configure CAN interface on the slave Titati Jetson. This prepares the bus for master control.
# Usage: ./titati_can_setup_slave.sh [can-interface]
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

echo "[Titati] ${CAN_IFACE} configured (slave)."
echo "If the slave contributes additional actuators, export TITATI_CAN_INTERFACE=${CAN_IFACE} and"
echo "TITATI_CAN_ID_OFFSET before running diagnostics or custom tools."
