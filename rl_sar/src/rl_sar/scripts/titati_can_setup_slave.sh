#!/bin/bash
set -euo pipefail

CAN_IFACE="${1:-can0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RL_SAR_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROUTER_BIN="${RL_SAR_ROOT}/cmake_build/bin/titati_canfd_router_node"

sudo ip link set "${CAN_IFACE}" down || true
sudo ip link set "${CAN_IFACE}" up type can bitrate 1000000 sample-point 0.80 \
    dbitrate 8000000 dsample-point 0.80 fd on restart-ms 100
sudo ifconfig "${CAN_IFACE}" txqueuelen 1000

echo "[titati_can_setup_slave] ${CAN_IFACE} configured for CAN-FD 1M/8M."

echo "[titati_can_setup_slave] Launching Titati CAN-FD router (Ctrl+C to stop)..."
if [[ ! -x "${ROUTER_BIN}" ]]; then
    echo "Binary ${ROUTER_BIN} not found. Build rl_sar with CMake first." >&2
    exit 1
fi

"${ROUTER_BIN}" --stay-alive
