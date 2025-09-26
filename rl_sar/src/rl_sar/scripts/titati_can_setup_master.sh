#!/bin/bash
# Configure the Titati master Jetson CAN interface and spawn the required ROS power services.
# Usage: ./titati_can_setup_master.sh [can-interface]
set -euo pipefail

CAN_IFACE=${1:-can0}
BITRATE=${TITATI_CAN_BITRATE:-1000000}
DBITRATE=${TITATI_CAN_DBITRATE:-8000000}
SAMPLE_POINT=${TITATI_CAN_SAMPLE_POINT:-0.80}
DSAMPLE_POINT=${TITATI_CAN_DSAMPLE_POINT:-0.80}
QUEUE_LEN=${TITATI_CAN_QUEUE_LEN:-1000}
TITATI_NAMESPACE=${TITATI_NAMESPACE:-tita}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/../../.." && pwd)
BIN_DIR="${REPO_ROOT}/cmake_build/bin"
LOG_DIR="${REPO_ROOT}/cmake_build/logs"
mkdir -p "${LOG_DIR}"

if [[ ! -x "${BIN_DIR}/battery_device_node" || ! -x "${BIN_DIR}/titati_canfd_router_node" ]]; then
    echo "[Titati] Missing battery_device_node or titati_canfd_router_node binaries in ${BIN_DIR}." >&2
    echo "[Titati] Please run ./build.sh -m before launching the hardware stack." >&2
    exit 1
fi

sudo systemctl stop tita-bringup.service >/dev/null 2>&1 || true
sudo ip link set "${CAN_IFACE}" down >/dev/null 2>&1 || true
sudo ip link set "${CAN_IFACE}" up type can bitrate "${BITRATE}" sample-point "${SAMPLE_POINT}" \
    dbitrate "${DBITRATE}" dsample-point "${DSAMPLE_POINT}" fd on restart-ms 100
sudo ifconfig "${CAN_IFACE}" txqueuelen "${QUEUE_LEN}"

ROS_DISTRO_ENV=${ROS_DISTRO:-humble}
if [[ -f "/opt/ros/${ROS_DISTRO_ENV}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO_ENV}/setup.bash"
else
    echo "[Titati] Warning: ROS 2 setup for ${ROS_DISTRO_ENV} not found. Ensure ROS is sourced before running rl_real_titati." >&2
fi

start_background_node() {
    local binary_name="$1"
    local log_file="${LOG_DIR}/${binary_name}.log"
    pkill -f "${BIN_DIR}/${binary_name}" >/dev/null 2>&1 || true
    nohup "${BIN_DIR}/${binary_name}" --ros-args --namespace "${TITATI_NAMESPACE}" \
        >"${log_file}" 2>&1 &
    echo "[Titati] Launched ${binary_name} (log: ${log_file})."
}

start_background_node battery_device_node
start_background_node titati_canfd_router_node

echo "[Titati] ${CAN_IFACE} configured for master."
echo "[Titati] Active services:"
echo "    - battery_device_node → /${TITATI_NAMESPACE}/power_state_set, /${TITATI_NAMESPACE}/power_heart_beat, /${TITATI_NAMESPACE}/power_self_test"
echo "    - titati_canfd_router_node → CAN power heartbeat bridge"
echo "[Titati] Run rl_real_titati from ${REPO_ROOT}/cmake_build/bin after the slave bring-up completes."
