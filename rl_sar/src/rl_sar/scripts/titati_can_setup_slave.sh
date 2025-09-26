#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
LOG_DIR="${REPO_ROOT}/logs"
CAN_INTERFACE="can0"
LOG_PREFIX="${LOG_DIR}/titati_slave"

mkdir -p "${LOG_DIR}"

stop_systemd_service() {
    local service_name="$1"
    if systemctl list-unit-files 2>/dev/null | grep -q "^${service_name}"; then
        echo "[INFO] Stopping systemd service ${service_name}" >&2
        sudo systemctl stop "${service_name}" 2>/dev/null || true
    fi
}

prepare_can_interface() {
    echo "[INFO] Configuring ${CAN_INTERFACE} for CAN-FD" >&2
    if ip link show "${CAN_INTERFACE}" &>/dev/null; then
        sudo ip link set "${CAN_INTERFACE}" down || true
    fi
    sudo ip link set "${CAN_INTERFACE}" up type can \
        bitrate 1000000 sample-point 0.80 \
        dbitrate 8000000 dsample-point 0.80 \
        fd on restart-ms 100
    sudo ifconfig "${CAN_INTERFACE}" txqueuelen 1000
}

source_ros_environment() {
    local ros_distro="${ROS_DISTRO:-humble}"
    local ros_setup="/opt/ros/${ros_distro}/setup.bash"
    if [ -f "${ros_setup}" ]; then
        # shellcheck disable=SC1090
        source "${ros_setup}"
    else
        echo "[WARNING] ROS setup file ${ros_setup} not found. Set ROS_DISTRO or install ROS 2." >&2
    fi

    local workspace_setup="${REPO_ROOT}/install/setup.bash"
    if [ ! -f "${workspace_setup}" ]; then
        echo "[ERROR] ${workspace_setup} not found. Run ./build.sh -m first." >&2
        exit 1
    fi
    # shellcheck disable=SC1090
    source "${workspace_setup}"
}

launch_background() {
    local label="$1"
    shift
    local log_file="${LOG_PREFIX}_${label}.log"
    echo "[INFO] Launching ${label} (log: ${log_file})" >&2
    nohup "$@" >"${log_file}" 2>&1 &
}

stop_systemd_service "tita-bringup.service"
prepare_can_interface
source_ros_environment

export TITATI_NAMESPACE="${TITATI_NAMESPACE:-titati}"

launch_background "battery" ros2 launch battery_device battery_device_node.launch.py namespace:="${TITATI_NAMESPACE}"
launch_background "canfd_router" ros2 run titati_canfd_router titati_canfd_router_node --ros-args --remap __ns:="/${TITATI_NAMESPACE}"

echo "[INFO] Titati slave pre-launch complete." >&2
echo "[INFO] Keep this terminal open to monitor logs in ${LOG_DIR}." >&2
