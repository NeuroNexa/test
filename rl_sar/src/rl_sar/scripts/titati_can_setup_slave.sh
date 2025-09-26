#!/bin/bash
# Configure the slave Titati Jetson: stop the stock service, bring up CAN, and launch the ROS bringup stack.
set -e

CAN_IFACE=${1:-can0}
BITRATE=${TITATI_CAN_BITRATE:-1000000}
DBITRATE=${TITATI_CAN_DBITRATE:-8000000}
SAMPLE_POINT=${TITATI_CAN_SAMPLE_POINT:-0.80}
DSAMPLE_POINT=${TITATI_CAN_DSAMPLE_POINT:-0.80}
QUEUE_LEN=${TITATI_CAN_QUEUE_LEN:-1000}
LOG_DIR=${TITATI_CAN_LOG_DIR:-/tmp/titati}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_ROOT="$( cd "${SCRIPT_DIR}/../../.." && pwd )"

mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/slave_bringup.log"

source_ros_env() {
    if command -v ros2 >/dev/null 2>&1; then
        return 0
    fi

    if [[ -n "$ROS_DISTRO" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
        # shellcheck disable=SC1090
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
    else
        local candidates=("humble" "foxy")
        for distro in "${candidates[@]}"; do
            local setup="/opt/ros/${distro}/setup.bash"
            if [ -f "$setup" ]; then
                # shellcheck disable=SC1090
                source "$setup"
                break
            fi
        done
    fi

    command -v ros2 >/dev/null 2>&1
}

source_workspace_overlay() {
    if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
        # shellcheck disable=SC1090
        source "${WORKSPACE_ROOT}/install/setup.bash"
    elif [ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]; then
        # shellcheck disable=SC1090
        source "${WORKSPACE_ROOT}/devel/setup.bash"
    fi
}

echo "[Titati] Preparing CAN interface ${CAN_IFACE} on slave..."
sudo systemctl stop tita-bringup.service >/dev/null 2>&1 || true
sudo ip link set "${CAN_IFACE}" down >/dev/null 2>&1 || true
sudo ip link set "${CAN_IFACE}" up type can bitrate "${BITRATE}" sample-point "${SAMPLE_POINT}" \
    dbitrate "${DBITRATE}" dsample-point "${DSAMPLE_POINT}" fd on restart-ms 100
sudo ifconfig "${CAN_IFACE}" txqueuelen "${QUEUE_LEN}"

echo "[Titati] CAN interface ${CAN_IFACE} configured."

if ! source_ros_env; then
    echo "[Titati] ERROR: Unable to locate a ROS 2 environment. Please source /opt/ros/<distro>/setup.bash first." >&2
    exit 1
fi

source_workspace_overlay

if ! command -v ros2 >/dev/null 2>&1; then
    echo "[Titati] ERROR: 'ros2' command not available after sourcing environment." >&2
    exit 1
fi

if pgrep -f "battery_device_node" >/dev/null 2>&1; then
    pkill -f "battery_device_node" >/dev/null 2>&1 || true
    sleep 1
fi

if pgrep -f "titati_canfd_router_node" >/dev/null 2>&1; then
    pkill -f "titati_canfd_router_node" >/dev/null 2>&1 || true
    sleep 1
fi

echo "[Titati] Launching battery_device_node on slave..."
nohup ros2 launch battery_device battery_device_node.launch.py \
    >"${LOG_FILE%.log}_battery.log" 2>&1 &
BATTERY_PID=$!

echo "[Titati] Launching titati_canfd_router_node on slave..."
nohup ros2 run titati_canfd_router titati_canfd_router_node \
    >"${LOG_FILE%.log}_router.log" 2>&1 &
ROUTER_PID=$!

disown "$BATTERY_PID" "$ROUTER_PID" 2>/dev/null || true

echo "[Titati] battery_device_node PID ${BATTERY_PID}, titati_canfd_router_node PID ${ROUTER_PID}."
echo "Logs: ${LOG_FILE%.log}_battery.log and ${LOG_FILE%.log}_router.log"
echo "If the slave contributes additional actuators, export TITATI_CAN_INTERFACE=${CAN_IFACE} and"
echo "TITATI_CAN_ID_OFFSET before running diagnostics or custom tools."
