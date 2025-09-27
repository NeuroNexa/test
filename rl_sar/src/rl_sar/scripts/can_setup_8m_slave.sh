#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/../../.." && pwd)
BIN_DIR="${PROJECT_ROOT}/cmake_build/bin"
LOG_DIR="${PROJECT_ROOT}/logs"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

mkdir -p "${LOG_DIR}"

if systemctl list-unit-files | grep -q "tita-bringup.service"; then
  sudo systemctl stop tita-bringup.service || true
fi

sudo ip link set can0 down || true
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.80 dbitrate 8000000 dsample-point 0.80 fd on restart-ms 100
sudo ifconfig can0 txqueuelen 1000

echo "Configured can0 for 1Mbps/8Mbps FD operation."

if command -v pkill >/dev/null 2>&1; then
  pkill -f titati_canfd_router_node || true
  pkill -f titati_battery_device_node || true
fi

if [ -f "${ROS_SETUP}" ]; then
  # shellcheck source=/dev/null
  source "${ROS_SETUP}"
fi

export LD_LIBRARY_PATH="${PROJECT_ROOT}/cmake_build/lib:${LD_LIBRARY_PATH:-}"

start_node() {
  local executable="$1"
  local log_name="$2"
  if [ ! -x "${BIN_DIR}/${executable}" ]; then
    echo "Warning: ${executable} not found in ${BIN_DIR}; build the project with ./build.sh -m."
    return
  fi
  nohup "${BIN_DIR}/${executable}" > "${LOG_DIR}/${log_name}" 2>&1 &
  local pid=$!
  echo "Started ${executable} (PID ${pid}) -> ${LOG_DIR}/${log_name}"
}

start_node titati_canfd_router_node titati_canfd_router_slave.log
start_node titati_battery_device_node titati_battery_device_slave.log

echo "Slave CAN watchdog and battery services are running in the background."
