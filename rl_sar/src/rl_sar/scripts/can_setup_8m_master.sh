#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/../../.." && pwd)
INSTALL_SETUP="${PROJECT_ROOT}/cmake_build/install/setup.bash"
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
  pkill -f "ros2 launch rl_sar titati_bringup.launch.py" || true
fi

if [ -f "${ROS_SETUP}" ]; then
  # shellcheck source=/dev/null
  source "${ROS_SETUP}"
fi

if [ ! -f "${INSTALL_SETUP}" ]; then
  echo "Error: ${INSTALL_SETUP} not found. Run ./build.sh -m to create the install space."
  exit 1
fi

# shellcheck source=/dev/null
source "${INSTALL_SETUP}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "Error: ros2 CLI not found in PATH after sourcing ${INSTALL_SETUP}."
  exit 1
fi

export ROS_LOG_DIR="${LOG_DIR}"

nohup ros2 launch rl_sar titati_bringup.launch.py role:=master \
  > "${LOG_DIR}/titati_infrastructure_master.log" 2>&1 &
echo "Started Titati infrastructure launch (PID $!) -> ${LOG_DIR}/titati_infrastructure_master.log"
