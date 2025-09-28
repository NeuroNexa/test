#!/bin/bash
# Launch the Titati power handshake helpers (power services + CAN-FD gateway)
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_DIR=$(dirname "$SCRIPT_DIR")

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "[ERROR] ROS 2 environment not found. Please source /opt/ros/<distro>/setup.bash first." >&2
  exit 1
fi

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  # Source the workspace setup under a relaxed nounset policy because the
  # generated colcon setup scripts intentionally reference optional variables
  # such as COLCON_TRACE. The workspace setup re-enables strict mode below.
  set +u
  # shellcheck disable=SC1090
  source "$WORKSPACE_DIR/install/setup.bash"
  set -u
else
  echo "[WARNING] install/setup.bash not found. Make sure you have built the workspace." >&2
fi

exec ros2 launch rl_sar titati_power_stack.launch.py "$@"
