#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "[run_titati_canfd_router] Please source your ROS 2 workspace before running this script." >&2
  exit 1
fi

ros2 run rl_sar titati_canfd_router_node "$@"
