#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

launch_ros_router() {
  local cmd
  case "${ROS_DISTRO:-}" in
    noetic)
      cmd=(rosrun rl_sar titati_canfd_router_node)
      ;;
    "" )
      return 1
      ;;
    *)
      cmd=(ros2 run rl_sar titati_canfd_router_node)
      ;;
  esac

  if ! command -v "${cmd[0]}" >/dev/null 2>&1; then
    return 1
  fi

  if "${cmd[@]}" "$@"; then
    return 0
  fi

  local rc=$?
  echo "[run_titati_canfd_router] ${cmd[*]} failed with exit code ${rc}." >&2
  return 2
}

launch_cmake_router() {
  local repo_root
  repo_root="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
  local binary_path="${repo_root}/cmake_build/bin/titati_canfd_router_node"

  if [[ ! -x "${binary_path}" ]]; then
    echo "[run_titati_canfd_router] Unable to find ${binary_path}." >&2
    echo "[run_titati_canfd_router] Build the hardware stack first with ./build.sh -m (or source ./build.sh without ROS)." >&2
    exit 1
  fi

  exec "${binary_path}" "$@"
}

if launch_ros_router "$@"; then
  exit 0
fi

status=$?
if [[ ${status} -ne 1 ]]; then
  echo "[run_titati_canfd_router] Falling back to the standalone CMake binary." >&2
else
  echo "[run_titati_canfd_router] ROS environment not detected; launching the standalone CMake binary." >&2
fi

launch_cmake_router "$@"
