#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "[setup_titati_canfd] Please run as root (use sudo)." >&2
  exit 1
fi

CAN_DEV="${1:-can0}"

ip link set "${CAN_DEV}" down 2>/dev/null || true
ip link set "${CAN_DEV}" up type can \
  bitrate 1000000 sample-point 0.80 \
  dbitrate 8000000 dsample-point 0.80 \
  fd on restart-ms 100
ifconfig "${CAN_DEV}" txqueuelen 1000

ip -details link show "${CAN_DEV}"
