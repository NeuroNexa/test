#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
BUILD_DIR=${1:-${SCRIPT_DIR}/../../cmake_build}
ROUTER_BIN=${BUILD_DIR}/bin/titati_canfd_router

if [ ! -x "${ROUTER_BIN}" ]; then
    echo "[start_titati_router] titati_canfd_router binary not found at ${ROUTER_BIN}" >&2
    echo "Run build.sh -m or cmake to compile the Titati targets first." >&2
    exit 1
fi

exec "${ROUTER_BIN}"
