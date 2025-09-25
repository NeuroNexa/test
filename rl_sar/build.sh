#!/bin/bash
# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "$0")" && pwd)
BUILD_DIR="${ROOT_DIR}/cmake_build"
SRC_DIR="${ROOT_DIR}/src/rl_sar"

usage() {
    cat <<USAGE
Usage: ./build.sh [OPTIONS]

Options:
  -c, --clean    Remove cmake build artifacts
  -m, --cmake    Build using CMake (default behaviour, kept for backward compatibility)
  -h, --help     Show this help message
USAGE
}

clean() {
    echo "Removing build directories..."
    rm -rf "${BUILD_DIR}" "${ROOT_DIR}/build" "${ROOT_DIR}/devel" \
           "${ROOT_DIR}/install" "${ROOT_DIR}/log" "${ROOT_DIR}/logs" \
           "${ROOT_DIR}/.catkin_tools"
    echo "Clean complete."
}

configure() {
    echo "Configuring CMake project..."
    cmake -S "${SRC_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=RelWithDebInfo
}

build() {
    echo "Building rl_sar hardware targets..."
    cmake --build "${BUILD_DIR}" -j"$(nproc)"
    echo "Build complete. Binaries are available in ${BUILD_DIR}/bin"
}

if [[ $# -gt 0 ]]; then
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -c|--clean)
                clean
                exit 0
                ;;
            -m|--cmake)
                # No-op flag for backward compatibility with previous workflow
                ;;
            -h|--help)
                usage
                exit 0
                ;;
            *)
                echo "Unknown option: $1" >&2
                usage
                exit 1
                ;;
        esac
        shift
    done
fi

configure
build
