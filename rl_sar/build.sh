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
  <pkg...>       Build the listed ROS 2 packages with colcon
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

ros_build() {
    local packages=("$@")
    if [[ ${#packages[@]} -eq 0 ]]; then
        echo "No ROS packages specified." >&2
        exit 1
    fi

    if ! command -v colcon >/dev/null 2>&1; then
        echo "colcon not found. Please source your ROS 2 environment." >&2
        exit 1
    fi

    echo "-------------------------------------------------------------------"
    echo "[Running ROS Build]"
    echo "-------------------------------------------------------------------"
    echo "[Cleaning Existing Symlinks]"
    rm -rf "${ROOT_DIR}/build" "${ROOT_DIR}/install" "${ROOT_DIR}/log"
    echo "-------------------------------------------------------------------"
    echo "[Using colcon build]"
    echo "Building specific packages: ${packages[*]}"
    colcon build --symlink-install --packages-select "${packages[@]}"
    echo "ROS build completed!"
}

PACKAGES=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        -c|--clean)
            clean
            exit 0
            ;;
        -m|--cmake)
            # flag retained for compatibility; no-op because CMake build is default
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            while [[ $# -gt 0 ]]; do
                PACKAGES+=("$1")
                shift
            done
            break
            ;;
        -*)
            echo "Unknown option: $1" >&2
            usage
            exit 1
            ;;
        *)
            PACKAGES+=("$1")
            ;;
    esac
    shift || true
done

if [[ ${#PACKAGES[@]} -gt 0 ]]; then
    ros_build "${PACKAGES[@]}"
    exit 0
fi

configure
build
