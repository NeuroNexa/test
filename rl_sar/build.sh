# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

#!/bin/bash
set -e

# ========================
# Configuration
# ========================

# Color definitions
COLOR_ERROR='\033[0;31m'     # Red
COLOR_SUCCESS='\033[0;32m'   # Green
COLOR_WARNING='\033[1;33m'   # Yellow
COLOR_INFO='\033[0;34m'      # Blue
COLOR_DEBUG='\033[0;36m'     # Cyan
COLOR_RESET='\033[0m'        # Reset

PACKAGE_SYMLINK_ROOT="src/.ros_package_links"
USE_PACKAGE_LINKS=false
COLCON_EXTRA_ARGS=()
declare -a CMAKE_EXTRA_ARGS=()

# ========================
# Helper Functions
# ========================

print_separator() {
    echo -e "${COLOR_INFO}-------------------------------------------------------------------${COLOR_RESET}"
}

print_header() {
    print_separator
    echo -e "${COLOR_INFO}$1${COLOR_RESET}"
}

print_success() {
    echo -e "${COLOR_SUCCESS}$1${COLOR_RESET}"
}

print_warning() {
    echo -e "${COLOR_WARNING}$1${COLOR_RESET}"
}

print_error() {
    echo -e "${COLOR_ERROR}$1${COLOR_RESET}"
}

print_info() {
    echo -e "${COLOR_INFO}$1${COLOR_RESET}"
}

ask_confirmation() {
    local message="$1"
    echo -e -n "${COLOR_WARNING}$message (y/n): ${COLOR_RESET}"
    read -r response
    case "$response" in
        [yY]) return 0 ;;
        [nN]) return 1 ;;
        *)
            print_error "Please enter 'y' or 'n'"
            ask_confirmation "$message"
            ;;
    esac
}

# ========================
# Build Functions
# ========================

run_cmake_build() {
    print_header "[Running CMake Build]"
    print_warning "NOTE: CMake build is for hardware deployment only, not for simulation."
    print_separator

    cmake src/rl_sar/ -B cmake_build -DUSE_CMAKE=ON "${CMAKE_EXTRA_ARGS[@]}"
    cmake --build cmake_build -j4

    print_success "CMake build completed!"
}

run_ros_build() {
    local packages=("$@")
    local package_list=$(IFS=' '; echo "${packages[*]}")

    print_header "[Running ROS Build]"

    # Clean existing symlinks
    clean_existing_symlinks "${packages[@]}"

    # Detect incompatible artifacts
    detect_incompatible_build_artifacts

    # Create appropriate symlinks
    if [ ${#packages[@]} -eq 0 ]; then
        create_symlinks_for_all_packages
    else
        create_symlinks_for_specific_packages "${packages[@]}"
        prepare_package_workspace_symlinks "${packages[@]}"
    fi

    # Execute build
    if [ ${#packages[@]} -eq 0 ]; then
        if [[ "$ROS_DISTRO" == "noetic" ]]; then
            print_header "[Using catkin build]"
            print_info "Building all packages..."
            catkin build
        else
            print_header "[Using colcon build]"
            print_info "Building all packages..."
            colcon build --merge-install --symlink-install "${COLCON_EXTRA_ARGS[@]}"
        fi
    else
        if [[ "$ROS_DISTRO" == "noetic" ]]; then
            print_header "[Using catkin build]"
            print_info "Building specific packages: $package_list"
            catkin build $package_list
        else
            print_header "[Using colcon build]"
            print_info "Building specific packages: $package_list"
            if [ "$USE_PACKAGE_LINKS" = true ]; then
                colcon build --merge-install --symlink-install --base-paths "$PACKAGE_SYMLINK_ROOT" "${COLCON_EXTRA_ARGS[@]}"
            else
                colcon build --merge-install --symlink-install --packages-select $package_list "${COLCON_EXTRA_ARGS[@]}"
            fi
        fi
    fi

    print_success "ROS build completed!"
}

# ========================
# Clean Functions
# ========================

clean_workspace() {
    local packages=("$@")

    print_header "[Cleaning Workspace]"

    # Show what will be cleaned
    print_info "The following will be cleaned:"
    if [ ${#packages[@]} -eq 0 ]; then
        echo "  - All package.xml symlinks in directory src/"
    else
        echo "  - Package.xml symlinks for: ${packages[*]}"
    fi
    echo "  - directory build/"
    echo "  - directory devel/"
    echo "  - directory install/"
    echo "  - directory log/"
    echo "  - directory logs/"
    echo "  - directory .catkin_tools/"
    echo "  - directory ${PACKAGE_SYMLINK_ROOT}/"

    # Ask for confirmation
    if [ ${#packages[@]} -eq 0 ]; then
        if ! ask_confirmation "Are you sure you want to clean ALL symlinks and build artifacts?"; then
            print_warning "Clean operation cancelled."
            exit 0
        fi
    else
        if ! ask_confirmation "Are you sure you want to clean symlinks for specified packages and build artifacts?"; then
            print_warning "Clean operation cancelled."
            exit 0
        fi
    fi

    # Remove package.xml symlinks
    if [ ${#packages[@]} -eq 0 ]; then
        print_info "Removing all package.xml symlinks..."
        find src -name "package.xml" -type l -delete
        rm -rf "$PACKAGE_SYMLINK_ROOT"
        print_success "Removed all symlinks"
    else
        print_info "Removing symlinks for specific packages..."
        for package_name in "${packages[@]}"; do
            package_dir=$(find src -name "$package_name" -type d | head -n 1)
            if [ -n "$package_dir" ]; then
                if [ -L "$package_dir/package.xml" ]; then
                    rm -f "$package_dir/package.xml"
                    print_success "Removed symlink from $package_name"
                else
                    print_warning "No symlink found for $package_name"
                fi
                if [ -L "$PACKAGE_SYMLINK_ROOT/$package_name" ]; then
                    rm -f "$PACKAGE_SYMLINK_ROOT/$package_name"
                fi
            else
                print_error "Package '$package_name' not found in src directory"
            fi
        done
    fi

    # Clean build artifacts
    print_info "Cleaning build artifacts..."
    rm -rf build/ devel/ install/ log/ logs/ .catkin_tools/ "$PACKAGE_SYMLINK_ROOT"

    print_success "Clean completed!"
}

clean_existing_symlinks() {
    local packages=("$@")

    print_header "[Cleaning Existing Symlinks]"

    rm -rf "$PACKAGE_SYMLINK_ROOT"

    if [ ${#packages[@]} -eq 0 ]; then
        print_info "Removing all existing package.xml symlinks..."
        find src -name "package.xml" -type l -delete
        print_success "Removed all existing symlinks"
    else
        print_info "Removing existing symlinks for specified packages..."
        removed_packages=()
        for package_name in "${packages[@]}"; do
            package_dir=$(find src -name "$package_name" -type d | head -n 1)
            if [ -n "$package_dir" ] && [ -L "$package_dir/package.xml" ]; then
                rm -f "$package_dir/package.xml"
                removed_packages+=("$package_name")
            fi
            if [ -L "$PACKAGE_SYMLINK_ROOT/$package_name" ]; then
                rm -f "$PACKAGE_SYMLINK_ROOT/$package_name"
            fi
        done

        if [ ${#removed_packages[@]} -gt 0 ]; then
            print_success "Removed existing symlinks from: ${removed_packages[*]}"
        else
            print_warning "No existing symlinks found"
        fi
    fi
}

# ========================
# ROS Specific Functions
# ========================

detect_incompatible_build_artifacts() {
    print_header "[Checking for Incompatible Build Artifacts]"

    local needs_cleanup=false

    # Check for ROS1 artifacts when using ROS2
    if [[ "$ROS_DISTRO" != "noetic" ]]; then
        if [ -d "devel" ] || [ -d ".catkin_tools" ]; then
            print_warning "Found ROS1 build artifacts (devel/ or .catkin_tools/) while using ROS2. Cleaning workspace..."
            needs_cleanup=true
        fi
    fi

    # Check for ROS2 artifacts when using ROS1
    if [[ "$ROS_DISTRO" == "noetic" ]]; then
        if [ -d "install" ] || [ -d "log" ]; then
            print_warning "Found ROS2 build artifacts (install/ or log/) while using ROS1. Cleaning workspace..."
            needs_cleanup=true
        fi
    fi

    if [ "$needs_cleanup" = true ]; then
        clean_workspace
    else
        print_success "No incompatible build artifacts found"
    fi
}

create_symlinks_for_package() {
    local package_dir="$1"
    local package_name=$(basename "$package_dir")
    local ros1_manifest="$package_dir/package.ros1.xml"
    local ros2_manifest="$package_dir/package.ros2.xml"

    if [ ! -d "$package_dir" ]; then
        return 1
    fi

    [ -e "$package_dir/package.xml" ] && rm -f "$package_dir/package.xml"

    if [[ "$ROS_DISTRO" == "noetic" ]]; then
        if [ -f "$ros1_manifest" ]; then
            ln -s package.ros1.xml "$package_dir/package.xml"
            return 0
        elif [ -f "$ros2_manifest" ]; then
            print_warning "Package $package_name does not provide package.ros1.xml, using ROS 2 manifest"
            ln -s package.ros2.xml "$package_dir/package.xml"
            return 0
        fi
    elif [[ "$ROS_DISTRO" == "foxy" || "$ROS_DISTRO" == "humble" ]]; then
        if [ -f "$ros2_manifest" ]; then
            ln -s package.ros2.xml "$package_dir/package.xml"
            return 0
        elif [ -f "$ros1_manifest" ]; then
            print_warning "Package $package_name does not provide package.ros2.xml, falling back to ROS 1 manifest"
            ln -s package.ros1.xml "$package_dir/package.xml"
            return 0
        fi
    else
        print_error "Unknown ROS version: $ROS_DISTRO"
        return 1
    fi

    return 1
}

prepare_package_workspace_symlinks() {
    local packages=("$@")

    if [ "$USE_PACKAGE_LINKS" = false ] || [ ${#packages[@]} -eq 0 ]; then
        return 0
    fi

    rm -rf "$PACKAGE_SYMLINK_ROOT"
    mkdir -p "$PACKAGE_SYMLINK_ROOT"

    local created_links=()
    for package_name in "${packages[@]}"; do
        local package_dir
        package_dir=$(find src -name "$package_name" -type d | head -n 1)
        if [ -z "$package_dir" ]; then
            print_warning "Package '$package_name' not found for workspace linking"
            continue
        fi

        local relative_target="../${package_dir#src/}"
        ln -sf "$relative_target" "$PACKAGE_SYMLINK_ROOT/$package_name"
        created_links+=("$package_name")
    done

    if [ ${#created_links[@]} -gt 0 ]; then
        print_success "Linked packages into minimal workspace: ${created_links[*]}"
    else
        print_warning "No packages linked into minimal workspace"
    fi
}

create_symlinks_for_all_packages() {
    print_header "[Creating Symlinks for All Packages]"

    created_packages=()
    declare -A package_dirs=()

    while IFS= read -r -d '' manifest; do
        package_dirs["$(dirname "$manifest")"]=1
    done < <(find src -name "package.ros1.xml" -print0)

    while IFS= read -r -d '' manifest; do
        package_dirs["$(dirname "$manifest")"]=1
    done < <(find src -name "package.ros2.xml" -print0)

    for package_dir in "${!package_dirs[@]}"; do
        package_name=$(basename "$package_dir")
        if create_symlinks_for_package "$package_dir"; then
            created_packages+=("$package_name")
        fi
    done

    if [ ${#created_packages[@]} -gt 0 ]; then
        print_success "Created symlinks for: ${created_packages[*]}"
    else
        print_warning "No packages with dual ROS support found"
    fi
}

create_symlinks_for_specific_packages() {
    local packages=("$@")

    print_header "[Creating Symlinks for Specific Packages]"
    print_info "Packages to process: ${packages[*]}"

    created_packages=()
    for package_name in "${packages[@]}"; do
        package_dir=$(find src -name "$package_name" -type d | head -n 1)
        if [ -n "$package_dir" ] && create_symlinks_for_package "$package_dir"; then
            created_packages+=("$package_name")
        fi
    done

    if [ ${#created_packages[@]} -gt 0 ]; then
        print_success "Created symlinks for: ${created_packages[*]}"
    fi
}

# ========================
# Main Script
# ========================

show_usage() {
    print_header "[Build System Usage]"
    print_header
    echo -e "Usage: $0 [OPTIONS] [PACKAGE_NAMES...]"
    echo ""
    echo -e "${COLOR_INFO}Options:${COLOR_RESET}"
    echo -e "  -c, --clean      Clean workspace (remove symlinks and build artifacts)"
    echo -e "  -m, --minimal    Build Titati hardware stack (CMake + minimal ROS packages)"
    echo -e "      --cmake      Build using CMake only (for hardware deployment)"
    echo -e "  -h, --help       Show this help message"
    echo ""
    echo -e "${COLOR_INFO}Examples:${COLOR_RESET}"
    echo -e "  $0                    # Build all ROS packages"
    echo -e "  $0 package1 package2  # Build specific ROS packages"
    echo -e "  $0 -c                 # Clean all symlinks and build artifacts"
    echo -e "  $0 --clean package1   # Clean specific package and build artifacts"
    echo -e "  $0 -m                 # Build Titati hardware stack (CMake + minimal ROS)"
    echo -e "  $0 --cmake            # Build with CMake for hardware deployment"
}

main() {
    local packages=()
    local clean_mode=false
    local cmake_mode=false
    local minimal_mode=false
    local cmake_only_mode=false

    local -a MINIMAL_TITATI_PACKAGES=(
        "titati_can_driver"
        "titati_system_interfaces"
        "titati_topics"
        "titati_power_services"
        "titati_canfd_gateway"
        "titati_motor_test"
        "rl_sar"
    )

    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -c|--clean) clean_mode=true; shift ;;
            -m|--minimal) minimal_mode=true; cmake_mode=true; shift ;;
            --cmake) cmake_mode=true; cmake_only_mode=true; shift ;;
            -h|--help) show_usage; exit 0 ;;
            --) shift; packages+=("$@"); break ;;
            -*) print_error "Unknown option: $1"; show_usage; exit 1 ;;
            *) packages+=("$1"); shift ;;
        esac
    done

    if [ "$minimal_mode" = true ]; then
        USE_PACKAGE_LINKS=true
        COLCON_EXTRA_ARGS+=(--cmake-args -DRL_SAR_HARDWARE_ONLY=ON)
        CMAKE_EXTRA_ARGS+=(-DRL_SAR_HARDWARE_ONLY=ON)

        local -a resolved_packages=()
        declare -A seen_packages=()

        add_minimal_package() {
            local pkg="$1"
            if [ -n "$pkg" ] && [ -z "${seen_packages[$pkg]}" ]; then
                resolved_packages+=("$pkg")
                seen_packages[$pkg]=1
            fi
        }

        for pkg in "${MINIMAL_TITATI_PACKAGES[@]}"; do
            add_minimal_package "$pkg"
        done

        for pkg in "${packages[@]}"; do
            add_minimal_package "$pkg"
        done

        packages=("${resolved_packages[@]}")
    fi

    # Handle CMake build mode
    if [ "$cmake_mode" = true ]; then
        run_cmake_build
        if [ "$minimal_mode" = false ] || [ "$cmake_only_mode" = true ]; then
            exit 0
        fi
    fi

    # Handle clean mode
    if [ "$clean_mode" = true ]; then
        clean_workspace "${packages[@]}"
        exit 0
    fi

    local should_run_ros=true

    if [ "$cmake_mode" = true ] && [ "$minimal_mode" = false ]; then
        should_run_ros=false
    fi

    if [ "$minimal_mode" = true ] && [ -z "$ROS_DISTRO" ]; then
        print_warning "ROS environment not detected. Skipping Titati ROS package build."
        should_run_ros=false
    fi

    if [ "$should_run_ros" = true ]; then
        if [ -z "$ROS_DISTRO" ]; then
            print_error "ROS environment not detected. Please source your ROS setup.bash first."
            print_info "For hardware deployment, use the --cmake option instead."
            exit 1
        fi

        run_ros_build "${packages[@]}"
    fi
}

main "$@"
