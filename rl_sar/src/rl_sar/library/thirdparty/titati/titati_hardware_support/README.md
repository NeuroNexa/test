<p align="center"><strong>titati_hardware_support</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>

## Description

This directory now keeps only the low-level SDK helpers needed by the RL stack:

- `titati_can_driver`: C++ bindings around the Titati CAN-FD SDK.
- `titati_power_services`: ROS 2 wrapper for the power handshake services/topics.

The ros2_control bridge (`hardware_bridge`) and bringup launch files have been removed from `rl_sar` because the RL controller talks to the robot directly.

## Prerequisites

- **Operating System**: Ubuntu 22.04
- **ROS 2**: Humble

## Build & Launch

These packages are built together with the rest of `rl_sar`. After compiling the workspace, start the power helpers with:

```bash
ros2 launch rl_sar titati_power_stack.launch.py
```

This launches `titati_power_services` and `titati_canfd_gateway`, preparing the CAN-FD bus for direct control through `titati_can_driver`.
