<p align="center"><strong>titati_hardware_support</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>

## Description

Hardware support helpers for the Titati robot. The repository contains two ROS 2 packages:

- `titati_can_driver`: C++ bindings around the Titati CAN-FD SDK.
- `titati_power_services`: ROS 2 nodes that expose the robot power-handshake services and status topics.

These helpers are used by the reinforcement learning stack to communicate with the robot without the legacy ros2_control bridge.

## Prerequisites

- **Operating System**: Ubuntu 22.04
- **ROS 2**: Humble

## Build

Clone the repository into a ROS 2 workspace and build it with `colcon`:

```bash
mkdir -p titati_ws/src && cd titati_ws/src
git clone https://github.com/DDTRobot/TITA_Description.git
# Clone this repository next to the rest of the stack
colcon build --packages-up-to titati_power_services titati_can_driver
source install/setup.bash
```

## Launch

Launch the power stack to initialise the CAN-FD bus:

```bash
ros2 launch rl_sar titati_power_stack.launch.py
```

This starts `titati_power_services` together with the CAN-FD gateway. Once they are running you can interact with the robot directly through `titati_can_driver` or higher level RL controllers.
