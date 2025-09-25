<p align="center"><strong>TITA-SDK-ROS2</strong></p>
<p align="center"><a href="https://github.com/DDTRobot/TITA-SDK-ROS2/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_cn/README_CN.md"><strong>中文</strong></a> / <a href="/README.md"><strong>English</strong>
</p>

​	TITA Ubuntu 系统的 SDK Demo.

## 基本信息

| Installation method | Supported platform[s]    |
| ------------------- | ------------------------ |
| Source              | Jetpack 6.0 , ros-humble |

------

## 发布的 ROS2 Topic

|       ROS Topic        |                   Interface                    | Frame ID |    Description    |
| :--------------------: | :--------------------------------------------: | :------: | :---------------: |
| `command/user/command` | `tita_locomotion_interfaces/msg/LocomotionCmd` |  `cmd`   | 用户 SDK 控制指令 |

## 构建包

```bash
mkdir -p tita_sdk/src
cd tita_sdk/src
git clone https://github.com/DDTRobot/TITA-SDK-ROS2.git
colcon build
source install/setup.bash
ros2 launch tita_bringup sdk_launch.py
```

## 配置 

|       Param       |      Range      | Default |                    Description                     |
| :---------------: | :-------------: | :-----: | :------------------------------------------------: |
|  `sdk_max_speed`  |      `3.0`      |  `3.0`  |              机器的速度上限，3.0 m/s                  |
| `turn_max_speed`  |      `6.0`      |  `6.0`  |              旋转速度上限，6.0 rad/s                  |
|  `pub_freq`       |  [100.0,170.0]  |  `170`  | 发布频率，单位 Hz，范围 [100.0,170.0]                  |    
| `height_max` | [0.0,0.3]     |  `0.3`  |  对应遥控器最高高度挡位，轮轴与车身中心的距离0.3 m                              |
| `height_min`     |  [0.0,0.3]          |  `0.2`  |    对应遥控器中间高度挡位，轮轴与车身中心的距离0.2m                         |
| `height_min` |   [0.0,0.3]   |  `0.1`  |    对应遥控器最低高度挡位，轮轴与车身中心的距离0.1 m                     |
|`pitch_max_pose`|   `1.0`  |  `1.0`  | 机器人最大俯仰角度，单位 rad，范围 [-1.0,1.0] |
## 快速开始

1. 依次按下遥控器小屏幕右侧的按键，直到出现 `mode select` 界面。
2. 将选择按钮向下拨到 **USE-SDK Mode** 并按下确认，机器人会自动执行，由 USE-SDK 流程接管控制权限，此时遥控器无法直接控制机器人。

## FAQ

1. 如果 `ros2 launch tita_bringup sdk_launch.py` 退出后，机器人仍会自动执行，除非释放 USE-SDK 控制权限。
2. 如果出现机器人没有响应，请检查 `sdk_command_node.cpp` 中的 `angular.z` 数值是否过小。
