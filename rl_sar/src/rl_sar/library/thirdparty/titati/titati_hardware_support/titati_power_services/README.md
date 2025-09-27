<p align="center"><strong>titati_power_services</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>



## Description
本模块负责从从系统中读取电源数据，定义实现各类电源事件和接口，并以 ros2 的消息发布到系统中。

电池设备模块实现了`电池数据的话题`、`整机电源功耗的话题`和`电源管理命令的服务`，用户可以订阅电池信息话题完善自己程序的业务逻辑。

<u>**不建议**</u>用户对电源管理命令的服务做任何请求，因为电源管理命令的服务用于响应TITA机器人其他内部模块的请求，并非设计作为用户接口。


## Preparation

运行电池设备模块：
```
ros2 run titati_power_services titati_power_services_node
或者
ros2 launch titati_power_services titati_power_services_node.launch.py
```

打印电池设备数据：
```
ros2 topic echo /your_tita_name/system/batteries/left
或者
ros2 topic echo /your_tita_name/system/batteries/right
```

其中`your_tita_name`为具体的机器人设备命名，例如 `tita1234567`。

## Basic Information

| Installation method | Supported platform[s]    |
| ------------------- | ------------------------ |
| Source              | Jetpack 6.0 , ros-humble |

------

## Published

| ROS Topic |       Interface        | Frame ID | Description |
| :-------: | :--------------------: | :------: | :---------: |
| `../system/batteries/left`  | sensor_msgs::msg::BatteryState |  left_battery_info  |  左侧电池数据 5Hz  |
| `../system/batteries/right`  | sensor_msgs::msg::BatteryState |  right_battery_info  |  右侧电池数据 5Hz |
| `../system/battery_diagnostic/left`  | diagnostic_msgs::msg::DiagnosticArray |  left_battery_diagnostic_info  |  左侧电池诊断数据 5Hz |
| `../system/battery_diagnostic/right`  | diagnostic_msgs::msg::DiagnosticArray |  right_battery_diagnostic_info  |  右侧电池诊断数据 5hz |
| `../system/power_domain_info_diagnostic/power_domain_info`  | diagnostic_msgs::msg::DiagnosticArray |  power_domain_info  |  电源域诊断数据 30hz |

## Service

不建议用户对以下任何服务做请求，因为电源管理命令服务用于响应TITA机器人其他内部模块的请求，并非设计作为用户接口。

| Service Topic |     Call Interface     |    Return Interface    |       Description        |
| :-----------: | :--------------------: | :--------------------: | :----------------------: |
| `system/power_supply/power_state_set_srv`  | diagnostic_msgs::msg::DiagnosticArray | std_msgs::msg::Bool |    设置整机各级电平电压状态     |
| `system/power_supply/power_heart_beat_srv`  | diagnostic_msgs::msg::DiagnosticArray | std_msgs::msg::Bool |    发送一次电源管理系统心跳     |
| `system/power_supply/power_self_test_srv`  | diagnostic_msgs::msg::DiagnosticArray | std_msgs::msg::Bool |    发送一次电源管理系统自检请求     |




## Build Package

```bash
# If you haven't installed can-utils
# apt-get install can-utils
colcon build --merge-install --packages-select titati_power_services
```
