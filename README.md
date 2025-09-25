# Titati 四轮足使用 rl_sar 的迁移说明

本仓库同时保留了原始的 `titati_control` 项目以及已经移植完成的 `rl_sar` 强化学习控制栈，方便在两套方案之间切换、对照调试。本文档梳理 Titati 机器人（两台 Tita 通过连线拼接成的四轮足）在 `rl_sar` 上线需要注意的软硬件准备、构建流程以及运行方法。

## 仓库结构速览

- `titati_control/`：原始 ROS2 控制器、CANFD 路由以及 bringup 脚本，可用于参考 URDF、参数或继续使用传统控制器。
- `rl_sar/`：强化学习仿真与真机部署框架，本次移植已经加入 Titati 的硬件接口（`tita_robot_sdk`）、状态机（`policy/titati`）以及真机入口程序 `rl_real_titati`。

建议把 `titati_control` 当作参考资料和紧急兜底方案，实际在线运行强化学习策略时全部工作都在 `rl_sar` 目录完成。

## 硬件与系统先决条件

1. **Jetson Orin NX (16G)**：确保已经刷入 Ubuntu 20.04/22.04。若完全在 `rl_sar` 内运行并通过键盘/手柄控制，可跳过 ROS 的安装。
2. **实时总线**：`tita_robot_sdk` 默认通过 `can0` 接口以 CAN FD 模式访问 16 个电机与 IMU，波特率 1 Mbps / 数据速率 8 Mbps。只需完成接口设置即可；可直接复用 `titati_control` 中的脚本初始化 CAN：
   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 up type can bitrate 1000000 sample-point 0.80 \
       dbitrate 8000000 dsample-point 0.80 fd on restart-ms 100
   sudo ifconfig can0 txqueuelen 1000
   ```
   或执行 `titati_control/src/can_setup_8m_master.sh` 一键设置（脚本仅负责网络参数，`rl_sar` 不再依赖其它 `titati_control` 组件）。
   对于运行 `rl_real_titati` 的主控 Jetson，`tita_robot_sdk` 内置的 CAN-FD 路由桥会在切换到 SDK 模式时自动监听路由心跳并下发
   `READY_WAITING/ FORCE_DIRECT` 序列，保证主控端直接接管全部 16 个电机。
   串联结构仍需要从机 Jetson 保持与路由器的握手，此时可使用新的守护程序 `rl_titati_router` 取代原来的 ROS 节点：

   ```bash
   cd rl_sar
   ./cmake_build/bin/rl_titati_router --interface can0
   ```

   该工具会自动在检测到主从模式变化或路由复位时重发握手指令，确保主从机之间的 CAN 通道持续打通。
3. **第三方依赖**：
   - [libtorch 2.0.1 cxx11 ABI](https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip)，并通过 `export Torch_DIR=<PATH>/libtorch` 指向其根目录。
   - `yaml-cpp` 与 `lcm`：`sudo apt install libyaml-cpp-dev liblcm-dev`。
   - 可选：若需要 ROS 键盘/遥控输入，请安装 `teleop-twist-keyboard`、`ros_control` 等依赖，具体列表见 `rl_sar/README.md`。

在启动 `rl_real_titati` 前务必确认 `can0` 已经 up，且旧的 `titati_controller` 节点不会同时抢占电机。

## 构建 rl_sar（含 Titati 支持）

> 建议在机器人本机或开发主机上完成以下步骤，默认从仓库根目录执行。

1. **配置 Torch 路径**（仅首次）
   ```bash
   echo 'export Torch_DIR=/opt/libtorch' >> ~/.bashrc
   source ~/.bashrc
   ```
2. **进入 rl_sar 并执行硬件构建**：
   ```bash
   cd rl_sar
   ./build.sh --cmake rl_sar
   ```
   该命令会以纯 CMake 方式生成 `cmake_build/`，包含 `tita_robot_sdk` 与 `rl_real_titati` 可执行文件。
3. **ROS 工作区构建（可选）**：若需要通过 `ros2 run rl_sar rl_real_titati` 启动，可执行 `./build.sh --ros rl_sar`，脚本会自动创建符号链接并调用 `colcon`/`catkin` 构建。

构建成功后，`cmake_build/bin/rl_real_titati` 即可直接运行，无需 ROS 环境。

## 策略、模型与参数配置

Titati 的策略配置集中在 `rl_sar/src/rl_sar/policy/titati/`：

- `base.yaml`：对齐物理硬件的默认关节姿态、力矩上限、轮子索引以及 `joint_mapping`。如硬件布线顺序发生变化，请同步修改 `joint_mapping`，否则状态和命令会错位。
- `base.yaml` 同时包含 `can_interface` 与 `use_canfd_router` 两个开关：前者指定使用的 Linux CAN 口，后者控制是否监听 CAN-FD 路由器心跳并自动打通主从机通讯（默认对串联的四轮足开启）。
- `robot_lab/config.yaml`：描述策略输入输出维度、观测项以及增益。可将训练得到的 TorchScript 模型命名为 `policy.pt`（或在 `model_name` 字段中填写自定义文件名），并放置到 `rl_sar/src/rl_sar/policy/titati/robot_lab/` 目录下。
- `fsm.hpp`：定义 Titati 的有限状态机，包括被动、起身、下蹲与 RL Locomotion 等状态，通常无须修改，但若需要自定义前置动作可在此扩展。

当 `rl_real_titati` 启动后会读取上述 YAML 并自动加载模型、初始化观测缓冲与控制参数。

## 运行 rl_real_titati

### 纯 CMake 可执行文件

```bash
cd rl_sar
./cmake_build/bin/rl_real_titati
```

程序启动后会自动尝试将电机切换至 SDK 直驱模式；退出（或 `Ctrl+C`）时会回落到 MCU 控制，以避免无人值守状态下电机保持使能。

### ROS2 启动（可选）

```bash
source install/setup.bash   # 或 source devel/setup.bash (ROS1)
ros2 run rl_sar rl_real_titati
```

在定义了 `USE_ROS` 的编译配置下，节点会订阅 `/cmd_vel` 并根据导航模式决定使用 ROS 命令或键盘命令。

### 键盘 / 手柄指令速查

- `W/S`：前后速度增减；`A/D`：侧向速度增减；`Q/E`：航向角速度增减；`Space`：清空所有速度命令。
- `M`：强制切换到 SDK 直驱，使能电机；`K`：切回 MCU 控制，关闭电机；`N`：切换导航模式（键盘 VS `/cmd_vel`）。
- 当接入 Retroid 手柄时，`LB+RB` 会触发 `set_robot_stop()` 急停，`LB+A/B/X` 与键盘热键一致。

策略推理线程会将 IMU、关节位置/速度/力矩写入 `RobotState`，经过状态机输出 MIT 力矩命令，并通过 `tita_robot_sdk` 下发到各电机。

## 与 titati_control 共存的注意事项

> 若已完全迁移到 `rl_sar` 并不再运行旧栈，可忽略本节。

- 两套系统都会占用 `can0` 与电机 SDK 通道，请确保同一时间只启动其中一个，否则会互相抢占导致力矩指令异常。
- 如需回退到原控制器，可先退出 `rl_real_titati`，执行 `titati_control/src/can_setup_8m_master.sh` 重新拉起原 ROS2 launch。
- 通过 `tita_robot::set_motors_sdk(false)` 可以在不退出程序的情况下让 MCU 接管，以便切换到 `titati_control` 的节点。

## 调试与排错建议

- 若 `rl_real_titati` 启动报错 `TORCH` 未找到，请检查 `Torch_DIR` 环境变量是否指向正确目录。
- 若 CAN 报错 `Interface does not support CAN FD`，请确认网卡驱动支持 CAN FD，并按脚本重新设置。
- 可通过定义 `PLOT` 或 `CSV_LOGGER`（`rl_real_titati.hpp` 顶部）开启实时曲线或数据记录，便于排查策略输出与硬件反馈的差异。
- 若怀疑主从机握手异常，可观察主控上的 `rl_real_titati` 与从控上的 `rl_titati_router` 日志：程序会在检测到路由模式变化或重发
  `FORCE_DIRECT` 时打印提示，便于确认 CAN-FD 路由是否保持连通。

按照以上流程完成准备后，即可使用 `rl_sar` 中的强化学习策略对 Titati 四轮足进行实时控制，实现从 `titati_control` 向 RL 框架的平滑迁移。
