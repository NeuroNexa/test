# Titati 四轮足：从 `titati_control` 迁移到 `rl_sar`

本文记录如何在无需 ROS 的前提下，将由两台 Tita 拼接的四轮足机器人切换到 `rl_sar` 强化学习控制栈。请在动手前完整阅读“主从运行顺序”小节，
确保两台 Jetson（主控、从控）和 16 个电机能够在同一条 CAN-FD 总线上协同工作。

---

## 1. 系统概览

- **机器人拓扑**：四轮足由两台 Tita 通过连接盒串接，整体共有 16 个电机、2 组 IMU，各自由一台 Jetson Orin NX 16G 控制。
- **主控 Jetson**：运行 `rl_real_titati`，直接与 16 个电机通信并执行强化学习策略。
- **从控 Jetson**：保持 CAN-FD 路由心跳，使连接盒持续处于直通模式。`rl_sar` 提供了 `rl_titati_router` 守护程序完成此任务。
- **仓库结构**：
  - `titati_control/`：旧 ROS2 控制栈、URDF、CAN 初始化脚本，仅作为参考或回退方案。
  - `rl_sar/`：强化学习主仓库，本次迁移已经集成 Titati 的硬件驱动（`tita_robot_sdk`）、策略配置与真机程序。

在 `rl_sar` 下调试/运行时，无需再启动 `titati_control` 的节点，但仍可复用其中的工具脚本（如 CAN 设置）。

---

## 2. 先决条件

### 2.1 操作系统与工具链

| 组件 | 要求 |
| --- | --- |
| 系统 | Ubuntu 20.04 或 22.04（Jetson 官方系统即可） |
| 编译工具 | CMake ≥ 3.18、gcc ≥ 9（Jetson 默认满足） |
| Python | 3.8–3.10，用于 TorchScript 运行时 |

### 2.2 第三方依赖

在**两台 Jetson**上均需要准备：

1. **libtorch**：建议使用 `libtorch 2.0.1 cxx11 ABI`，下载后设置环境变量：
   ```bash
   echo 'export Torch_DIR=/opt/libtorch' >> ~/.bashrc
   source ~/.bashrc
   ```
2. **系统库**：
   ```bash
   sudo apt install libyaml-cpp-dev liblcm-dev
   ```
3. **可选 ROS 依赖**：若需要在 ROS2 中启动，可再安装 `ros-<distro>-teleop-twist-keyboard` 等工具。

### 2.3 CAN-FD 总线配置

`tita_robot_sdk` 默认使用 `can0`，速率 1 Mbps / 8 Mbps。可执行旧项目里的脚本完成设置：

```bash
sudo bash titati_control/src/can_setup_8m_master.sh
```

或手动运行：

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.80 \
    dbitrate 8000000 dsample-point 0.80 fd on restart-ms 100
sudo ifconfig can0 txqueuelen 1000
```

确保命令在**主控与从控**两侧都执行一次，使两条 CAN 链路都处于 CAN-FD 直通状态。

---

## 3. 构建 `rl_sar`

以下步骤默认在仓库根目录执行。

1. **进入子仓库**：
   ```bash
   cd rl_sar
   ```
2. **硬件部署构建（推荐）**：
   ```bash
   ./build.sh --cmake rl_sar
   ```
   生成的 `cmake_build/` 将包含 `rl_real_titati`、`rl_titati_router`、`rl_titati_motor_test` 等可执行文件。
   - CMake 会在未发现 Unitree/Lite3 依赖时自动启用 `BUILD_TITATI_ONLY=ON`，仅编译 Titati 所需目标。
   - 若未来需要同时构建其它机器人的可执行文件，可手动传入 `-DBUILD_TITATI_ONLY=OFF` 并确保相关 SDK 就绪。
   - 可执行文件会在运行时自动解析 `rl_sar/src/rl_sar/policy`，无需切换工作目录；如需自定义配置路径，可使用 `--config` 或设置 `RL_SAR_CONFIG_DIR`。
3. **ROS 工作区（可选）**：如需 `ros2 run rl_sar rl_real_titati`，执行：
   ```bash
   ./build.sh --ros rl_sar
   ```
   脚本会自动调用 `colcon`/`catkin` 进行额外构建。

> 提示：首次构建可能因 Torch 没有找到而失败，请确认 `Torch_DIR` 已正确导出。

---

## 4. 策略与配置文件

相关配置位于 `rl_sar/src/rl_sar/policy/titati/`：

- **`base.yaml`**：硬件相关参数，包括 `joint_mapping`、关节限幅、`can_interface` 以及 `use_canfd_router`。若布线顺序有变，务必同步 `joint_mapping`。
- **`robot_lab/config.yaml`**：策略输入输出维度、观测项定义与默认模型名。训练好的 TorchScript 模型（如 `policy.pt`）需置于同目录。
- **`fsm.hpp`**：Titati 的有限状态机定义，一般只在自定义起步/收脚动作时修改。

`rl_real_titati` 启动时会读取以上配置，初始化观测缓存并加载策略模型。

---

## 5. 主从运行顺序

为了保证 CAN-FD 路由始终保持直通，请严格遵循以下流程：

1. **从控 Jetson**：
   ```bash
   cd rl_sar
   ./cmake_build/bin/rl_titati_router --interface can0
   ```
   - 该守护程序会持续监听路由心跳，一旦检测到模式回落会立即重发 `READY_WAITING/ FORCE_DIRECT` 序列。
   - 程序输出会记录当前路由模式及是否成功切换到 FORCE_DIRECT，稍后主控上线时应看到 `Router entered FORCE_DIRECT mode.` 的提示；若迟迟未出现，请核对 CAN 线束与速率设置。

2. **主控 Jetson**（确保从控已在运行）：
   ```bash
   cd rl_sar
   ./cmake_build/bin/rl_real_titati
   ```
   - 启动阶段会自动发送零力矩并将 MCU 切换到 SDK 直驱模式。
   - 一旦退出（`Ctrl+C`），程序会重新下发 `AUTO_LOCOMOTION`，确保 MCU 重新接管。

3. **再次启动**：直接重复第 2 步即可，程序会检测到路由仍在直通并重发握手以确认主控拥有全部 16 个电机的控制权。

> 若需要回退到 `titati_control`：先退出 `rl_real_titati`，待终端打印“MCU control restored”后，再启动原 ROS2 launch。

---

## 6. 操作接口

- **键盘**：`W/S` 控制前后速度，`A/D` 控制横移，`Q/E` 控制偏航；`Space` 清零命令。
- **模式切换**：`M` 进入 SDK 直驱、`K` 回落 MCU；`N` 切换键盘/ROS 输入源。
- **遥控手柄**：`LB+RB` 急停，`LB+A/B/X` 功能等同于键盘热键。

`rl_real_titati` 会循环读取关节位置/速度/力矩与 IMU 数据，经过 FSM 与策略后通过 MIT 控制下发到电机。

---

## 7. 电机连通性与安全测试

在加载强化学习策略前，建议先通过下列测试确认 16 个电机均能响应：

```bash
cd rl_sar

# MIT：对每个关节施加小幅度位置扰动，验证编码器读取与 MIT 参数映射
./cmake_build/bin/rl_titati_motor_test --mode mit --amplitude 0.05 --hold 0.5

# 力矩：对每个关节输出短脉冲，检查是否存在饱和或正反向接线问题
./cmake_build/bin/rl_titati_motor_test --mode torque --amplitude 2.0 --hold 0.3
```

程序会按 `joint_mapping` 顺序依次测试每个电机，并打印实时的位置、速度、力矩与电机状态。当检测不到 CAN-FD 路由时会提示警告；退出或测试完成后自动切回 MCU 控制。默认配置会自动从源码目录读取 `policy/titati/base.yaml`，无需修改当前工作目录；如需调试其它参数，可显式传入 `--config` 或设置 `RL_SAR_CONFIG_DIR=/path/to/policy`。

---

## 8. 常见问题

| 现象 | 排查步骤 |
| --- | --- |
| `Torch package not found` | 检查 `Torch_DIR` 是否指向 libtorch 根目录，或确认 `./build.sh --cmake rl_sar` 的日志中已找到 Torch。 |
| `Interface does not support CAN FD` | 核对内核与驱动是否支持 CAN-FD，重新执行 CAN 配置脚本，确认 `ip -details link show can0` 中 `fd` 字段为 `on`。 |
| 主控无法接管全部电机 | 确认从控上的 `rl_titati_router` 正在运行，查看其日志是否持续打印 `FORCE_DIRECT`; 同时检查 `rl_real_titati` 是否成功调用 `set_motors_sdk(true)`。 |
| 需要回退到旧栈 | 在主控执行 `K` 或直接退出 `rl_real_titati`，待提示“MCU 接管”后，再启动 `titati_control` 的 launch。 |

---

完成以上步骤后，便可在主控 Jetson 上运行 `rl_real_titati`，加载强化学习策略并控制四轮足机器人，实现从 `titati_control` 到 `rl_sar` 的平滑迁移。

