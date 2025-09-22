# rl_sar

[![Ubuntu 20.04/22.04](https://img.shields.io/badge/Ubuntu-20.04/22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)
[![ROS Noetic](https://img.shields.io/badge/ros-noetic-brightgreen.svg?logo=ros)](https://wiki.ros.org/noetic)
[![ROS2 Foxy/Humble](https://img.shields.io/badge/ros2-foxy/humble-brightgreen.svg?logo=ros)](https://wiki.ros.org/foxy)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg?logo=apache)](https://opensource.org/license/apache-2-0)

[English document](README.md)

本仓库提供了机器人强化学习算法的仿真验证与实物部署框架，适配四足机器人、轮足机器人、人形机器人。"sar"代表"simulation and real"

> 支持**IsaacGym**和**IsaacSim**
>
> 支持**ROS-Noetic**和**ROS2-Foxy/Humble**

支持列表：

|Robot Name (rname:=)|Pre-Trained Policy|Real|
|-|-|-|
|Unitree-A1 (a1)|legged_gym (IsaacGym)|✅|
|Unitree-Go2 (go2)|himloco (IsaacGym)</br>robot_lab (IsaacSim)|✅</br>✅|
|Unitree-Go2W (go2w)|robot_lab (IsaacSim)|✅|
|Unitree-B2 (b2)|robot_lab (IsaacSim)|⚪|
|Unitree-B2W (b2w)|robot_lab (IsaacSim)|⚪|
|Unitree-G1 (g1)|unitree_rl_gym (IsaacGym)</br>robomimic pre-loco (IsaacGym)</br>robomimic_dance (IsaacGym)</br>robomimic_kick (IsaacGym)</br>robomimic_kungfu (IsaacGym)|✅</br>✅</br>✅</br>🚫</br>🚫|
|FFTAI-GR1T1 (gr1t1)</br>(Only available on Ubuntu20.04)|legged_gym (IsaacGym)|⚪|
|FFTAI-GR1T2 (gr1t2)</br>(Only available on Ubuntu20.04)|legged_gym (IsaacGym)|⚪|
|GoldenRetriever-L4W4 (l4w4)|legged_gym (IsaacGym)</br>robot_lab (IsaacSim)|✅</br>✅|
|Deeprobotics-Lite3 (lite3)|himloco (IsaacGym)|✅|
|DDTRobot-Tita (tita)|robot_lab (IsaacSim)|⚪|

> [!IMPORTANT]
> Python版本暂时停止维护，如有需要请使用[v2.3](https://github.com/fan-ziqi/rl_sar/releases/tag/v2.3)版本，后续可能会重新上线。

> [!NOTE]
> 如果你想使用IsaacLab（IsaacSim）训练策略，请使用 [robot_lab](https://github.com/fan-ziqi/robot_lab) 项目。
>
> robot_lab配置文件中的关节顺序 `joint_names` 与本项目代码中 `xxx/robot_lab/config.yaml` 中定义的相同。
>
> 在 [Github Discussion](https://github.com/fan-ziqi/rl_sar/discussions) 或 [Discord](https://www.robotsfan.com/dc_rl_sar) 中讨论

> [!CAUTION]
> **免责声明：使用者确认使用本代码产生的所有风险及后果均由使用者自行承担，作者不承担任何直接或间接责任，操作前必须确保已采取充分安全防护措施。**

## 准备

拉取代码

```bash
git clone https://github.com/fan-ziqi/rl_sar.git
```

## 依赖

如果您使用`ros-noetic`(Ubuntu20.04)，需要安装以下的ros依赖包：

```bash
sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-controller-interface ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller ros-noetic-joy ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-controller-manager
```

如果您使用`ros2-foxy`(Ubuntu20.04)或`ros2-humble`(Ubuntu22.04)，需要安装以下的ros依赖包：

```bash
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-gazebo-ros2-control ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-xacro
```

在任意位置下载并部署`libtorch`（请修改下面的 **\<YOUR_PATH\>** 为实际路径）

```bash
cd <YOUR_PATH>
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./
echo 'export Torch_DIR=<YOUR_PATH>/libtorch' >> ~/.bashrc
source ~/.bashrc
```

安装`yaml-cpp`和`lcm`，若您使用Ubuntu，可以直接使用包管理器进行安装

```bash
sudo apt install liblcm-dev libyaml-cpp-dev
```

<details>

<summary>也可使用源码安装（点击展开）</summary>

安装yaml-cpp

```bash
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp && mkdir build && cd build
cmake -DYAML_BUILD_SHARED_LIBS=on .. && make
sudo make install
sudo ldconfig
```

安装lcm

```bash
git clone https://github.com/lcm-proj/lcm.git
cd lcm && mkdir build && cd build
cmake .. && make
sudo make install
sudo ldconfig
```
</details>

## 编译

由于本项目支持多版本的ROS，需要针对不同版本创建一些软链接，项目根目录中提供了编译脚本供一键编译。

在项目根目录中执行下面的脚本编译整个项目

```bash
./build.sh
```

若想单独编译某几个包，可以在后面加上包名

```bash
./build.sh package1 package2
```

若想删除构建，可以使用下列命令，此命令会删除所有编译产物和创建的软链接

```bash
./build.sh -c  # or ./build.sh --clean
```

如果不需要仿真，只在机器人上运行，可使用硬件预设：当已 `source` ROS 工作空间时会通过 catkin/colcon 构建 `rl_sar` 并跳过 Gazebo 相关目标；若未检测到 ROS 环境则自动回退到独立的 CMake 工具链，并在 `cmake_build/bin`（库位于 `cmake_build/lib`）生成可执行文件：

```bash
./build.sh -m  # or ./build.sh --cmake
```

> [!TIP]
> 若未提前 source 任意 ROS 环境，直接执行 `./build.sh` 会自动切换到同样的硬件构建流程，因此从机 Jetson 只需克隆仓库并执行 `./build.sh` 即可得到二进制文件。

详细的使用说明可以通过`./build.sh -h`查看

```bash
Usage: ./build.sh [OPTIONS] [PACKAGE_NAMES...]

Options:
  -c, --clean    Clean workspace (remove symlinks and build artifacts)
  -m, --cmake    Build the Titati hardware stack (uses ROS when available)
  -h, --help     Show this help message

Examples:
  ./build.sh                    # Build all ROS packages
  ./build.sh package1 package2  # Build specific ROS packages
  ./build.sh -c                 # Clean all symlinks and build artifacts
  ./build.sh --clean package1   # Clean specific package and build artifacts
  ./build.sh -m                 # Build the Titati hardware stack
```

> [!TIP]
> 如果 catkin build 报错: `Unable to find either executable 'empy' or Python module 'em'`, 在`catkin build` 之前执行 `catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3`

## 运行

下文中使用 **\<ROBOT\>/\<CONFIG\>** 代替表示不同的环境，如 `go2/himloco` 、 `go2w/robot_lab`。

运行前请将训练好的pt模型文件拷贝到`rl_sar/src/rl_sar/policy/<ROBOT>/<CONFIG>`中，并配置`<ROBOT>/<CONFIG>/config.yaml`和`<ROBOT>/base.yaml`中的参数。

### 仿真

打开一个终端，启动gazebo仿真环境

```bash
# ROS1
source devel/setup.bash
roslaunch rl_sar gazebo.launch rname:=<ROBOT>

# ROS2
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=<ROBOT>
```

打开一个新终端，启动控制程序

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_sim

# ROS2
source install/setup.bash
ros2 run rl_sar rl_sim
```

> [!TIP]
> Ubuntu22.04中若启动Gazebo后看不到机器人，则是机器人初始化到了视野范围外，启动rl_sim后会自动重置机器人位置。若机器人在站立过程中翻倒，请使用键盘`R`或手柄`RB+Y`重置机器人环境。

如果第一次启动Gazebo无法打开则需要下载模型包

```bash
git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models
```

### 手柄与键盘控制

|手柄控制|键盘控制|功能描述|
|---|---|---|
|**基础**|||
|A|Num0|让机器人从程序开始运行时的姿态以位控插值运动到`base.yaml`中定义的`default_dof_pos`|
|B|Num9|让机器人从当前位置以位控插值运动到程序开始运行时的姿态|
|X|N|切换导航模式 (导航模式屏蔽速度命令，接收`cmd_vel`话题)|
|Y|N/A|N/A|
|**仿真**|||
|RB+Y|R|重置Gazebo环境 (让摔倒的机器人站起来)|
|RB+X|Enter|切换Gazebo运行/停止 (默认为运行状态)|
|**电机**|||
|LB+A|M|N/A (推荐设置为电机使能)|
|LB+B|K|N/A (推荐设置为电机失能)|
|LB+X|P|电机Passive模式 (`kp=0, kd=8`)|
|LB+RB|N/A|N/A (推荐设置为急停保护)|
|**技能**|||
|RB+DPadUp|Num1|基础Locomotion|
|RB+DPadDown|Num2|技能2|
|RB+DPadLeft|Num3|技能3|
|RB+DPadRight|Num4|技能4|
|LB+DPadUp|Num5|技能5|
|LB+DPadDown|Num6|技能6|
|LB+DPadLeft|Num7|技能7|
|LB+DPadRight|Num8|技能8|
|**移动**|||
|LY轴|W/S|前后移动 (X轴)|
|LX轴|A/D|左右移动 (Y轴)|
|RX轴|Q/E|偏航旋转 (Yaw)|
|N/A(松开摇杆)|Space|将所有控制指令设置为零|

### 真实机器人

> [!IMPORTANT]
> 硬件预设仅构建 Titati 所需的组件（CAN-FD 路由器、16 电机诊断程序和 `rl_real_titati`）。Unitree、Lite3 等驱动已从本工程移除，以下小节仅保留历史资料供参考。

<details>

<summary>Unitree A1（点击展开）</summary>

与Unitree A1连接可以使用无线与有线两种方式

- 无线：连接机器人发出的Unitree开头的WIFI **（注意：无线连接可能会出现丢包断联甚至失控，请注意安全）**
- 有线：用网线连接计算机和机器人的任意网口，配置计算机地址为192.168.123.162，子网掩码255.255.255.0

新建终端，启动控制程序

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_a1

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_a1

# CMake
./cmake_build/bin/rl_real_a1
```

</details>

<details>

<summary>Unitree Go2/Go2W/G1(29dofs)（点击展开）</summary>

#### 网线连接

用网线的一端连接Go2/Go2W/G1(29dofs)机器人，另一端连接你的电脑，并开启电脑的 USB Ethernet 后进行配置。机器狗机载电脑的 IP 地地址为 `192.168.123.161`，故需将电脑 USB Ethernet 地址设置为与机器狗同一网段，如在 Address 中输入 `192.168.123.222` (`222`可以改成其他)。

通过`ifconfig`命令查看123网段的网卡名字，如`enxf8e43b808e06`，下文用 \<YOUR_NETWORK_INTERFACE\> 代替

Go2:

新建终端，启动控制程序。如果控制Go2W，需要在命令后加`wheel`，否则留空。

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]

# CMake
./cmake_build/bin/rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]
```

G1(29dofs):

开机后将机器人吊起来，按L2+R2进入调试模式，然后新建终端，启动控制程序。

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_g1 <YOUR_NETWORK_INTERFACE>

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_g1 <YOUR_NETWORK_INTERFACE>

# CMake
./cmake_build/bin/rl_real_g1 <YOUR_NETWORK_INTERFACE>
```

#### 在机载Jetson中部署

使用网线连接电脑和机器狗，登陆Jetson主机，密码123：

```bash
ssh unitree@192.168.123.18
```

查看jetpack版本

```bash
sudo pip install jetson-stats
sudo jtop
```

下载并安装对应jetpack版本的pytorch

```bash
wget https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
sudo apt install python-is-python3 python3.9-dev
pip install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
```

查看torch路径

```bash
python -c "import torch; print(torch.__file__)"
```

自行修改下面路径，手动创建libtorch库

```bash
mkdir ~/libtorch
cp -r /home/unitree/.local/lib/python3.8/site-packages/torch/{bin,include,lib,share} ~/libtorch
echo 'export Torch_DIR=~/libtorch' >> ~/.bashrc
source ~/.bashrc
```

拉取代码并编译，流程与上文相同。

</details>

<details>

<summary>DDTRobot Titati（16 自由度双机）</summary>

#### 硬件连线与模式切换

- 两台 Jetson（主/从）分别将 `can0` 配置为 **CAN-FD 1M/8M**。使用本仓库提供的脚本即可：

  ```bash
  sudo bash rl_sar/src/rl_sar/scripts/setup_titati_canfd.sh can0
  ```

- 使用“小盒子”或 CAN-HUB 将两台 `can0` 物理连接到同一条 CAN-FD 总线上。
- 从机在 MCU 自检完成后执行 CAN 直通节点。启动脚本会自动判断是否存在 ROS 环境：

  ```bash
  # 独立硬件部署（./build.sh -m 或自动降级）
  ./src/rl_sar/scripts/run_titati_canfd_router.sh

  # 若在 ROS 工作空间内构建
  source install/setup.bash        # ROS 2
  # 或：source devel/setup.bash    # ROS 1
  rl_sar/src/rl_sar/scripts/run_titati_canfd_router.sh
  ```

  如果检测不到 ROS 命令或未安装 `rl_sar` 包，脚本会自动回退到 `cmake_build/bin/titati_canfd_router_node`。该节点会持续监听控制板心跳，在检测到板载模式不是直通时自动发送 `SET_READY_NEXT -> FORCE_DIRECT` 的 CAN RPC。路由器进入直通模式后，主机需要同时向两块电机 MCU 广播同样的 RPC 帧；本仓库内置的 Titati SDK 会在 16 自由度配置下自动完成该步骤，确保 **0-7 号关节（主 MCU）** 与 **8-15 号关节（从 MCU）** 一起切换到直驱模式。
- 主机侧运行本仓库提供的程序：在完全直通的 CAN-FD 网络上直接输出 16 个电机的力矩指令（PD 增益来自 RL 策略或单独的诊断程序）。

#### 电机连通性自检

编译完成后，先运行测试程序确认每个电机都能响应。新的诊断程序以“恒定力矩 / 恒定速度”的方式推动关节，操作更加直观：

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar test_titati_motors --joint 3 --mode torque --value 5 --duration 1

# ROS2
source install/setup.bash
ros2 run rl_sar test_titati_motors --joint 12 --mode velocity --value 2.0 --duration 2 --velocity-kd 1.2

# CMake
./cmake_build/bin/test_titati_motors --all --mode torque --value 3 --duration 1.5 --rest 0.5
```

常用参数说明：

- `--joint <index>`：指定需要测试的关节（0-15，可重复）；若不传或使用 `--all`，将按顺序遍历全部电机。
- `--mode torque|velocity`：选择恒定力矩（牛·米）或恒定角速度（弧度/秒）测试模式。
- `--value <number>`：施加到目标关节的力矩或角速度大小，可为正或负。
- `--duration <sec>` / `--rest <sec>`：分别表示持续施加指令的时间，以及施加结束后回到零力矩的休息时间。
- `--velocity-kd <gain>`：速度模式下使用的速度闭环增益（MIT 控制中的 KD），力矩模式会忽略该参数。
- `--log-interval <sec>`：终端打印测量位置/速度/力矩的时间间隔。

程序会循环发送强制直驱 RPC，等待 MCU 反馈就绪后再施加命令，并在执行过程中持续打印回读数据。在给电机下指令前会先列出已经收到遥测数据（带时间戳）的关节编号，确保 **16 个电机** 都在线。如果提示 `0-7` 号关节缺失，说明主机侧 MCU 仍处于自动模式；如果缺失 `8-15` 号关节，则表示从机（经 CAN-FD 路由）尚未放行。程序会自动向两块板子重发直驱 RPC，待路由或 MCU 就绪后即可立即复测。

#### 运行 RL 控制

1. 将策略文件放置在 `rl_sar/src/rl_sar/policy/titati/robot_lab/policy.pt`（或修改 `config.yaml` 指向的路径）。
2. 在主机 Jetson（或上位机）上启动控制程序：

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_titati

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_titati

# CMake
./cmake_build/bin/rl_real_titati
```

3. 键盘默认快捷键与其他机器人一致：`Num0` 起身、`Num1` 切换 RL 行走、`Num9` 俯身、`P` 回到被动模式、`WASD/QE` 调整速度，`N` 切换导航模式（ROS 下读取 `/cmd_vel`）。

</details>

<details>

<summary>云深处科技 Lite3 (Click to expand)</summary>

Lite3通过无线网络进行连接。
(由于一些型号的Lite3没有开放网线接口，需要额外安装，所以有线连接方式暂时没有进行测试)

- 连接Lite3的Wifi，并测试通信状况。我们强烈建议在运行本项目之前，先通过 [Lite3_Motion_SDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK)进行测试和检查，在确认一切正常后再运行。
 **(注意：无线连接可能会出现丢包断联甚至失控，请注意安全)**

- 确认所使用Lite3的IP地址和本地端口与目标端口号码，并设置 **在 rl_sar/src/rl_real_lite3.cpp的行46-48**中.
- 在Lite3的运动主机中设置 **jy_exe/conf/network.toml**，使其IP地址指向与Lite3同一网段的本机，建立基于UDP的双向通信.

> [!CAUTION]
> **检查关节映射参数<br>检查确认 rl_sar/policy/himloco/config.yaml中的joint mappng参数。在Sim2Sim中使用的默认joint mapping参数与实机部署时的joint mapping是不同的，如果使用错误可能造成机器人错误的行为，带来潜在的硬件损坏和安全风险。**

Lite3也支持使用云深处Retroid手柄控制，详情参见[Deeprobotics Gamepad](https://github.com/DeepRoboticsLab/gamepad)

新建终端，启动控制程序

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_lite3

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_lite3

# CMake
./cmake_build/bin/rl_real_lite3
```

</details>

### 训练执行器网络

下面拿A1举例

1. 取消注释`rl_real_a1.hpp`中最上面的`#define CSV_LOGGER`，你也可以在仿真程序中修改对应部分采集仿真数据用来测试训练过程。
2. 运行控制程序，程序会记录所有数据到`src/rl_sar/policy/<ROBOT>/motor.csv`。
3. 停止控制程序，开始训练执行器网络。注意，下面的路径前均省略了`rl_sar/src/rl_sar/policy/`。
    ```bash
    rosrun rl_sar actuator_net.py --mode train --data a1/motor.csv --output a1/motor.pt
    ```
4. 验证已经训练好的训练执行器网络。
    ```bash
    rosrun rl_sar actuator_net.py --mode play --data a1/motor.csv --output a1/motor.pt
    ```

## 添加你的机器人

下面使用 **\<ROBOT\>/\<CONFIG\>** 代替表示你的机器人环境，且路径均在`rl_sar/src/`下。您只需要创建或修改下述文件，命名必须跟下面一样。（你可以参考go2w对应的文件）

```yaml
# 你的机器人description
robots/<ROBOT>_description/CMakeLists.txt
robots/<ROBOT>_description/package.ros1.xml
robots/<ROBOT>_description/package.ros2.xml
robots/<ROBOT>_description/xacro/robot.xacro
robots/<ROBOT>_description/xacro/gazebo.xacro
robots/<ROBOT>_description/config/robot_control.yaml
robots/<ROBOT>_description/config/robot_control_ros2.yaml

# 你训练的policy
rl_sar/policy/fsm.hpp
rl_sar/policy/<ROBOT>/fsm.hpp
rl_sar/policy/<ROBOT>/base.yaml  # 此文件中必须遵守实物机器人的关节顺序
rl_sar/policy/<ROBOT>/<CONFIG>/config.yaml
rl_sar/policy/<ROBOT>/<CONFIG>/<POLICY>.pt  # 必须导出jit才可使用

# 你实物机器人的代码
rl_sar/src/rl_real_<ROBOT>.cpp  # 可以按需自定义forward()函数以适配您的policy
```

## 贡献

衷心欢迎社区的贡献，以使这个框架更加成熟和对所有人有用。贡献可以是bug报告、功能请求或代码贡献。

[贡献者名单](CONTRIBUTORS.md)

## 引用

如果您使用此代码或其部分内容，请引用以下内容：

```
@software{fan-ziqi2024rl_sar,
  author = {fan-ziqi},
  title = {rl_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.},
  url = {https://github.com/fan-ziqi/rl_sar},
  year = {2024}
}
```

## 致谢

本项目使用了以下开源代码库中的部分代码：

- [unitreerobotics/unitree_sdk2-2.0.0](https://github.com/unitreerobotics/unitree_sdk2/tree/2.0.0)
- [unitreerobotics/unitree_legged_sdk-v3.2](https://github.com/unitreerobotics/unitree_legged_sdk/tree/v3.2)
- [unitreerobotics/unitree_guide](https://github.com/unitreerobotics/unitree_guide)
- [mertgungor/unitree_model_control](https://github.com/mertgungor/unitree_model_control)
- [Improbable-AI/walk-these-ways](https://github.com/Improbable-AI/walk-these-ways)
- [ccrpRepo/RoboMimic_Deploy](https://github.com/ccrpRepo/RoboMimic_Deploy)
- [Deeprobotics/Lite3_Motion_SDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK)
