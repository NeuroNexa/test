# Titati deployment on Jetson Orin NX (aarch64)

This guide walks through preparing a Jetson Orin NX 16 GB controller pair ("master" and "slave") to run the Titati real-time controller that ships with `rl_sar`.  The steps assume JetPack 6.x (CUDA 12.2) and a ROS-less hardware build.

> [!IMPORTANT]
> Always put the robot on a stand or support harness during bring-up.  Keep a physical e-stop handy and be ready to cut power if the actuators move unexpectedly.

## 1. Clone the code and install dependencies

1. Update the apt index and install the native dependencies required by the CMake build:
   ```bash
   sudo apt update
   sudo apt install --yes git build-essential cmake libyaml-cpp-dev liblcm-dev libtbb-dev python3-dev
   ```
2. (Optional) Install ROS 2 Humble if you plan to use the ROS-based launch files.  The hardware binaries in this guide do not depend on ROS.
3. Clone the repository on both the master and the slave Jetson computers:
   ```bash
   cd ~/wjl/test
   git clone https://github.com/fan-ziqi/rl_sar.git
   ```

## 2. Prepare LibTorch and CUDA environment variables

PyTorch is already available through the Python wheel on Jetson.  Export the C++ headers and libraries so CMake can link against them.

1. Copy the wheel contents to `~/libtorch` (run once per machine):
   ```bash
   mkdir -p ~/libtorch
   cp -r ~/.local/lib/python3.10/site-packages/torch/{bin,include,lib,share} ~/libtorch
   ```
2. Append the minimal environment configuration to `~/.bashrc`:
   ```bash
   cat <<'EOT' >> ~/.bashrc
# >>> rl_sar Titati hardware env >>>
export Torch_DIR="$HOME/libtorch"
if [ -d "$Torch_DIR/lib" ]; then
  export LD_LIBRARY_PATH="$Torch_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
if [ -d /usr/local/cuda-12.2 ]; then
  export CUDA_HOME=/usr/local/cuda-12.2
  export PATH="$CUDA_HOME/bin:$PATH"
  export CUDACXX="$CUDA_HOME/bin/nvcc"
  export LD_LIBRARY_PATH="$CUDA_HOME/targets/aarch64-linux/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
# <<< rl_sar Titati hardware env <<<
EOT
   ```
3. Reload the shell and verify PyTorch is visible:
   ```bash
   source ~/.bashrc
   python3 -c "import torch, sys; print(sys.version); print(torch.__file__)"
   ```

## 3. Configure the CAN FD buses

Titati uses one or two CAN FD links:
- `can0` – required.  Always carries feedback from all 16 joints.
- `can1` – optional.  Some wiring looms dedicate it to actuator torque commands.

Start by confirming which interfaces exist.  On many builds only `can0` is wired, so `can1` will not appear in `ip link show`.

```bash
ip link show | grep -E "can[0-9]"
```

If the interface you plan to use does not appear in the output, stay with `can0` for both feedback and commands—running the tools with `--command-can can1` will fail with `if_nametoindex: No such device`.

Bring up `can0` (repeat on both Jetsons whenever the machine boots):

```bash
sudo systemctl stop tita-bringup.service  # stop vendor daemon that monopolises the bus
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.80 \
    dbitrate 8000000 dsample-point 0.80 fd on restart-ms 100
sudo ifconfig can0 txqueuelen 1000
```

If you also have a second actuator bus (`can1`), bring it up with the same parameters and use the `--command-can can1` flag when running diagnostics or the RL controller.

## 4. Keep the slave Titati router alive

The slave Jetson must run `titati_canfd_router_node` continuously; otherwise the MCU will fall back to `AUTO_LOCOMOTION` and ignore external MIT commands.  Use the vendor launch file after the CAN interface is ready:

```bash
cd ~/tita_ros2_open
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch titati_bringup quadruped_slave_controller.launch.py
```

Verify the process stays alive with:

```bash
ps -ef | grep titati_canfd_router
```

Only one router instance should be running—if the command shows several, terminate the duplicates before continuing.

## 5. Build the Titati hardware binaries

1. On each Jetson, enter the repository and run the CMake-based build:
   ```bash
   cd ~/wjl/test/rl_sar
   ./build.sh -m -- -DRL_SAR_BUILD_TITATI_ONLY=ON
   ```
   The option skips Unitree/Lite3 targets and configures only the Titati executables.  Subsequent incremental builds can omit the extra flag (`./build.sh -m`).
2. The binaries are written to `cmake_build/bin/`.  The ones used for bring-up are:
   - `titati_motor_test` – diagnostics (monitor, single-joint, torque tests)
   - `rl_real_titati` – reinforcement-learning controller

## 6. Run smoke tests

1. **Check feedback** – from the master Jetson:
   ```bash
   ./cmake_build/bin/titati_motor_test --mode monitor
   ```
   Confirm all 16 joints stream sensible positions/velocities.
2. **Scan the actuators** – still on the master Jetson, with the slave router running:
   ```bash
   ./cmake_build/bin/titati_motor_test --mode scan --offset 0.12 --rate 400
   ```
   If the program reports “joint barely moved,” inspect the command bus with `sudo candump -L can0` (or `can1`) and confirm the router process is alive.
3. **Move individual joints** – exercise wheel, calf, thigh, and hip joints by index.  Example for the front-right leg (joint indices 12, 2, 1, 0):
   ```bash
   ./cmake_build/bin/titati_motor_test --mode single --joint 12 --offset 0.15
   ./cmake_build/bin/titati_motor_test --mode single --joint 2  --offset 0.12
   ./cmake_build/bin/titati_motor_test --mode single --joint 1  --offset 0.12
   ./cmake_build/bin/titati_motor_test --mode single --joint 0  --offset 0.10
   ```

## 7. Launch the reinforcement-learning controller

1. Copy your TorchScript policy to `src/rl_sar/policy/titati/<CONFIG>/` and update the YAML files.
2. Start the controller once the robot is on a stand and the slave router is running:
   ```bash
   ./cmake_build/bin/rl_real_titati --can can0
   ```
   Use the more explicit form if the wiring splits feedback and command buses:
   ```bash
   ./cmake_build/bin/rl_real_titati --feedback-can can0 --command-can can1
   ```
3. During operation:
   - Press `M` or `Esc` in the terminal to engage the soft e-stop (zero torques and relinquish SDK mode).
   - Press `K` to re-enable control once conditions are safe.

Following the steps above reproduces the behaviour of the original `titati_control` hardware bridge while keeping the build lightweight and focused on Titati.
