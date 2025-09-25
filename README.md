# Titati RL Control Integration

This repository replaces the original `titati_control` motion controller with the reinforcement-learning pipeline from `rl_sar` while keeping the CAN/MCU communication model used on the Titati platform.  The workflow focuses exclusively on the Titati quadruped (two Tita bases connected by the bridge box) and provides:

- a C++17 hardware SDK (`titati_sdk`) reused by the RL controller and the diagnostic tools;
- a real-robot RL executable (`rl_real_titati`) that reads the Titati CAN bus directly and publishes MIT commands for all 16 actuators;
- a command-line motor test utility (`titati_motor_test`) for checking and exercising every actuator before running the RL policy;
- a lightweight CAN router daemon (`titati_can_router`) that keeps the MCU in forced-direct SDK mode without ROS.

## 1. Build Instructions

Run the helper script on both Jetsons (master and slave) to generate the CMake hardware binaries:

```bash
cd rl_sar
./build.sh
```

The hardware executables are written to `rl_sar/cmake_build/bin/`.  Use `./build.sh -c` to remove the build directory.

The slave Jetson also needs the ROS 2 packages that expose the CAN-FD router node and the Titati system service interfaces (run once after every clean build):

```bash
source /opt/ros/humble/setup.bash
./build.sh tita_utils tita_system_interfaces titati_canfd_router
```

After the colcon build completes, source the workspace before launching any ROS programs:

```bash
source install/local_setup.bash
```

Repeat the ROS build on the master Jetson if you plan to run the router there for debugging.

## 2. Deployment Sequence

Perform the following steps each time the robot boots.  Commands marked **[master]** run on the front Jetson, **[slave]** on the rear Jetson.

1. **Bring up the CAN interface on both Jetsons**
   ```bash
   sudo systemctl stop tita-bringup.service
   sudo ip link set can0 down
   sudo ip link set can0 up type can bitrate 1000000 sample-point 0.80 \
        dbitrate 8000000 dsample-point 0.80 fd on restart-ms 100
   sudo ifconfig can0 txqueuelen 1000
   ```

2. **Start the CAN router daemon on the slave Jetson**
   ```bash
   cd rl_sar
   source install/local_setup.bash
   ros2 run titati_canfd_router titati_canfd_router_node
   ```
   Leave this node running; it listens for the CAN-FD heartbeat and automatically issues the forced-direct handshake so the MCU accepts SDK commands from the master.  If ROS 2 is not available you can fall back to the CLI daemon `./cmake_build/bin/titati_can_router`, but the ROS node mirrors the original `titati_control` behaviour and is recommended.

3. **Run motor diagnostics on the master Jetson** (recommended before every RL session)
   ```bash
   cd rl_sar
   ./cmake_build/bin/titati_motor_test --read        # one-shot snapshot of all joints
   ./cmake_build/bin/titati_motor_test --monitor     # streaming telemetry, Ctrl+C to stop
   ./cmake_build/bin/titati_motor_test --mode torque --id 3 --tau 1.0 --duration 2.0
   ./cmake_build/bin/titati_motor_test --mode mit \
       --id 5 --pos 0.2 --vel 0.0 --kp 60 --kd 3 --tau 0.0 --duration 1.5
   ```
   Use the torque/MIT commands to validate each of the 16 actuators individually before moving on.

4. **Launch the RL controller on the master Jetson**
   ```bash
   ./cmake_build/bin/rl_real_titati
   ```
   The controller loads parameters from `rl_sar/src/rl_sar/policy/titati/base.yaml` and the default Titati policy under `policy/titati/robot_lab/`.  Keyboard commands follow the standard `rl_sar` bindings (WASD for linear velocity, Q/E for yaw, `Space` to zero commands).  Use `Ctrl+C` to exit; the program sends zero torque and releases SDK mode on shutdown.

## 3. Repository Layout Highlights

- `rl_sar/src/rl_sar/library/titati_sdk/` – CAN sender/receiver and Titati hardware wrapper derived from `titati_control`.
- `rl_sar/src/rl_sar/src/rl_real_titati.cpp` – real-robot RL executable.
- `rl_sar/src/rl_sar/src/titati_motor_test.cpp` – CLI diagnostics tool.
- `rl_sar/src/titati_canfd_router/src/main.cpp` – ROS 2 CAN router node that mirrors the original `titati_control` bring-up.
- `rl_sar/src/titati_canfd_router/src/main_cli.cpp` (built as `titati_can_router`) – ROS-free fallback daemon for environments without ROS 2.

Follow the sequence **build → bring up CAN → run the slave CAN router → verify all motors → launch `rl_real_titati`** to ensure the robot is safe and fully verified before the RL policy takes control.
