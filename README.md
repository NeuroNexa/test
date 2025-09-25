# Titati RL Control Integration

This repository replaces the original `titati_control` motion controller with the reinforcement-learning pipeline from `rl_sar` while keeping the CAN/MCU communication model used on the Titati platform.  The workflow focuses exclusively on the Titati quadruped (two Tita bases connected by the bridge box) and provides:

- a C++17 hardware SDK (`titati_sdk`) reused by the RL controller and the diagnostic tools;
- a real-robot RL executable (`rl_real_titati`) that reads the Titati CAN bus directly and publishes MIT commands for all 16 actuators;
- a command-line motor test utility (`titati_motor_test`) for checking and exercising every actuator before running the RL policy;
- a lightweight CAN router daemon (`titati_can_router`) that keeps the MCU in forced-direct SDK mode without ROS.

## 1. Build Instructions

The project is built purely with CMake.  Run the helper script once on both Jetsons (master and slave) after cloning or pulling updates:

```bash
cd rl_sar
./build.sh
```

The hardware binaries are generated under `rl_sar/cmake_build/bin/`.  Use `./build.sh -c` if you need to remove all build artefacts and reconfigure from scratch.

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
   ./cmake_build/bin/titati_can_router
   ```
   Keep this process running; it listens for the CAN-FD heartbeat and automatically sends the forced-direct handshake so the MCU accepts SDK commands from the master.

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
- `rl_sar/src/titati_canfd_router/src/main.cpp` (built as `titati_can_router`) – forced-direct CAN handshake daemon.

Follow the sequence **build → bring up CAN → run the slave CAN router → verify all motors → launch `rl_real_titati`** to ensure the robot is safe and fully verified before the RL policy takes control.
