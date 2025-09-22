# Titati Hardware Sanity Test

This guide describes a lightweight bring-up test that can be executed on the master Jetson before running any reinforcement-learning (RL) policy on Titati. The helper binary exercises the new `TitatiHardware` interface without launching the legacy `titati_control` stack and verifies that both Tita halves stream motor feedback and accept MIT-style torque commands from `rl_sar`.

## Why run this test?

Running `titati_self_test` helps you confirm the minimum hardware prerequisites for sim-to-real experiments:

- Both Jetsons share the same CAN-FD backbone and publish router status frames.
- All 16 actuator channels report timestamps and update continuously.
- The READY→FORCE_DIRECT handshake succeeds so the MCU accepts externally commanded torques.
- Zero-torque commands can be broadcast without triggering faults.

If any of these checks fail, address the hardware or wiring issue before starting RL control.

## Prerequisites

1. **Power on both Tita platforms and the CAN-FD router.** The master Jetson runs the test, but the slave Jetson must also enable its `can0` interface so that the router keeps publishing the 0x09F status frame.
2. **Configure CAN-FD on each Jetson:**

   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 up type can bitrate 1000000 sample-point 0.80 \
        dbitrate 8000000 dsample-point 0.80 fd on restart-ms 100
   sudo ifconfig can0 txqueuelen 1000
   ```

   Adjust `can0` if you are using a different interface name.
3. **Build `rl_sar`** (choose the workflow that matches your environment):

   - **ROS 1 (catkin)**

     ```bash
     catkin_make
     source devel/setup.bash
     ```

   - **ROS 2 (colcon)**

     ```bash
     colcon build --packages-select rl_sar
     source install/setup.bash
     ```

   - **Standalone CMake**

     ```bash
     ./build.sh -m
     ```

     This flow generates binaries under `cmake_build/bin` and does **not** create an `install/` workspace or `setup.bash` script. Run the executables directly from `cmake_build/bin`.

## Running the test

Execute the binary from the master Jetson after sourcing the appropriate workspace setup file (standalone CMake builds do not require sourcing). The utility defaults to `can0` and runs for 10 seconds.

```bash
# ROS 1
rosrun rl_sar titati_self_test

# ROS 2
ros2 run rl_sar titati_self_test

# Standalone CMake
./cmake_build/bin/titati_self_test
```

Optional flags:

- `--can-interface <name>` — override the CAN device (for example, `can1` or `slcan0`).
- `--duration <seconds>` — total run time (default: 10).
- `--command-period <milliseconds>` — period for broadcasting zero MIT commands (default: 10 ms).
- `--motor-count <n>` — expected number of actuators (default: 16).
- `--jog-motor <index>` — gently oscillate a single motor (0-based index) to confirm motion.
- `--jog-torque <Nm>` — sine-wave amplitude for the jogged motor (default: 0.4 Nm).
- `--jog-period <milliseconds>` — sine-wave period for the jog motion (default: 2000 ms).

Example:

```bash
ros2 run rl_sar titati_self_test -- --can-interface can1 --duration 15 --jog-motor 3
```

When a jog motor is specified the tool waits until feedback from every actuator is streaming, then prints a message such as:

```
[TitatiSelfTest] Jogging motor 3 with ±0.4 Nm sine wave.
```

You should see and hear a gentle, low-amplitude motion on that actuator while the overall test continues to monitor bus health.

## Interpreting the output

During execution the tester prints periodic status lines:

```
[00.2s] waiting for motor feedback (0/16 ready)
[01.1s] streaming feedback: seq=87 motors=16 imu_ts=516842
[05.0s] command OK (500 frames, longest gap 4.0 ms)
```

At the end it summarizes the run and exits with one of the following statuses:

- **`PASS` (exit code 0):** all 16 actuators reported timestamps, the sequence counter advanced, and zero-torque commands were accepted for the requested duration.
- **`FAIL` (exit code 1):** no motor feedback was received—check the cabling, CAN configuration, or robot power.
- **`FAIL` (exit code 2):** feedback arrived but fewer than 16 actuators produced timestamps—verify the slave Jetson and router status.
- **`FAIL` (exit code 3):** CAN writes failed or the interface dropped offline—inspect the CAN cabling and retry after cycling power.

If the program is interrupted (Ctrl+C), it disables direct mode before exiting.

## Additional preparation steps

Passing the sanity test ensures that `rl_sar` can reach all 16 motors, but you should also:

1. **Check IMU orientation:** while `titati_self_test` is running, gently tilt the robot and confirm that the IMU quaternion changes in the console output. Persistent zeros indicate a router or sensor fault.
2. **Validate joint mapping:** open `rl_sar/policy/titati/config.yaml` and confirm that `joint_mapping` enumerates 16 entries. A mismatch will prevent the learned policy from commanding the correct actuators.
3. **Record a short log:** run `ros2 topic echo /titati/raw_state` (or the ROS 1 equivalent) in another terminal to ensure the state publisher you rely on for RL observes consistent timestamps.
4. **Review torque limits:** verify `max_torque` and safety thresholds in your policy configuration before applying non-zero torques on hardware.

Completing these checks gives confidence that the replacement `TitatiHardware` path is ready for sim-to-real deployment without falling back to `titati_control`.
