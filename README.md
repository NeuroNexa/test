# Titati RL bring-up workspace

This workspace combines the upstream `rl_sar` reinforcement-learning stack with the legacy `titati_control` utilities so that Titati (two Tita robots joined through the connector box) can be driven directly by learned policies.

If you are looking for day-to-day usage instructions, start with [`rl_sar/README.md`](rl_sar/README.md). The new **DDT Titati** section under “Real Robots” walks through:

- preparing the CAN-FD interface on the master and slave Jetsons,
- replaying the vendor FORCE_DIRECT handshake with `titati_can_router` on both Jetsons,
- validating all 16 actuators with the standalone motor tester once CAN is online,
- placing the trained policy inside `rl_sar/src/rl_sar/policy/titati/robot_lab/`,
- compiling and launching `rl_real_titati` via ROS1, ROS2, or the pure CMake binary, and
- safety checks to perform before letting the robot support itself.

The original `titati_control` scripts are still available under `titati_control/src` for reference (for example, to reuse their CAN configuration commands).
