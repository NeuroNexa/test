# Titati RL Deployment Workspace

This workspace contains two ROS-related projects:

- `rl_sar/`: the reinforcement learning stack and all hardware-facing utilities that should be used on the robot. Updated quick-start instructions for the Titati robot live in [rl_sar/README.md](rl_sar/README.md).
- `titati_control/`: the original vendor packages kept for reference only. All components that are still required on the robot have been migrated into `rl_sar` and do not need to be built or launched directly.

Follow the build and launch steps from the `rl_sar` documentation to bring up the master/slave Jetsons, run the CAN power services, and start the RL controller.
