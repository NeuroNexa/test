/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_TITATI_HPP
#define RL_REAL_TITATI_HPP

// #define CSV_LOGGER

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "fsm.hpp"

#include "titati_hw/tita_robot.hpp"

#include <chrono>
#include <memory>
#include <vector>

class RL_Real : public RL
{
public:
    explicit RL_Real(bool wheel_torque_mode = true);
    ~RL_Real();

private:
    torch::Tensor Forward() override;
    void GetState(RobotState<double> *state) override;
    void SetCommand(const RobotCommand<double> *command) override;
    void RunModel();
    void RobotControl();

    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;

    std::unique_ptr<titati::hardware::TitatiRobot> robot_;
    bool wheel_torque_mode_;

    std::vector<double> joint_command_pos_;
    std::vector<double> joint_command_vel_;
    std::vector<double> joint_command_tau_;

    int motiontime_ = 0;
    std::chrono::steady_clock::time_point last_print_time_;
};

#endif  // RL_REAL_TITATI_HPP
