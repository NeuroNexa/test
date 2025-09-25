/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_TITATI_HPP
#define RL_REAL_TITATI_HPP

// #define PLOT
// #define CSV_LOGGER

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "fsm.hpp"

#include "titati_sdk/tita_robot.hpp"

#include <csignal>
#include <memory>
#include <vector>

class RL_Real : public RL
{
public:
    RL_Real();
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
    std::shared_ptr<LoopFunc> loop_plot;

#ifdef PLOT
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<double>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();
#endif

    std::unique_ptr<titati::TitatiRobot> robot;

    int motiontime = 0;
};

#endif // RL_REAL_TITATI_HPP

