/*
 * Reinforcement learning deployment for Titati hardware.
 */

#ifndef RL_REAL_TITATI_HPP
#define RL_REAL_TITATI_HPP

// #define PLOT
// #define CSV_LOGGER
// #define USE_ROS

#include "rl_sdk.hpp"
#include "loop.hpp"
#include "titati/titati_hardware.hpp"

#include <csignal>

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

    rl_sar::TitatiHardware hardware;
    std::vector<double> torque_limits;
    int motiontime = 0;
};

#endif // RL_REAL_TITATI_HPP
