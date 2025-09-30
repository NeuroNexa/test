/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TITATI_FSM_HPP
#define TITATI_FSM_HPP

#include <algorithm>
#include <iomanip>
#include <vector>

#include "fsm_core.hpp"
#include "rl_sdk.hpp"

namespace titati_fsm
{

inline double ComputePD(const RLFSMState& state_ctx, int index, double target_q, double target_dq,
                        const torch::Tensor& kp_tensor, const torch::Tensor& kd_tensor)
{
    const double kp = kp_tensor[0][index].item<double>();
    const double kd = kd_tensor[0][index].item<double>();
    const double current_q = state_ctx.fsm_state->motor_state.q[index];
    const double current_dq = state_ctx.fsm_state->motor_state.dq[index];
    return kp * (target_q - current_q) - kd * (current_dq - target_dq);
}

class RLFSMStatePassive : public RLFSMState
{
public:
    explicit RLFSMStatePassive(RL* rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        rl.running_percent = 0.0f;
        std::cout << LOGGER::NOTE
                  << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp."
                  << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            fsm_command->motor_command.tau[i] = 0.0;
            fsm_command->motor_command.q[i] = 0.0;
            fsm_command->motor_command.dq[i] = 0.0;
            fsm_command->motor_command.kp[i] = 0.0;
            fsm_command->motor_command.kd[i] = 0.0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
            rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    explicit RLFSMStateGetUp(RL* rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    void Enter() override
    {
        pre_running_percent = 0.0f;
        rl.running_percent = 0.0f;
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        if (pre_running_percent < 1.0f)
        {
            pre_running_percent += 1.0f / 200.0f;
            pre_running_percent = std::min(pre_running_percent, 1.0f);

            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                const double start_q = rl.now_state.motor_state.q[i];
                const double target_q = pre_running_pos[i];
                const double desired_q = (1.0 - pre_running_percent) * start_q + pre_running_percent * target_q;
                const double tau = ComputePD(*this, i, desired_q, 0.0,
                                             rl.params.fixed_kp, rl.params.fixed_kd);
                fsm_command->motor_command.tau[i] = tau;
                fsm_command->motor_command.q[i] = 0.0;
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = 0.0;
                fsm_command->motor_command.kd[i] = 0.0;
            }

            std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "Pre Getting up " << std::fixed
                      << std::setprecision(2) << pre_running_percent * 100.0f << "%" << std::flush;
        }

        if (pre_running_percent == 1.0f && rl.running_percent < 1.0f)
        {
            rl.running_percent += 1.0f / 400.0f;
            rl.running_percent = std::min(rl.running_percent, 1.0f);

            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                const double start_q = pre_running_pos[i];
                const double target_q = rl.params.default_dof_pos[0][i].item<double>();
                const double desired_q = (1.0 - rl.running_percent) * start_q + rl.running_percent * target_q;
                const double tau = ComputePD(*this, i, desired_q, 0.0,
                                             rl.params.fixed_kp, rl.params.fixed_kd);
                fsm_command->motor_command.tau[i] = tau;
                fsm_command->motor_command.q[i] = 0.0;
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = 0.0;
                fsm_command->motor_command.kd[i] = 0.0;
            }

            std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "Getting up " << std::fixed
                      << std::setprecision(2) << rl.running_percent * 100.0f << "%" << std::flush;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P ||
            rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (rl.running_percent == 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 ||
                rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRL_Locomotion";
            }
            if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
                rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }

private:
    float pre_running_percent = 0.0f;
    const std::vector<double> pre_running_pos = {
        0.00, 1.40, -2.40,
        0.00, 1.40, -2.40,
        0.00, -1.40, 2.40,
        0.00, -1.40, 2.40,
        0.00, 0.00, 0.00, 0.00
    };
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    explicit RLFSMStateGetDown(RL* rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    void Enter() override
    {
        rl.running_percent = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        if (rl.running_percent < 1.0f)
        {
            rl.running_percent += 1.0f / 500.0f;
            rl.running_percent = std::min(rl.running_percent, 1.0f);
            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                const double desired = (1.0 - rl.running_percent) * rl.now_state.motor_state.q[i] +
                                       rl.running_percent * rl.start_state.motor_state.q[i];
                const double tau = ComputePD(*this, i, desired, 0.0,
                                             rl.params.fixed_kp, rl.params.fixed_kd);
                fsm_command->motor_command.tau[i] = tau;
                fsm_command->motor_command.q[i] = 0.0;
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = 0.0;
                fsm_command->motor_command.kd[i] = 0.0;
            }
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "Getting down " << std::fixed
                      << std::setprecision(2) << rl.running_percent * 100.0f << "%" << std::flush;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P ||
            rl.control.current_gamepad == Input::Gamepad::LB_X ||
            rl.running_percent == 1.0f)
        {
            return "RLFSMStatePassive";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
            rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateRL_Locomotion : public RLFSMState
{
public:
    explicit RLFSMStateRL_Locomotion(RL* rl) : RLFSMState(*rl, "RLFSMStateRL_Locomotion") {}

    void Enter() override
    {
        rl.episode_length_buf = 0;
        rl.config_name = "robot_lab";
        const std::string robot_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_path);
            rl.rl_init_done = true;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.control.current_keyboard = Input::Keyboard::Num0;
        }
    }

    void Run() override
    {
        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller x:" << rl.control.x
                  << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;

        torch::Tensor dummy_pos;
        torch::Tensor dummy_vel;
        torch::Tensor dummy_tau;
        const bool has_pos = rl.output_dof_pos_queue.try_pop(dummy_pos);
        const bool has_vel = rl.output_dof_vel_queue.try_pop(dummy_vel);
        rl.output_dof_tau_queue.try_pop(dummy_tau);
        if (has_pos && has_vel)
        {
            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                if (rl.output_dof_pos.defined() && rl.output_dof_pos.numel() > 0)
                {
                    fsm_command->motor_command.q[i] = rl.output_dof_pos[0][i].item<double>();
                }
                if (rl.output_dof_vel.defined() && rl.output_dof_vel.numel() > 0)
                {
                    fsm_command->motor_command.dq[i] = rl.output_dof_vel[0][i].item<double>();
                }
                fsm_command->motor_command.kp[i] = rl.params.rl_kp[0][i].item<double>();
                fsm_command->motor_command.kd[i] = rl.params.rl_kd[0][i].item<double>();
                fsm_command->motor_command.tau[i] = 0.0;
            }
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P ||
            rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
            rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
            rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num1 ||
            rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRL_Locomotion";
        }
        return state_name_;
    }

};

} // namespace titati_fsm

class TitatiFSMFactory : public FSMFactory
{
public:
    explicit TitatiFSMFactory(const std::string& initial) : initial_state_(initial) {}

    std::shared_ptr<FSMState> CreateState(void* context, const std::string& state_name) override
    {
        auto* rl = static_cast<RL*>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<titati_fsm::RLFSMStatePassive>(rl);
        if (state_name == "RLFSMStateGetUp")
            return std::make_shared<titati_fsm::RLFSMStateGetUp>(rl);
        if (state_name == "RLFSMStateGetDown")
            return std::make_shared<titati_fsm::RLFSMStateGetDown>(rl);
        if (state_name == "RLFSMStateRL_Locomotion")
            return std::make_shared<titati_fsm::RLFSMStateRL_Locomotion>(rl);
        return nullptr;
    }

    std::string GetType() const override { return "titati"; }

    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRL_Locomotion"
        };
    }

    std::string GetInitialState() const override { return initial_state_; }

private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(TitatiFSMFactory, "RLFSMStatePassive")

#endif // TITATI_FSM_HPP