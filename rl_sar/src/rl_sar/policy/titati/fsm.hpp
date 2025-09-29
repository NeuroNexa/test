/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TITATI_FSM_HPP
#define TITATI_FSM_HPP

#include "fsm_core.hpp"
#include "rl_sdk.hpp"

#include <iomanip>

namespace titati_fsm
{

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

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
            fsm_command->motor_command.dq[i] = 0.0;
            fsm_command->motor_command.kp[i] = 0.0;
            fsm_command->motor_command.kd[i] = 8.0;
            fsm_command->motor_command.tau[i] = 0.0;
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
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    void Enter() override
    {
        pre_running_percent_ = 0.0f;
        rl.running_percent = 0.0f;
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        if (pre_running_percent_ < 1.0f)
        {
            pre_running_percent_ += 1.0f / 200.0f;
            pre_running_percent_ = std::min(pre_running_percent_, 1.0f);

            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                fsm_command->motor_command.q[i] =
                    (1.0f - pre_running_percent_) * rl.now_state.motor_state.q[i] +
                    pre_running_percent_ * pre_running_pos_[i];
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = rl.params.fixed_kp[0][i].item<double>();
                fsm_command->motor_command.kd[i] = rl.params.fixed_kd[0][i].item<double>();
                fsm_command->motor_command.tau[i] = 0.0;
            }
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                      << "Pre Getting up " << std::fixed << std::setprecision(2)
                      << pre_running_percent_ * 100.0f << "%" << std::flush;
        }

        if (pre_running_percent_ == 1.0f && rl.running_percent < 1.0f)
        {
            rl.running_percent += 1.0f / 400.0f;
            rl.running_percent = std::min(rl.running_percent, 1.0f);

            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                fsm_command->motor_command.q[i] =
                    (1.0f - rl.running_percent) * pre_running_pos_[i] +
                    rl.running_percent * rl.params.default_dof_pos[0][i].item<double>();
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = rl.params.fixed_kp[0][i].item<double>();
                fsm_command->motor_command.kd[i] = rl.params.fixed_kd[0][i].item<double>();
                fsm_command->motor_command.tau[i] = 0.0;
            }
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                      << "Getting up " << std::fixed << std::setprecision(2)
                      << rl.running_percent * 100.0f << "%" << std::flush;
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
    float pre_running_percent_ = 0.0f;
    std::vector<float> pre_running_pos_ = {
        -0.40f, 1.40f, -2.40f, 0.00f,
        0.40f, 1.40f, -2.40f, 0.00f,
        0.40f, 1.40f, -2.40f, 0.00f,
        -0.40f, 1.40f, -2.40f, 0.00f
    };
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

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
                fsm_command->motor_command.q[i] =
                    (1.0f - rl.running_percent) * rl.now_state.motor_state.q[i] +
                    rl.running_percent * rl.start_state.motor_state.q[i];
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = rl.params.fixed_kp[0][i].item<double>();
                fsm_command->motor_command.kd[i] = rl.params.fixed_kd[0][i].item<double>();
                fsm_command->motor_command.tau[i] = 0.0;
            }
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                      << "Getting down " << std::fixed << std::setprecision(2)
                      << rl.running_percent * 100.0f << "%" << std::flush;
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
    RLFSMStateRL_Locomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRL_Locomotion") {}

    void Enter() override
    {
        rl.episode_length_buf = 0;
        rl.config_name = "robot_lab";
        std::string robot_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_path);
            rl.rl_init_done = true;
        }
        catch (const std::exception &e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.control.current_keyboard = Input::Keyboard::Num0;
        }
    }

    void Run() override
    {
        std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                  << "RL Controller x:" << rl.control.x
                  << " y:" << rl.control.y
                  << " yaw:" << rl.control.yaw << std::flush;

        torch::Tensor latest_pos, latest_vel, latest_tau;
        torch::Tensor tensor_buffer;

        while (rl.output_dof_pos_queue.try_pop(tensor_buffer))
        {
            latest_pos = tensor_buffer;
        }
        while (rl.output_dof_vel_queue.try_pop(tensor_buffer))
        {
            latest_vel = tensor_buffer;
        }
        while (rl.output_dof_tau_queue.try_pop(tensor_buffer))
        {
            latest_tau = tensor_buffer;
        }

        if (!latest_pos.defined())
        {
            latest_pos = rl.output_dof_pos;
        }
        if (!latest_vel.defined())
        {
            latest_vel = rl.output_dof_vel;
        }
        if (latest_tau.defined())
        {
            rl.output_dof_tau = latest_tau;
        }

        if (latest_pos.defined() && latest_vel.defined() &&
            latest_pos.numel() > 0 && latest_vel.numel() > 0)
        {
            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                fsm_command->motor_command.q[i] = latest_pos[0][i].item<double>();
                fsm_command->motor_command.dq[i] = latest_vel[0][i].item<double>();
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
    explicit TitatiFSMFactory(const std::string &initial) : initial_state_(initial) {}

    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
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
