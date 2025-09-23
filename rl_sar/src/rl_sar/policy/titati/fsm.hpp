/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef Titati_FSM_HPP
#define Titati_FSM_HPP

#include "fsm_core.hpp"   // FSM 基类与核心逻辑
#include "rl_sdk.hpp"     // RL 控制相关的接口

namespace titati_fsm
{

// ========== 状态 1：被动模式 ==========
class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        // 进入被动模式时，RL 不运行
        rl.running_percent = 0.0f;
        std::cout << LOGGER::NOTE
                  << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp."
                  << std::endl;
    }

    void Run() override
    {
        // 被动模式下，关节不给位置控制，只给阻尼，保持柔顺
        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            // fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i]; // 不锁死位置
            fsm_command->motor_command.dq[i] = 0;   // 不指定速度
            fsm_command->motor_command.kp[i] = 0;   // Kp=0，表示无位置刚度
            fsm_command->motor_command.kd[i] = 8;   // 给较大的阻尼，避免乱晃
            fsm_command->motor_command.tau[i] = 0;  // 无额外力矩
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        // 按键触发：键盘 Num0 或手柄 A 键 -> 切换到起身状态
        if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
            rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_; // 否则保持当前状态
    }
};

// ========== 状态 2：起身 ==========
class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    float pre_running_percent = 0.0f;   // 插值进度（预起身阶段）
    std::vector<float> pre_running_pos = {
        // 起身时的中间参考关节角 (rad)
        0.00, 1.36, -2.65,
        0.00, 1.36, -2.65,
        0.00, 1.36, -2.65,
        0.00, 1.36, -2.65,
        0.00, 0.00, 0.00, 0.00
    };

    void Enter() override
    {
        pre_running_percent = 0.0f;
        rl.running_percent = 0.0f;
        rl.now_state = *fsm_state;   // 记录当前状态
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        // Step 1: 先从当前状态 -> pre_running_pos 插值（200 步）
        if (pre_running_percent < 1.0f)
        {
            pre_running_percent += 1.0f / 200.0f;
            pre_running_percent = std::min(pre_running_percent, 1.0f);

            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                fsm_command->motor_command.q[i] =
                    (1 - pre_running_percent) * rl.now_state.motor_state.q[i] +
                    pre_running_percent * pre_running_pos[i];
                fsm_command->motor_command.dq[i] = 0;
                fsm_command->motor_command.kp[i] = rl.params.fixed_kp[0][i].item<double>();
                fsm_command->motor_command.kd[i] = rl.params.fixed_kd[0][i].item<double>();
                fsm_command->motor_command.tau[i] = 0;
            }
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                      << "Pre Getting up " << std::fixed << std::setprecision(2)
                      << pre_running_percent * 100.0f << "%" << std::flush;
        }

        // Step 2: 再从 pre_running_pos -> default_dof_pos 插值（400 步）
        if (pre_running_percent == 1 && rl.running_percent < 1.0f)
        {
            rl.running_percent += 1.0f / 400.0f;
            rl.running_percent = std::min(rl.running_percent, 1.0f);

            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                fsm_command->motor_command.q[i] =
                    (1 - rl.running_percent) * pre_running_pos[i] +
                    rl.running_percent * rl.params.default_dof_pos[0][i].item<double>();
                fsm_command->motor_command.dq[i] = 0;
                fsm_command->motor_command.kp[i] = rl.params.fixed_kp[0][i].item<double>();
                fsm_command->motor_command.kd[i] = rl.params.fixed_kd[0][i].item<double>();
                fsm_command->motor_command.tau[i] = 0;
            }
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                      << "Getting up " << std::fixed << std::setprecision(2)
                      << rl.running_percent * 100.0f << "%" << std::flush;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        // P 键/手柄 LB_X -> 回到 Passive
        if (rl.control.current_keyboard == Input::Keyboard::P ||
            rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        // 起身完成后，按键切换
        if (rl.running_percent == 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 ||
                rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRL_Locomotion"; // 切换到 RL 行走
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
                     rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";       // 切换到趴下
            }
        }
        return state_name_;
    }
};

// ========== 状态 3：趴下 ==========
class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    void Enter() override
    {
        rl.running_percent = 0.0f;
        rl.now_state = *fsm_state;   // 保存当前状态
    }

    void Run() override
    {
        // 从当前状态 -> start_state 插值（500 步）
        if (rl.running_percent < 1.0f)
        {
            rl.running_percent += 1.0f / 500.0f;
            rl.running_percent = std::min(rl.running_percent, 1.0f);

            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                fsm_command->motor_command.q[i] =
                    (1 - rl.running_percent) * rl.now_state.motor_state.q[i] +
                    rl.running_percent * rl.start_state.motor_state.q[i];
                fsm_command->motor_command.dq[i] = 0;
                fsm_command->motor_command.kp[i] = rl.params.fixed_kp[0][i].item<double>();
                fsm_command->motor_command.kd[i] = rl.params.fixed_kd[0][i].item<double>();
                fsm_command->motor_command.tau[i] = 0;
            }
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                      << "Getting down " << std::fixed << std::setprecision(2)
                      << rl.running_percent * 100.0f << "%" << std::flush;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        // P/LB_X 或者插值完成 -> Passive
        if (rl.control.current_keyboard == Input::Keyboard::P ||
            rl.control.current_gamepad == Input::Gamepad::LB_X ||
            rl.running_percent == 1.0f)
        {
            return "RLFSMStatePassive";
        }
        // Num0/A 键 -> 再次起身
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
                 rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

// ========== 状态 4：RL 行走 ==========
class RLFSMStateRL_Locomotion : public RLFSMState
{
public:
    RLFSMStateRL_Locomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRL_Locomotion") {}

    void Enter() override
    {
        rl.episode_length_buf = 0;

        // 从 yaml 读取 RL 配置
        rl.config_name = "robot_lab";
        std::string robot_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_path);     // 初始化 RL 策略
            rl.rl_init_done = true;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.control.current_keyboard = Input::Keyboard::Num0; // 出错则回到起身状态
        }

        // pos init （可扩展，初始化位姿）
    }

    void Run() override
    {
        // 打印 RL 控制命令（外部速度输入）
        std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                  << "RL Controller x:" << rl.control.x
                  << " y:" << rl.control.y
                  << " yaw:" << rl.control.yaw << std::flush;

        // 从 RL 输出队列里取动作（目标关节角/速度）
        torch::Tensor _output_dof_pos;
        torch::Tensor _output_dof_vel;
        torch::Tensor _output_dof_tau;
        if (rl.output_dof_pos_queue.try_pop(_output_dof_pos) &&
            rl.output_dof_vel_queue.try_pop(_output_dof_vel))
        {
            bool has_tau = rl.output_dof_tau_queue.try_pop(_output_dof_tau);
            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                if (_output_dof_pos.defined() && _output_dof_pos.numel() > 0)
                {
                    fsm_command->motor_command.q[i] = _output_dof_pos[0][i].item<double>();
                }
                if (_output_dof_vel.defined() && _output_dof_vel.numel() > 0)
                {
                    fsm_command->motor_command.dq[i] = _output_dof_vel[0][i].item<double>();
                }
                // 使用 RL 模式的 Kp/Kd
                fsm_command->motor_command.kp[i] = rl.params.rl_kp[0][i].item<double>();
                fsm_command->motor_command.kd[i] = rl.params.rl_kd[0][i].item<double>();
                if (has_tau && _output_dof_tau.defined() && _output_dof_tau.numel() > 0)
                {
                    fsm_command->motor_command.tau[i] = _output_dof_tau[0][i].item<double>();
                }
                else
                {
                    fsm_command->motor_command.tau[i] = 0;
                }
            }
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;   // 退出时标记 RL 结束
    }

    std::string CheckChange() override
    {
        // P/LB_X -> Passive
        if (rl.control.current_keyboard == Input::Keyboard::P ||
            rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        // Num9/B -> GetDown
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
                 rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        // Num0/A -> GetUp
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
                 rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        // Num1/RB_DPadUp -> RL_Locomotion（保持当前状态）
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 ||
                 rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRL_Locomotion";
        }
        return state_name_;
    }
};

} // namespace titati_fsm

// ========== FSM 工厂：用于创建不同状态对象 ==========
class TitatiFSMFactory : public FSMFactory
{
public:
    TitatiFSMFactory(const std::string& initial) : initial_state_(initial) {}

    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<titati_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<titati_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<titati_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRL_Locomotion")
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
    std::string initial_state_;   // 初始状态名
};

// 注册 FSM 工厂，默认初始状态是 Passive
REGISTER_FSM_FACTORY(TitatiFSMFactory, "RLFSMStatePassive")

#endif // Titati_FSM_HPP
