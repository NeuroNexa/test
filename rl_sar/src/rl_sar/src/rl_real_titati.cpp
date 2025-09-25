#include "rl_real_titati.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <thread>

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
}

RL_Real::RL_Real()
{
    this->ang_vel_type = "ang_vel_body";
    this->robot_name = "titati";
    this->ReadYamlBase(this->robot_name);

    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "No FSM registered for robot: " << this->robot_name << std::endl;
    }

    torch::autograd::GradMode::set_enabled(false);
    torch::set_num_threads(4);

    this->InitOutputs();
    this->InitControl();

    if (this->params.torque_limits.numel() > 0)
    {
        torch::Tensor limits = this->params.torque_limits.flatten();
        this->torque_limits.resize(limits.numel());
        for (int64_t i = 0; i < limits.numel(); ++i)
        {
            this->torque_limits[i] = limits[i].item<double>();
        }
    }
    else
    {
        this->torque_limits.assign(this->params.num_of_dofs, std::numeric_limits<double>::max());
    }

    if (!this->hardware.EnableDirectControl())
    {
        std::cout << LOGGER::WARNING << "Failed to request Titati direct control mode" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&RL_Real::RunModel, this));
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

    std::cout << LOGGER::INFO << "Titati RL controller initialized" << std::endl;
}

RL_Real::~RL_Real()
{
    if (this->loop_keyboard)
    {
        this->loop_keyboard->shutdown();
    }
    if (this->loop_control)
    {
        this->loop_control->shutdown();
    }
    if (this->loop_rl)
    {
        this->loop_rl->shutdown();
    }
    std::cout << LOGGER::INFO << "RL_Real Titati exit" << std::endl;
}

void RL_Real::GetState(RobotState<double> *state)
{
    const auto joint_state = this->hardware.ReadJointState();
    const auto imu_state = this->hardware.ReadImuState();

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        int mapped = this->params.joint_mapping[i];
        if (mapped >= static_cast<int>(joint_state.position.size()))
        {
            continue;
        }

        double position = joint_state.position[mapped];
        if (mapped == 1 || mapped == 1 + this->params.num_of_dofs / 2)
        {
            if (position < -2.5)
            {
                position += kTwoPi;
            }
        }
        state->motor_state.q[i] = position;
        state->motor_state.dq[i] = joint_state.velocity[mapped];
        state->motor_state.tau_est[i] = joint_state.torque[mapped];
    }

    state->imu.quaternion[0] = imu_state.quaternion_wxyz[0];
    state->imu.quaternion[1] = imu_state.quaternion_wxyz[1];
    state->imu.quaternion[2] = imu_state.quaternion_wxyz[2];
    state->imu.quaternion[3] = imu_state.quaternion_wxyz[3];

    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = imu_state.angular_velocity[i];
        state->imu.accelerometer[i] = imu_state.linear_acceleration[i];
    }
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    const std::size_t motor_count = this->hardware.MotorCount();
    std::vector<double> q(motor_count, 0.0);
    std::vector<double> dq(motor_count, 0.0);
    std::vector<double> kp(motor_count, 0.0);
    std::vector<double> kd(motor_count, 0.0);
    std::vector<double> tau(motor_count, 0.0);

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        int mapped = this->params.joint_mapping[i];
        if (mapped >= static_cast<int>(motor_count))
        {
            continue;
        }
        q[mapped] = command->motor_command.q[i];
        dq[mapped] = command->motor_command.dq[i];
        kp[mapped] = command->motor_command.kp[i];
        kd[mapped] = command->motor_command.kd[i];
        double limit = (i < static_cast<int>(this->torque_limits.size())) ? this->torque_limits[i] : std::numeric_limits<double>::max();
        tau[mapped] = std::clamp(command->motor_command.tau[i], -limit, limit);
    }

    if (!this->hardware.SendMITCommand(q, dq, kp, kd, tau))
    {
        std::cout << LOGGER::WARNING << "Failed to send Titati MIT command" << std::endl;
    }
}

void RL_Real::RobotControl()
{
    this->motiontime++;

    if (this->control.current_keyboard == Input::Keyboard::W)
    {
        this->control.x += 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::S)
    {
        this->control.x -= 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::A)
    {
        this->control.y += 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::D)
    {
        this->control.y -= 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Q)
    {
        this->control.yaw += 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::E)
    {
        this->control.yaw -= 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Space)
    {
        this->control.x = 0.0;
        this->control.y = 0.0;
        this->control.yaw = 0.0;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::N)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
        this->control.current_keyboard = this->control.last_keyboard;
    }

    this->GetState(&this->robot_state);
    this->StateController(&this->robot_state, &this->robot_command);
    this->SetCommand(&this->robot_command);
}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = torch::tensor(this->robot_state.imu.gyroscope).unsqueeze(0);
        this->obs.base_quat = torch::tensor(this->robot_state.imu.quaternion).unsqueeze(0);
        this->obs.dof_pos = torch::tensor(this->robot_state.motor_state.q).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);
        this->obs.dof_vel = torch::tensor(this->robot_state.motor_state.dq).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);

        if (this->control.navigation_mode)
        {
            this->obs.commands = torch::tensor({{0.0, 0.0, 0.0}});
        }
        else
        {
            this->obs.commands = torch::tensor({{this->control.x, this->control.y, this->control.yaw}});
        }

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (this->output_dof_pos.defined() && this->output_dof_pos.numel() > 0)
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (this->output_dof_vel.defined() && this->output_dof_vel.numel() > 0)
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (this->output_dof_tau.defined() && this->output_dof_tau.numel() > 0)
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        this->TorqueProtect(this->output_dof_tau);
    }
}

torch::Tensor RL_Real::Forward()
{
    torch::autograd::GradMode::set_enabled(false);
    torch::Tensor clamped_obs = this->ComputeObservation();
    torch::Tensor actions;
    if (!this->params.observations_history.empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.observations_history);
        actions = this->model.forward({this->history_obs}).toTensor();
    }
    else
    {
        actions = this->model.forward({clamped_obs}).toTensor();
    }

    if (this->params.clip_actions_upper.numel() != 0 && this->params.clip_actions_lower.numel() != 0)
    {
        return torch::clamp(actions, this->params.clip_actions_lower, this->params.clip_actions_upper);
    }
    return actions;
}

int main(int argc, char **argv)
{
    RL_Real rl_sar;
    while (1)
    {
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }
    return 0;
}
