/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_titati.hpp"

#include "titati/fsm.hpp"

#include <algorithm>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

RL_Real::RL_Real(bool wheel_torque_mode)
    : wheel_torque_mode_(wheel_torque_mode)
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

    robot_ = std::make_unique<titati::hardware::TitatiRobot>(this->params.num_of_dofs);
    robot_->set_motors_sdk(true);

    joint_command_pos_.assign(this->params.num_of_dofs, 0.0);
    joint_command_vel_.assign(this->params.num_of_dofs, 0.0);
    joint_command_tau_.assign(this->params.num_of_dofs, 0.0);

    this->InitOutputs();
    this->InitControl();

    loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&RL_Real::RobotControl, this));
    loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&RL_Real::RunModel, this));
    loop_keyboard->start();
    loop_control->start();
    loop_rl->start();

    last_print_time_ = std::chrono::steady_clock::now();
}

RL_Real::~RL_Real()
{
    loop_keyboard->shutdown();
    loop_control->shutdown();
    loop_rl->shutdown();
    std::cout << LOGGER::INFO << "RL_Real titati exit" << std::endl;
}

void RL_Real::GetState(RobotState<double> *state)
{
    auto q = robot_->get_joint_q();
    auto v = robot_->get_joint_v();
    auto tau = robot_->get_joint_t();

    if (q.size() >= static_cast<std::size_t>(this->params.num_of_dofs))
    {
        for (int i = 0; i < this->params.num_of_dofs; ++i)
        {
            int idx = this->params.joint_mapping[i];
            double pos = q[idx];
            if (i == 1 || i == (1 + this->params.num_of_dofs / 2))
            {
                if (pos < -2.5)
                {
                    pos += 2.0 * kPi;
                }
            }
            state->motor_state.q[i] = pos;
            state->motor_state.dq[i] = v[idx];
            state->motor_state.tau_est[i] = tau[idx];
        }
    }

    auto quat = robot_->get_imu_quaternion();
    state->imu.quaternion[0] = quat[3];
    state->imu.quaternion[1] = quat[0];
    state->imu.quaternion[2] = quat[1];
    state->imu.quaternion[3] = quat[2];

    auto accel = robot_->get_imu_acceleration();
    auto gyro = robot_->get_imu_angular_velocity();
    for (int i = 0; i < 3; ++i)
    {
        state->imu.accelerometer[i] = accel[i];
        state->imu.gyroscope[i] = gyro[i];
    }
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    std::vector<double> q_hw(this->params.num_of_dofs, 0.0);
    std::vector<double> dq_hw(this->params.num_of_dofs, 0.0);
    std::vector<double> kp_hw(this->params.num_of_dofs, 0.0);
    std::vector<double> kd_hw(this->params.num_of_dofs, 0.0);
    std::vector<double> tau_hw(this->params.num_of_dofs, 0.0);

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        int idx = this->params.joint_mapping[i];
        bool is_wheel = std::find(this->params.wheel_indices.begin(), this->params.wheel_indices.end(), i) != this->params.wheel_indices.end();

        if (is_wheel && wheel_torque_mode_)
        {
            q_hw[idx] = 0.0;
            dq_hw[idx] = 0.0;
            kp_hw[idx] = 0.0;
            kd_hw[idx] = 0.0;
            tau_hw[idx] = command->motor_command.tau[i];
        }
        else
        {
            q_hw[idx] = command->motor_command.q[i];
            dq_hw[idx] = command->motor_command.dq[i];
            kp_hw[idx] = command->motor_command.kp[i];
            kd_hw[idx] = command->motor_command.kd[i];
            tau_hw[idx] = command->motor_command.tau[i];
        }
    }

    joint_command_pos_ = q_hw;
    joint_command_vel_ = dq_hw;
    joint_command_tau_ = tau_hw;

    robot_->set_target_joint_mit(joint_command_pos_, joint_command_vel_, kp_hw, kd_hw, joint_command_tau_);
}

void RL_Real::RobotControl()
{
    motiontime_++;

    if (this->control.current_keyboard == Input::Keyboard::W)
    {
        this->control.x += 0.05;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::S)
    {
        this->control.x -= 0.05;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::A)
    {
        this->control.y += 0.05;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::D)
    {
        this->control.y -= 0.05;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Q)
    {
        this->control.yaw += 0.05;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::E)
    {
        this->control.yaw -= 0.05;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Space)
    {
        this->control.x = 0;
        this->control.y = 0;
        this->control.yaw = 0;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::N)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << std::endl << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
        this->control.current_keyboard = this->control.last_keyboard;
    }

    this->GetState(&this->robot_state);
    this->StateController(&this->robot_state, &this->robot_command);
    this->SetCommand(&this->robot_command);

    auto now = std::chrono::steady_clock::now();
    if (now - last_print_time_ > std::chrono::seconds(1))
    {
        last_print_time_ = now;
        std::cout << "\r\033[K" << LOGGER::INFO
                  << "cmd x:" << this->control.x
                  << " y:" << this->control.y
                  << " yaw:" << this->control.yaw
                  << std::flush;
    }
}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = torch::tensor(this->robot_state.imu.gyroscope).unsqueeze(0);
        if (this->control.navigation_mode)
        {
            this->obs.commands = torch::tensor({{this->control.x, this->control.y, this->control.yaw}});
        }
        else
        {
            this->obs.commands = torch::tensor({{this->control.x, this->control.y, this->control.yaw}});
        }
        this->obs.base_quat = torch::tensor(this->robot_state.imu.quaternion).unsqueeze(0);
        this->obs.dof_pos = torch::tensor(this->robot_state.motor_state.q).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);
        this->obs.dof_vel = torch::tensor(this->robot_state.motor_state.dq).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);

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

#ifdef CSV_LOGGER
        torch::Tensor tau_est = torch::tensor(this->robot_state.motor_state.tau_est).unsqueeze(0);
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
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

static void signalHandler(int signum)
{
    (void)signum;
    std::exit(0);
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    signal(SIGINT, signalHandler);
    RL_Real rl(true);
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }
    return 0;
}
