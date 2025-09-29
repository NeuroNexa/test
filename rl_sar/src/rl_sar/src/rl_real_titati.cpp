/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_titati.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <iostream>

namespace
{
constexpr int kMaxAcquireAttempts = 10;
constexpr double kKeyboardStep = 0.1;
constexpr double kYawStep = 0.1;
constexpr double kCommandLimit = 1.0;
} // namespace

RL_Real::RL_Real()
#if defined(USE_ROS2) && defined(USE_ROS)
    : rclcpp::Node("rl_real_titati")
#endif
{
#if defined(USE_ROS1) && defined(USE_ROS)
    ros::NodeHandle nh;
    this->cmd_vel_subscriber_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real::CmdvelCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
    this->cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->CmdvelCallback(msg); });
#endif

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

    const int num_dofs = this->params.num_of_dofs;
    this->robot_ = std::make_unique<tita_robot>(static_cast<size_t>(num_dofs));
    this->command_tau_.assign(num_dofs, 0.0);

    this->InitOutputs();
    this->InitControl();

    if (this->AcquireSDKControl())
    {
        std::cout << LOGGER::INFO << "Titati SDK control acquired." << std::endl;
    }
    else
    {
        std::cout << LOGGER::WARNING << "Failed to acquire Titati SDK control on startup." << std::endl;
    }

    this->loop_keyboard_ = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control_ = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&RL_Real::RobotControl, this));
    this->loop_rl_ = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation,
                                                std::bind(&RL_Real::RunModel, this));

    this->loop_keyboard_->start();
    this->loop_control_->start();
    this->loop_rl_->start();

#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
}

RL_Real::~RL_Real()
{
    if (this->loop_keyboard_) this->loop_keyboard_->shutdown();
    if (this->loop_control_) this->loop_control_->shutdown();
    if (this->loop_rl_) this->loop_rl_->shutdown();

    this->ReleaseSDKControl();
    std::cout << LOGGER::INFO << "RL_Real Titati exit" << std::endl;
}

void RL_Real::RobotControl()
{
    if (this->control.current_keyboard == Input::Keyboard::W)
    {
        this->control.x = clamp(this->control.x + kKeyboardStep, -kCommandLimit, kCommandLimit);
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::S)
    {
        this->control.x = clamp(this->control.x - kKeyboardStep, -kCommandLimit, kCommandLimit);
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::A)
    {
        this->control.y = clamp(this->control.y + kKeyboardStep, -kCommandLimit, kCommandLimit);
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::D)
    {
        this->control.y = clamp(this->control.y - kKeyboardStep, -kCommandLimit, kCommandLimit);
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Q)
    {
        this->control.yaw = clamp(this->control.yaw + kYawStep, -kCommandLimit, kCommandLimit);
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::E)
    {
        this->control.yaw = clamp(this->control.yaw - kYawStep, -kCommandLimit, kCommandLimit);
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Space)
    {
        this->control.x = 0.0;
        this->control.y = 0.0;
        this->control.yaw = 0.0;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::N ||
        this->control.current_gamepad == Input::Gamepad::X)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << std::endl
                  << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF")
                  << std::endl;
        this->control.current_keyboard = this->control.last_keyboard;
    }

    this->GetState(&this->robot_state);
    this->StateController(&this->robot_state, &this->robot_command);
    this->SetCommand(&this->robot_command);
}

void RL_Real::RunModel()
{
    if (!this->rl_init_done)
    {
        return;
    }

    this->episode_length_buf += 1;

    this->obs.ang_vel = torch::tensor(this->robot_state.imu.gyroscope).unsqueeze(0);

    if (this->control.navigation_mode)
    {
#if !defined(USE_CMAKE) && defined(USE_ROS)
        this->obs.commands = torch::tensor({{this->cmd_vel_.linear.x, this->cmd_vel_.linear.y, this->cmd_vel_.angular.z}});
#else
        this->obs.commands = torch::tensor({{0.0, 0.0, 0.0}});
#endif
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

#ifdef CSV_LOGGER
    torch::Tensor tau_est = torch::tensor(this->robot_state.motor_state.tau_est).unsqueeze(0);
    this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
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

void RL_Real::GetState(RobotState<double>* state)
{
    if (!this->robot_)
    {
        return;
    }

    auto quaternion_xyzw = this->robot_->get_imu_quaternion();
    state->imu.quaternion[0] = quaternion_xyzw[3];
    state->imu.quaternion[1] = quaternion_xyzw[0];
    state->imu.quaternion[2] = quaternion_xyzw[1];
    state->imu.quaternion[3] = quaternion_xyzw[2];

    auto gyro = this->robot_->get_imu_angular_velocity();
    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = gyro[i];
    }

    auto accl = this->robot_->get_imu_acceleration();
    for (int i = 0; i < 3; ++i)
    {
        state->imu.accelerometer[i] = accl[i];
    }

    auto joint_q = this->robot_->get_joint_q();
    auto joint_dq = this->robot_->get_joint_v();
    auto joint_tau = this->robot_->get_joint_t();

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        const int physical_index = this->params.joint_mapping[i];
        if (physical_index >= 0 && physical_index < static_cast<int>(joint_q.size()))
        {
            state->motor_state.q[i] = joint_q[physical_index];
        }
        if (physical_index >= 0 && physical_index < static_cast<int>(joint_dq.size()))
        {
            state->motor_state.dq[i] = joint_dq[physical_index];
        }
        if (physical_index >= 0 && physical_index < static_cast<int>(joint_tau.size()))
        {
            state->motor_state.tau_est[i] = joint_tau[physical_index];
        }
    }
}

void RL_Real::SetCommand(const RobotCommand<double>* command)
{
    if (!EnsureSDKControl())
    {
        return;
    }

    std::fill(this->command_tau_.begin(), this->command_tau_.end(), 0.0);

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        const int physical_index = this->params.joint_mapping[i];
        if (physical_index >= 0 && physical_index < static_cast<int>(this->command_tau_.size()))
        {
            const double feedforward = command->motor_command.tau[i];
            const double kp = command->motor_command.kp[i];
            const double kd = command->motor_command.kd[i];
            const double desired_q = command->motor_command.q[i];
            const double desired_dq = command->motor_command.dq[i];
            const double measured_q = this->robot_state.motor_state.q[i];
            const double measured_dq = this->robot_state.motor_state.dq[i];
            const double torque = feedforward + kp * (desired_q - measured_q) +
                                  kd * (desired_dq - measured_dq);
            this->command_tau_[physical_index] = torque;
        }
    }

    if (!this->robot_->set_target_joint_t(this->command_tau_))
    {
        std::cout << LOGGER::WARNING << "Failed to send torque command. Will retry acquiring SDK control." << std::endl;
        this->sdk_enabled_.store(false);
    }
}

bool RL_Real::AcquireSDKControl()
{
    bool success = false;
    for (int attempt = 0; attempt < kMaxAcquireAttempts; ++attempt)
    {
        success = this->robot_->set_motors_sdk(true);
        if (success)
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    this->sdk_enabled_.store(success);
    return success;
}

void RL_Real::ReleaseSDKControl()
{
    if (!this->robot_)
    {
        return;
    }

    std::vector<double> zero_torque(this->command_tau_.size(), 0.0);
    this->robot_->set_target_joint_t(zero_torque);

    bool disabled = false;
    for (int attempt = 0; attempt < kMaxAcquireAttempts; ++attempt)
    {
        disabled = this->robot_->set_motors_sdk(false);
        if (disabled)
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    this->sdk_enabled_.store(false);
}

bool RL_Real::EnsureSDKControl()
{
    if (this->sdk_enabled_.load())
    {
        return true;
    }
    return this->AcquireSDKControl();
}

#if defined(USE_ROS1) && defined(USE_ROS)
void RL_Real::CmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    this->cmd_vel_ = *msg;
}
#elif defined(USE_ROS2) && defined(USE_ROS)
void RL_Real::CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->cmd_vel_ = *msg;
}
#endif

#if defined(USE_ROS1) && defined(USE_ROS)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(0);
}
#endif

int main(int argc, char** argv)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_real_titati");
    RL_Real rl_sar;
    ros::spin();
#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RL_Real>());
    rclcpp::shutdown();
#elif defined(USE_CMAKE) || !defined(USE_ROS)
    RL_Real rl_sar;
    while (true)
    {
        sleep(10);
    }
#endif
    return 0;
}