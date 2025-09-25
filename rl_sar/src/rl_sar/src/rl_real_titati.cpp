/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_titati.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

RL_RealTitati::RL_RealTitati()
#if defined(USE_ROS2) && defined(USE_ROS)
    : rclcpp::Node("rl_real_titati")
#endif
{
#if defined(USE_ROS1) && defined(USE_ROS)
    ros::NodeHandle nh;
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_RealTitati::CmdvelCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
    this->cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
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

    this->titati_robot_ = std::make_unique<tita_robot>(static_cast<size_t>(this->params.num_of_dofs));
    this->latest_position_.resize(this->params.num_of_dofs, 0.0);
    this->latest_velocity_.resize(this->params.num_of_dofs, 0.0);
    this->latest_torque_.resize(this->params.num_of_dofs, 0.0);

    if (!this->titati_robot_->set_motors_sdk(true))
    {
        std::cout << LOGGER::WARNING << "Failed to switch Titati to SDK control mode." << std::endl;
    }

    if (!this->titati_robot_->wait_for_feedback(std::chrono::milliseconds(500)))
    {
        std::cout << LOGGER::WARNING << "Timed out waiting for Titati motor feedback. Check CAN routing and ensure the CAN router"
                  << " daemon is running." << std::endl;
        auto missing = this->titati_robot_->get_missing_feedback_indices();
        if (!missing.empty())
        {
            std::cout << LOGGER::WARNING << "Motors without feedback:";
            for (auto idx : missing)
            {
                std::cout << ' ' << idx;
            }
            std::cout << std::endl;
        }
    }

    this->InitOutputs();
    this->InitControl();

    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_RealTitati::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&RL_RealTitati::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&RL_RealTitati::RunModel, this));
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.num_of_dofs);
    this->plot_target_joint_pos.resize(this->params.num_of_dofs);
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<double>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<double>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_RealTitati::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
}

RL_RealTitati::~RL_RealTitati()
{
    if (this->loop_keyboard) { this->loop_keyboard->shutdown(); }
    if (this->loop_control) { this->loop_control->shutdown(); }
    if (this->loop_rl) { this->loop_rl->shutdown(); }
#ifdef PLOT
    if (this->loop_plot) { this->loop_plot->shutdown(); }
#endif

    if (this->titati_robot_)
    {
        std::vector<double> zero(this->params.num_of_dofs, 0.0);
        this->titati_robot_->set_target_joint_mit(zero, zero, zero, zero, zero);
    }

    std::cout << LOGGER::INFO << "RL_RealTitati exit" << std::endl;
}

void RL_RealTitati::GetState(RobotState<double> *state)
{
    if (!this->titati_robot_)
    {
        return;
    }

    auto joint_positions = this->titati_robot_->get_joint_q();
    auto joint_velocities = this->titati_robot_->get_joint_v();
    auto joint_torques = this->titati_robot_->get_joint_t();
    auto imu_quat = this->titati_robot_->get_imu_quaternion();
    auto imu_accel = this->titati_robot_->get_imu_acceleration();
    auto imu_gyro = this->titati_robot_->get_imu_angular_velocity();

    if (joint_positions.size() < static_cast<size_t>(this->params.num_of_dofs))
    {
        std::cout << LOGGER::WARNING << "Incomplete joint state received from Titati." << std::endl;
        return;
    }

    std::lock_guard<std::mutex> lock(this->hardware_mutex_);
    this->latest_position_ = joint_positions;
    this->latest_velocity_ = joint_velocities;
    this->latest_torque_ = joint_torques;

    this->latest_quaternion_[0] = imu_quat[3];
    this->latest_quaternion_[1] = imu_quat[0];
    this->latest_quaternion_[2] = imu_quat[1];
    this->latest_quaternion_[3] = imu_quat[2];
    for (size_t i = 0; i < 3; ++i)
    {
        this->latest_accel_[i] = imu_accel[i];
        this->latest_gyro_[i] = imu_gyro[i];
    }

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        int idx = this->params.joint_mapping[i];
        state->motor_state.q[i] = joint_positions[idx];
        state->motor_state.dq[i] = joint_velocities[idx];
        state->motor_state.tau_est[i] = joint_torques[idx];
    }

    state->imu.quaternion = {this->latest_quaternion_[0], this->latest_quaternion_[1], this->latest_quaternion_[2], this->latest_quaternion_[3]};
    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = this->latest_gyro_[i];
        state->imu.accelerometer[i] = this->latest_accel_[i];
    }

    this->state_ready_.store(true);
}

void RL_RealTitati::SetCommand(const RobotCommand<double> *command)
{
    if (!this->titati_robot_)
    {
        return;
    }

    std::vector<double> q(this->params.num_of_dofs, 0.0);
    std::vector<double> dq(this->params.num_of_dofs, 0.0);
    std::vector<double> kp(this->params.num_of_dofs, 0.0);
    std::vector<double> kd(this->params.num_of_dofs, 0.0);
    std::vector<double> tau(this->params.num_of_dofs, 0.0);

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        int idx = this->params.joint_mapping[i];
        q[idx] = command->motor_command.q[i];
        dq[idx] = command->motor_command.dq[i];
        kp[idx] = command->motor_command.kp[i];
        kd[idx] = command->motor_command.kd[i];
        tau[idx] = command->motor_command.tau[i];
    }

    this->titati_robot_->set_target_joint_mit(q, dq, kp, kd, tau);
}

void RL_RealTitati::RobotControl()
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
        this->control.x = 0;
        this->control.y = 0;
        this->control.yaw = 0;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::N || this->control.current_gamepad == Input::Gamepad::X)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << std::endl << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
        this->control.current_keyboard = this->control.last_keyboard;
    }

    this->GetState(&this->robot_state);
    this->StateController(&this->robot_state, &this->robot_command);
    this->SetCommand(&this->robot_command);
}

void RL_RealTitati::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = torch::tensor(this->robot_state.imu.gyroscope).unsqueeze(0);
        if (this->control.navigation_mode)
        {
#if !defined(USE_CMAKE) && defined(USE_ROS)
            this->obs.commands = torch::tensor({{this->cmd_vel.linear.x, this->cmd_vel.linear.y, this->cmd_vel.angular.z}});
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

        this->TorqueProtect(this->output_dof_tau);

#ifdef CSV_LOGGER
        torch::Tensor tau_est = torch::tensor(this->robot_state.motor_state.tau_est).unsqueeze(0);
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

torch::Tensor RL_RealTitati::Forward()
{
    torch::autograd::GradMode::set_enabled(false);

    torch::Tensor clamped_obs = this->ComputeObservation();

    torch::Tensor actions;
    if (!this->params.observations_history.empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.observations_history);
        if (this->fsm.current_state_->GetStateName() != "RLFSMStateRL_LocomotionLab")
        {
            torch::Tensor myTensor = history_obs.view({1, 10, 57});
            actions = this->model.forward({clamped_obs, myTensor}).toTensor();
        }
        else
        {
            actions = this->model.forward({this->history_obs}).toTensor();
        }
    }
    else
    {
        actions = this->model.forward({clamped_obs}).toTensor();
    }

    if (this->params.clip_actions_upper.numel() != 0 && this->params.clip_actions_lower.numel() != 0)
    {
        return torch::clamp(actions, this->params.clip_actions_lower, this->params.clip_actions_upper);
    }
    else
    {
        return actions;
    }
}

void RL_RealTitati::Plot()
{
#ifdef PLOT
    if (!this->state_ready_.load())
    {
        return;
    }

    std::lock_guard<std::mutex> lock(this->hardware_mutex_);

    for (size_t i = 0; i < this->plot_size - 1; ++i)
    {
        this->plot_t[i] = this->plot_t[i + 1];
    }
    this->plot_t[this->plot_size - 1] += 1;

    for (int joint = 0; joint < this->params.num_of_dofs; ++joint)
    {
        for (size_t i = 0; i < this->plot_size - 1; ++i)
        {
            this->plot_real_joint_pos[joint][i] = this->plot_real_joint_pos[joint][i + 1];
            this->plot_target_joint_pos[joint][i] = this->plot_target_joint_pos[joint][i + 1];
        }
        this->plot_real_joint_pos[joint][this->plot_size - 1] = this->latest_position_[joint];
        this->plot_target_joint_pos[joint][this->plot_size - 1] = this->robot_command.motor_command.q[joint];

        plt::clf();
        plt::subplot(this->params.num_of_dofs, 1, joint + 1);
        plt::named_plot("real", this->plot_t, this->plot_real_joint_pos[joint]);
        plt::named_plot("target", this->plot_t, this->plot_target_joint_pos[joint]);
        plt::legend();
        if (joint == 0)
        {
            plt::title("Joint position tracking");
        }
        if (joint == this->params.num_of_dofs - 1)
        {
            plt::xlabel("Time");
        }
    }
    plt::pause(0.001);
#endif
}

#if defined(USE_ROS1) && defined(USE_ROS)
void RL_RealTitati::CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    this->cmd_vel = *msg;
}
#elif defined(USE_ROS2) && defined(USE_ROS)
void RL_RealTitati::CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->cmd_vel = *msg;
}
#endif

#if defined(USE_ROS1) && defined(USE_ROS)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(signum);
}
#endif

int main(int argc, char **argv)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_real_titati");
    RL_RealTitati rl_node;
    ros::spin();
#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RL_RealTitati>());
    rclcpp::shutdown();
#else
    RL_RealTitati rl_node;
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }
#endif
    return 0;
}
