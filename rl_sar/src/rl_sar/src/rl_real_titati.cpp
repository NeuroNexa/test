/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_titati.hpp"

#include <iostream>
#include <chrono>
#include <thread>

namespace
{
inline std::vector<double> make_vector_with_size(std::size_t size, double value = 0.0)
{
    return std::vector<double>(size, value);
}
}

RL_Real::RL_Real()
#if defined(USE_ROS2) && defined(USE_ROS)
    : rclcpp::Node("rl_real_node")
#endif
{
#if defined(USE_ROS1) && defined(USE_ROS)
    ros::NodeHandle nh;
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real::CmdvelCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
    this->cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->CmdvelCallback(msg); }
    );
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

    this->titati_robot_ = std::make_unique<tita_robot>(this->params.num_of_dofs);
    if (!this->titati_robot_->set_motors_sdk(true))
    {
        std::cerr << LOGGER::WARNING << "Failed to switch Titati to SDK control mode." << std::endl;
    }

    this->mapped_joint_positions_ = make_vector_with_size(this->params.num_of_dofs);
    this->mapped_joint_velocities_ = make_vector_with_size(this->params.num_of_dofs);
    this->last_command_positions_ = make_vector_with_size(this->params.num_of_dofs);
    this->last_command_velocities_ = make_vector_with_size(this->params.num_of_dofs);
    this->last_command_torques_ = make_vector_with_size(this->params.num_of_dofs);
    this->last_command_kp_ = make_vector_with_size(this->params.num_of_dofs);
    this->last_command_kd_ = make_vector_with_size(this->params.num_of_dofs);

    this->InitOutputs();
    this->InitControl();

    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&RL_Real::RunModel, this));

    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.num_of_dofs);
    this->plot_target_joint_pos.resize(this->params.num_of_dofs);
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<double>(this->plot_size, 0.0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<double>(this->plot_size, 0.0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_Real::Plot, this));
    this->loop_plot->start();
#endif

#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO << "RL_Real Titati initialized" << std::endl;
}

RL_Real::~RL_Real()
{
    if (this->loop_keyboard) this->loop_keyboard->shutdown();
    if (this->loop_control) this->loop_control->shutdown();
    if (this->loop_rl) this->loop_rl->shutdown();
#ifdef PLOT
    if (this->loop_plot) this->loop_plot->shutdown();
#endif

    if (this->titati_robot_)
    {
        std::vector<double> zero_cmd(this->params.num_of_dofs, 0.0);
        this->titati_robot_->set_target_joint_t(zero_cmd);
        this->titati_robot_->set_motors_sdk(false);
    }

    std::cout << LOGGER::INFO << "RL_Real Titati exit" << std::endl;
}

void RL_Real::GetState(RobotState<double> *state)
{
    auto joint_positions = this->titati_robot_->get_joint_q();
    auto joint_velocities = this->titati_robot_->get_joint_v();
    auto joint_torques = this->titati_robot_->get_joint_t();
    auto imu_quat = this->titati_robot_->get_imu_quaternion();
    auto imu_acc = this->titati_robot_->get_imu_acceleration();
    auto imu_gyro = this->titati_robot_->get_imu_angular_velocity();

    if (joint_positions.size() != this->params.num_of_dofs ||
        joint_velocities.size() != this->params.num_of_dofs ||
        joint_torques.size() != this->params.num_of_dofs)
    {
        std::cerr << LOGGER::WARNING << "Titati returned unexpected joint vector sizes" << std::endl;
        return;
    }

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        int mapped_index = this->params.joint_mapping[i];
        this->mapped_joint_positions_[i] = joint_positions[mapped_index];
        this->mapped_joint_velocities_[i] = joint_velocities[mapped_index];
    }

    state->imu.quaternion[0] = imu_quat[3];
    state->imu.quaternion[1] = imu_quat[0];
    state->imu.quaternion[2] = imu_quat[1];
    state->imu.quaternion[3] = imu_quat[2];

    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = imu_gyro[i];
        state->imu.accelerometer[i] = imu_acc[i];
    }

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        state->motor_state.q[i] = this->mapped_joint_positions_[i];
        state->motor_state.dq[i] = this->mapped_joint_velocities_[i];
        state->motor_state.tau_est[i] = joint_torques[this->params.joint_mapping[i]];
    }
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        int mapped_index = this->params.joint_mapping[i];
        this->last_command_positions_[mapped_index] = command->motor_command.q[i];
        this->last_command_velocities_[mapped_index] = command->motor_command.dq[i];
        this->last_command_torques_[mapped_index] = command->motor_command.tau[i];
        this->last_command_kp_[mapped_index] = command->motor_command.kp[i];
        this->last_command_kd_[mapped_index] = command->motor_command.kd[i];
    }

    this->titati_robot_->set_target_joint_mit(
        this->last_command_positions_,
        this->last_command_velocities_,
        this->last_command_kp_,
        this->last_command_kd_,
        this->last_command_torques_);
}

void RL_Real::RobotControl()
{
    this->motiontime_++;

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
    if (this->control.current_keyboard == Input::Keyboard::N || this->control.current_gamepad == Input::Gamepad::X)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << std::endl << LOGGER::INFO << "Navigation mode: "
                  << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
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

torch::Tensor RL_Real::Forward()
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
            int history_len = static_cast<int>(this->params.observations_history.size());
            if (history_len > 0)
            {
                int feature_dim = static_cast<int>(this->history_obs.size(-1) / history_len);
                torch::Tensor shaped = this->history_obs.view({1, history_len, feature_dim});
                actions = this->model.forward({clamped_obs, shaped}).toTensor();
            }
            else
            {
                actions = this->model.forward({clamped_obs}).toTensor();
            }
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
    return actions;
}

void RL_Real::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime_);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->mapped_joint_positions_[i]);
        this->plot_target_joint_pos[i].push_back(this->last_command_positions_[this->params.joint_mapping[i]]);
        plt::subplot(this->params.num_of_dofs, 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    plt::pause(0.0001);
}

#if !defined(USE_CMAKE) && defined(USE_ROS)
void RL_Real::CmdvelCallback(
#if defined(USE_ROS1) && defined(USE_ROS)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2) && defined(USE_ROS)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
    this->cmd_vel = *msg;
}
#endif

#if defined(USE_ROS1) && defined(USE_ROS)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(0);
}
#endif

int main(int argc, char **argv)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar");
    RL_Real rl_sar;
    ros::spin();
#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RL_Real>());
    rclcpp::shutdown();
#elif defined(USE_CMAKE) || !defined(USE_ROS)
    RL_Real rl_sar;
    while (1) { std::this_thread::sleep_for(std::chrono::seconds(10)); }
#endif
    return 0;
}
