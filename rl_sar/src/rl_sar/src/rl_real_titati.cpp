/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_titati.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <thread>

#if defined(USE_ROS1) && defined(USE_ROS)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(signum);
}
#endif

RL_Real::RL_Real(const std::string &can_interface)
#if defined(USE_ROS2) && defined(USE_ROS)
    : rclcpp::Node("rl_real_node")
#endif
    , can_interface_(can_interface)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    ros::NodeHandle nh;
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real::CmdvelCallback, this);
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

    this->InitOutputs();
    this->InitControl();

    this->robot = std::make_unique<tita_robot>(this->params.num_of_dofs, this->can_interface_);
    this->InitRobot();

    this->raw_joint_positions_.resize(this->params.num_of_dofs, 0.0);
    this->raw_joint_velocities_.resize(this->params.num_of_dofs, 0.0);
    this->raw_joint_torques_.resize(this->params.num_of_dofs, 0.0);
    this->command_position_buffer_.resize(this->params.num_of_dofs, 0.0);
    this->command_velocity_buffer_.resize(this->params.num_of_dofs, 0.0);
    this->command_kp_buffer_.resize(this->params.num_of_dofs, 0.0);
    this->command_kd_buffer_.resize(this->params.num_of_dofs, 0.0);
    this->command_tau_buffer_.resize(this->params.num_of_dofs, 0.0);

    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&RL_Real::RunModel, this));
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
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
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::InitRobot()
{
    if (!this->robot)
    {
        throw std::runtime_error("tita_robot interface is not initialised");
    }
    this->direct_mode_enabled_ = this->robot->set_motors_sdk(true);
    if (!this->direct_mode_enabled_)
    {
        std::cout << LOGGER::WARN << "Failed to enable direct control mode on interface " << this->can_interface_ << std::endl;
    }
    if (!this->WaitForInitialState())
    {
        std::cout << LOGGER::WARN << "No valid feedback received from Titati on interface " << this->can_interface_ << std::endl;
    }
    else
    {
        std::cout << LOGGER::INFO << "Titati hardware interface ready on " << this->can_interface_ << std::endl;
    }
}

bool RL_Real::WaitForInitialState()
{
    constexpr int max_attempts = 400;
    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        auto q = this->robot->get_joint_q();
        auto quat = this->robot->get_imu_quaternion();
        if (q.size() == static_cast<size_t>(this->params.num_of_dofs))
        {
            this->raw_joint_positions_ = q;
            this->raw_joint_velocities_ = this->robot->get_joint_v();
            this->raw_joint_torques_ = this->robot->get_joint_t();
            this->raw_quaternion_ = quat;
            this->raw_gyro_ = this->robot->get_imu_angular_velocity();
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

void RL_Real::GetState(RobotState<double> *state)
{
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    this->raw_joint_positions_ = this->robot->get_joint_q();
    this->raw_joint_velocities_ = this->robot->get_joint_v();
    this->raw_joint_torques_ = this->robot->get_joint_t();
    this->raw_quaternion_ = this->robot->get_imu_quaternion();
    this->raw_gyro_ = this->robot->get_imu_angular_velocity();

    if (this->raw_joint_positions_.size() != static_cast<size_t>(this->params.num_of_dofs))
    {
        std::cout << LOGGER::WARN << "Unexpected joint state size: " << this->raw_joint_positions_.size() << std::endl;
        return;
    }

    state->imu.quaternion[0] = this->raw_quaternion_[3];
    state->imu.quaternion[1] = this->raw_quaternion_[0];
    state->imu.quaternion[2] = this->raw_quaternion_[1];
    state->imu.quaternion[3] = this->raw_quaternion_[2];

    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = this->raw_gyro_[i];
    }

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        const int hw_index = this->params.joint_mapping[i];
        state->motor_state.q[i] = this->raw_joint_positions_[hw_index];
        state->motor_state.dq[i] = this->raw_joint_velocities_[hw_index];
        state->motor_state.tau_est[i] = this->raw_joint_torques_[hw_index];
    }
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    std::fill(this->command_position_buffer_.begin(), this->command_position_buffer_.end(), 0.0);
    std::fill(this->command_velocity_buffer_.begin(), this->command_velocity_buffer_.end(), 0.0);
    std::fill(this->command_kp_buffer_.begin(), this->command_kp_buffer_.end(), 0.0);
    std::fill(this->command_kd_buffer_.begin(), this->command_kd_buffer_.end(), 0.0);
    std::fill(this->command_tau_buffer_.begin(), this->command_tau_buffer_.end(), 0.0);

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        const int hw_index = this->params.joint_mapping[i];
        this->command_position_buffer_[hw_index] = command->motor_command.q[i];
        this->command_velocity_buffer_[hw_index] = command->motor_command.dq[i];
        this->command_kp_buffer_[hw_index] = command->motor_command.kp[i];
        this->command_kd_buffer_[hw_index] = command->motor_command.kd[i];
        this->command_tau_buffer_[hw_index] = command->motor_command.tau[i];
    }

    if (!this->robot->set_target_joint_mit(
            this->command_position_buffer_, this->command_velocity_buffer_,
            this->command_kp_buffer_, this->command_kd_buffer_, this->command_tau_buffer_))
    {
        std::cout << LOGGER::WARN << "Failed to send joint command over CAN interface " << this->can_interface_ << std::endl;
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
        std::cout << std::endl << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
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
#else
        this->obs.commands = torch::tensor({{0.0, 0.0, 0.0}});
#endif
    }
    else
    {
        this->obs.commands = torch::tensor({{this->control.x, this->control.y, this->control.yaw}});
    }
    this->obs.base_quat = torch::tensor(this->robot_state.imu.quaternion).unsqueeze(0);
    this->obs.dof_pos = torch::tensor(this->robot_state.motor_state.q).unsqueeze(0);
    this->obs.dof_vel = torch::tensor(this->robot_state.motor_state.dq).unsqueeze(0);

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

#if !defined(USE_CMAKE) && defined(USE_ROS)
#if defined(USE_ROS1) && defined(USE_ROS)
void RL_Real::CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    this->cmd_vel = *msg;
}
#elif defined(USE_ROS2) && defined(USE_ROS)
void RL_Real::CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->cmd_vel = *msg;
}
#endif
#endif

int main(int argc, char **argv)
{
    std::string can_interface = "can0";
    if (argc > 1)
    {
        can_interface = argv[1];
    }

#if defined(USE_ROS1) && defined(USE_ROS)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar");
    RL_Real rl_sar(can_interface);
    ros::spin();
#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RL_Real>(can_interface));
    rclcpp::shutdown();
#elif defined(USE_CMAKE) || !defined(USE_ROS)
    RL_Real rl_sar(can_interface);
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }
#endif
    return 0;
}
