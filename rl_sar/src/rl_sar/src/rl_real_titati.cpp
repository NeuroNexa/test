/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_titati.hpp"

#include <iostream>

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

    robot_ = std::make_unique<tita_robot>(this->params.num_of_dofs);
    motors_sdk_enabled_ = robot_->set_motors_sdk(true);
    if (!motors_sdk_enabled_)
    {
        std::cout << LOGGER::WARNING << "Failed to switch Titati motors to SDK control mode. Will retry automatically." << std::endl;
        last_sdk_retry_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    }
    else
    {
        last_sdk_retry_ = std::chrono::steady_clock::now();
    }

    joint_positions_.assign(this->params.num_of_dofs, 0.0);
    joint_velocities_.assign(this->params.num_of_dofs, 0.0);
    joint_torques_.assign(this->params.num_of_dofs, 0.0);

    this->InitOutputs();
    this->InitControl();

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
    if (robot_ && motors_sdk_enabled_)
    {
        robot_->set_target_joint_t(std::vector<double>(this->params.num_of_dofs, 0.0));
        robot_->set_motors_sdk(false);
    }

    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<double> *state)
{
    auto q = robot_->get_joint_q();
    auto v = robot_->get_joint_v();
    auto tau = robot_->get_joint_t();

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        joint_positions_[i] = q[this->params.joint_mapping[i]];
        joint_velocities_[i] = v[this->params.joint_mapping[i]];
        joint_torques_[i] = tau[this->params.joint_mapping[i]];
        state->motor_state.q[i] = joint_positions_[i];
        state->motor_state.dq[i] = joint_velocities_[i];
        state->motor_state.tau_est[i] = joint_torques_[i];
    }

    auto quat = robot_->get_imu_quaternion();
    state->imu.quaternion[0] = quat[3];
    state->imu.quaternion[1] = quat[0];
    state->imu.quaternion[2] = quat[1];
    state->imu.quaternion[3] = quat[2];

    auto gyro = robot_->get_imu_angular_velocity();
    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = gyro[i];
    }

    auto accl = robot_->get_imu_acceleration();
    for (int i = 0; i < 3; ++i)
    {
        state->imu.accelerometer[i] = accl[i];
    }
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    if (!EnsureMotorsSdkMode())
    {
        return;
    }

    std::vector<double> q;
    std::vector<double> v;
    std::vector<double> kp;
    std::vector<double> kd;
    std::vector<double> tau;
    q.reserve(this->params.num_of_dofs);
    v.reserve(this->params.num_of_dofs);
    kp.reserve(this->params.num_of_dofs);
    kd.reserve(this->params.num_of_dofs);
    tau.reserve(this->params.num_of_dofs);

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        q.push_back(command->motor_command.q[i]);
        v.push_back(command->motor_command.dq[i]);
        kp.push_back(command->motor_command.kp[i]);
        kd.push_back(command->motor_command.kd[i]);
        tau.push_back(command->motor_command.tau[i]);
    }

    if (!robot_->set_target_joint_mit(q, v, kp, kd, tau) && motors_sdk_enabled_)
    {
        motors_sdk_enabled_ = false;
        last_sdk_retry_ = std::chrono::steady_clock::now();
        std::cout << LOGGER::WARNING << "Failed to send MIT command to Titati motors. Retrying SDK handshake." << std::endl;
    }
}

bool RL_Real::EnsureMotorsSdkMode()
{
    if (motors_sdk_enabled_)
    {
        return true;
    }

    const auto now = std::chrono::steady_clock::now();
    constexpr auto kRetryInterval = std::chrono::milliseconds(200);
    if (now - last_sdk_retry_ < kRetryInterval)
    {
        return false;
    }

    last_sdk_retry_ = now;
    if (robot_->set_motors_sdk(true))
    {
        motors_sdk_enabled_ = true;
        std::cout << LOGGER::INFO << "Titati motors switched to SDK control mode." << std::endl;
        robot_->set_target_joint_t(std::vector<double>(this->params.num_of_dofs, 0.0));
        return true;
    }

    std::cout << LOGGER::WARNING << "Retrying to switch Titati motors to SDK control mode..." << std::endl;
    return false;
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
        this->control.x = 0;
        this->control.y = 0;
        this->control.yaw = 0;
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
    else
    {
        return actions;
    }
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
    while (1) { sleep(10); }
#endif
    return 0;
}
