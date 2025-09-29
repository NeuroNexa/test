/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_titati.hpp"

#include <chrono>
#include <thread>

RL_Real::RL_Real()
#if defined(USE_ROS2) && defined(USE_ROS)
    : rclcpp::Node("rl_real_titati")
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
    last_commanded_torque_.assign(this->params.num_of_dofs, 0.0);
    mapped_joint_positions.assign(this->params.num_of_dofs, 0.0);
    mapped_joint_velocities.assign(this->params.num_of_dofs, 0.0);

    EnableDirectDrive();

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
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<double>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<double>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_Real::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
}

RL_Real::~RL_Real()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    DisableDirectDrive();
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::EnableDirectDrive()
{
    if (!robot_)
    {
        return;
    }

    bool enabled = false;
    for (int attempt = 0; attempt < 5 && !enabled; ++attempt)
    {
        enabled = robot_->set_motors_sdk(true);
        if (!enabled)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    if (!enabled)
    {
        std::cout << LOGGER::WARNING << "Failed to request FORCE_DIRECT mode via SDK." << std::endl;
    }
    else
    {
        std::cout << LOGGER::INFO << "Requested FORCE_DIRECT mode via SDK." << std::endl;
    }
}

void RL_Real::DisableDirectDrive()
{
    if (!robot_)
    {
        return;
    }
    robot_->set_target_joint_t(std::vector<double>(this->params.num_of_dofs, 0.0));
    std::cout << LOGGER::INFO << "Cleared commanded torques; SDK control state left unchanged." << std::endl;
}

void RL_Real::GetState(RobotState<double> *state)
{
    if (!robot_)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(robot_mutex_);
    auto joint_pos = robot_->get_joint_q();
    auto joint_vel = robot_->get_joint_v();
    auto joint_tau = robot_->get_joint_t();
    auto quat_xyzw = robot_->get_imu_quaternion();
    auto imu_acc = robot_->get_imu_acceleration();
    auto imu_gyro = robot_->get_imu_angular_velocity();

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        state->motor_state.q[i] = joint_pos.size() > static_cast<size_t>(i) ? joint_pos[i] : 0.0;
        state->motor_state.dq[i] = joint_vel.size() > static_cast<size_t>(i) ? joint_vel[i] : 0.0;
        state->motor_state.tau_est[i] = joint_tau.size() > static_cast<size_t>(i) ? joint_tau[i] : 0.0;
    }

    state->imu.quaternion[0] = quat_xyzw[3];
    state->imu.quaternion[1] = quat_xyzw[0];
    state->imu.quaternion[2] = quat_xyzw[1];
    state->imu.quaternion[3] = quat_xyzw[2];

    for (int i = 0; i < 3; ++i)
    {
        state->imu.accelerometer[i] = imu_acc[i];
        state->imu.gyroscope[i] = imu_gyro[i];
    }
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    if (!robot_)
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
        auto mapped_index = this->params.joint_mapping[i];
        q[mapped_index] = command->motor_command.q[i];
        dq[mapped_index] = command->motor_command.dq[i];
        kp[mapped_index] = command->motor_command.kp[i];
        kd[mapped_index] = command->motor_command.kd[i];
        tau[mapped_index] = command->motor_command.tau[i];
    }

    {
        std::lock_guard<std::mutex> lock(robot_mutex_);
        robot_->set_target_joint_mit(q, dq, kp, kd, tau);
    }

    last_commanded_torque_ = tau;
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

void RL_Real::Plot()
{
#ifdef PLOT
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->robot_state.motor_state.q[i]);
        this->plot_target_joint_pos[i].push_back(this->robot_command.motor_command.q[i]);
        plt::subplot(this->params.num_of_dofs, 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    plt::pause(0.0001);
#endif
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
    ros::init(argc, argv, "rl_real_titati");
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
