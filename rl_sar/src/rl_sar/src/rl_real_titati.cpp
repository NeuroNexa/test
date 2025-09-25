/*
 * Copyright (c) 2024-2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_titati.hpp"

#include <chrono>
#include <thread>
#include <unistd.h>

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

    this->titati_robot_ = std::make_unique<tita_robot>(this->params.num_of_dofs);
    if (!this->titati_robot_->set_motors_sdk(true))
    {
        std::cout << LOGGER::WARNING << "Failed to switch Titati motors to SDK direct mode." << std::endl;
    }

    this->mapped_joint_positions_.resize(this->params.num_of_dofs, 0.0);
    this->mapped_joint_velocities_.resize(this->params.num_of_dofs, 0.0);
    this->command_q_.resize(this->params.num_of_dofs, 0.0);
    this->command_dq_.resize(this->params.num_of_dofs, 0.0);
    this->command_kp_.resize(this->params.num_of_dofs, 0.0);
    this->command_kd_.resize(this->params.num_of_dofs, 0.0);
    this->command_tau_.resize(this->params.num_of_dofs, 0.0);

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
    if (this->loop_keyboard) this->loop_keyboard->shutdown();
    if (this->loop_control) this->loop_control->shutdown();
    if (this->loop_rl) this->loop_rl->shutdown();
#ifdef PLOT
    if (this->loop_plot) this->loop_plot->shutdown();
#endif

    try
    {
        std::vector<double> zero(this->params.num_of_dofs, 0.0);
        if (this->titati_robot_)
        {
            this->titati_robot_->set_target_joint_t(zero);
            this->titati_robot_->set_motors_sdk(false);
        }
    }
    catch (const std::exception &e)
    {
        std::cout << LOGGER::WARNING << "Failed to send zero command during shutdown: " << e.what() << std::endl;
    }

    std::cout << LOGGER::INFO << "RL_Real Titati exit" << std::endl;
}

void RL_Real::GetState(RobotState<double> *state)
{
    constexpr double two_pi = 6.28318530717958647692;
    auto joint_positions = this->titati_robot_->get_joint_q();
    auto joint_velocities = this->titati_robot_->get_joint_v();
    auto joint_torques = this->titati_robot_->get_joint_t();
    auto joint_status = this->titati_robot_->get_joint_status();

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        int mapped = this->params.joint_mapping[i];
        double position = joint_positions[mapped];
        if ((i == 1 || i == 1 + this->params.num_of_dofs / 2) && position < -2.5)
        {
            position += two_pi;
        }
        state->motor_state.q[i] = position;
        state->motor_state.dq[i] = joint_velocities[mapped];
        state->motor_state.tau_est[i] = joint_torques[mapped];
        state->motor_state.cur[i] = static_cast<double>(joint_status[mapped % joint_status.size()]);
    }

    auto quat = this->titati_robot_->get_imu_quaternion(); // x y z w
    state->imu.quaternion[0] = quat[3];
    state->imu.quaternion[1] = quat[0];
    state->imu.quaternion[2] = quat[1];
    state->imu.quaternion[3] = quat[2];

    auto gyro = this->titati_robot_->get_imu_angular_velocity();
    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = gyro[i];
    }

    auto accl = this->titati_robot_->get_imu_acceleration();
    for (int i = 0; i < 3; ++i)
    {
        state->imu.accelerometer[i] = accl[i];
    }
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        int mapped = this->params.joint_mapping[i];
        this->command_q_[mapped] = command->motor_command.q[i];
        this->command_dq_[mapped] = command->motor_command.dq[i];
        this->command_kp_[mapped] = command->motor_command.kp[i];
        this->command_kd_[mapped] = command->motor_command.kd[i];
        this->command_tau_[mapped] = command->motor_command.tau[i];
    }

    this->titati_robot_->set_target_joint_mit(this->command_q_, this->command_dq_, this->command_kp_, this->command_kd_, this->command_tau_);
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
            torch::Tensor myTensor = history_obs.view({1, 10, this->params.num_observations});
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

#ifdef PLOT
void RL_Real::Plot()
{
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->robot_state.motor_state.q[i]);
        this->plot_target_joint_pos[i].push_back(this->output_dof_pos[0][i].item<double>());
    }
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->plot_t.back() + 1);

    plt::clf();
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        plt::subplot(4, 4, i + 1);
        plt::plot(this->plot_t, this->plot_real_joint_pos[i], "b");
        plt::plot(this->plot_t, this->plot_target_joint_pos[i], "r--");
    }
    plt::pause(0.001);
}
#endif

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
    while (1)
    {
        sleep(10);
    }
#endif
    return 0;
}

