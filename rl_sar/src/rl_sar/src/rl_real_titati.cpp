/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_titati.hpp"

#include <cstring>
#include <unistd.h>

RL_Real::RL_Real(const std::string &robot_variant)
#if defined(USE_ROS2) && defined(USE_ROS)
    : rclcpp::Node("rl_real_node")
    , robot_variant_(robot_variant)
#else
    : robot_variant_(robot_variant)
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

    // read params from yaml
    this->ang_vel_type = "ang_vel_body";
    this->robot_name = this->robot_variant_;
    this->ReadYamlBase(this->robot_name);

    // auto load FSM by robot_name
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

    // init torch
    torch::autograd::GradMode::set_enabled(false);
    torch::set_num_threads(4);

    // init robot
    robot_ = std::make_unique<tita_robot>(static_cast<size_t>(this->params.num_of_dofs));
    if (!robot_->set_motors_sdk(true))
    {
        std::cout << LOGGER::ERROR << "Failed to switch motors to SDK direct-control mode" << std::endl;
    }
    std::vector<double> zero_torque(this->params.num_of_dofs, 0.0);
    robot_->set_target_joint_t(zero_torque);

    this->InitOutputs();
    this->InitControl();

    // loop
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
}

RL_Real::~RL_Real()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    if (this->loop_plot)
    {
        this->loop_plot->shutdown();
    }
#endif
    if (robot_)
    {
        std::vector<double> zero_torque(this->params.num_of_dofs, 0.0);
        robot_->set_target_joint_t(zero_torque);
        robot_->set_motors_sdk(false);
    }
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<double> *state)
{
    if (this->control.navigation_mode)
    {
#if !defined(USE_CMAKE) && defined(USE_ROS)
        this->control.x = this->cmd_vel.linear.x;
        this->control.y = this->cmd_vel.linear.y;
        this->control.yaw = this->cmd_vel.angular.z;
#endif
    }

    auto quat = robot_->get_imu_quaternion();
    state->imu.quaternion[0] = quat[3]; // w
    state->imu.quaternion[1] = quat[0]; // x
    state->imu.quaternion[2] = quat[1]; // y
    state->imu.quaternion[3] = quat[2]; // z

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

    auto joint_q = robot_->get_joint_q();
    auto joint_dq = robot_->get_joint_v();
    auto joint_tau = robot_->get_joint_t();

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        int mapped = this->params.joint_mapping[i];
        state->motor_state.q[i] = joint_q[mapped];
        state->motor_state.dq[i] = joint_dq[mapped];
        state->motor_state.tau_est[i] = joint_tau[mapped];
    }
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    std::vector<double> q(this->params.num_of_dofs, 0.0);
    std::vector<double> dq(this->params.num_of_dofs, 0.0);
    std::vector<double> kp(this->params.num_of_dofs, 0.0);
    std::vector<double> kd(this->params.num_of_dofs, 0.0);
    std::vector<double> tau(this->params.num_of_dofs, 0.0);

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        int mapped = this->params.joint_mapping[i];
        q[mapped] = command->motor_command.q[i];
        dq[mapped] = command->motor_command.dq[i];
        kp[mapped] = command->motor_command.kp[i];
        kd[mapped] = command->motor_command.kd[i];
        tau[mapped] = command->motor_command.tau[i];
    }

    robot_->set_target_joint_mit(q, dq, kp, kd, tau);
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

void RL_Real::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->robot_state.motor_state.q[i]);
        this->plot_target_joint_pos[i].push_back(this->output_dof_pos[0][i].item<double>());
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

static std::string ParseRobotVariant(int argc, char **argv)
{
    std::string variant = "titati";
    for (int i = 1; i < argc; ++i)
    {
        std::string arg(argv[i]);
        if (arg.rfind("--robot=", 0) == 0)
        {
            variant = arg.substr(std::strlen("--robot="));
        }
        else if (arg.rfind("--robot_name=", 0) == 0)
        {
            variant = arg.substr(std::strlen("--robot_name="));
        }
    }
    if (variant != "tita" && variant != "titati")
    {
        std::cout << LOGGER::WARNING << "Unknown robot variant '" << variant << "', fallback to titati" << std::endl;
        variant = "titati";
    }
    return variant;
}

int main(int argc, char **argv)
{
    std::string robot_variant = ParseRobotVariant(argc, argv);
#if defined(USE_ROS1) && defined(USE_ROS)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar");
    RL_Real rl_sar(robot_variant);
    ros::spin();
#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RL_Real>(robot_variant));
    rclcpp::shutdown();
#elif defined(USE_CMAKE) || !defined(USE_ROS)
    RL_Real rl_sar(robot_variant);
    while (1) { sleep(10); }
#endif
    return 0;
}
