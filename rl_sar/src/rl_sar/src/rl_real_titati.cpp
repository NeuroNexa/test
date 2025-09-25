/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_titati.hpp"

#include <thread>

RL_Real::RL_Real()
#if defined(USE_ROS2) && defined(USE_ROS)
    : rclcpp::Node("rl_real_node")
#endif
{
#if defined(USE_ROS1) && defined(USE_ROS)
    ros::NodeHandle nh;
    this->robot_name = "titati";
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real::CmdvelCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
    this->declare_parameter<std::string>("robot_name", "titati");
    this->robot_name = this->get_parameter("robot_name").as_string();
    this->cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->CmdvelCallback(msg); });
#else
    this->robot_name = "titati";
#endif

#if defined(USE_ROS1) && defined(USE_ROS)
    nh.param<std::string>("robot_name", this->robot_name, "titati");
#endif

    this->ang_vel_type = "ang_vel_body";
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

    motor_count_ = static_cast<size_t>(this->params.num_of_dofs);
    titati_robot_ = std::make_unique<tita_robot>(motor_count_);
    if (!titati_robot_->set_motors_sdk(true))
    {
        std::cout << LOGGER::WARNING << "Failed to switch Titati MCU to SDK mode" << std::endl;
    }

    this->InitOutputs();
    this->InitControl();

    mapped_joint_positions.resize(this->params.num_of_dofs, 0.0);
    mapped_joint_velocities.resize(this->params.num_of_dofs, 0.0);

    loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&RL_Real::RobotControl, this));
    loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&RL_Real::RunModel, this));
    loop_keyboard->start();
    loop_control->start();
    loop_rl->start();

#ifdef PLOT
    plot_t = std::vector<int>(plot_size, 0);
    plot_real_joint_pos.resize(this->params.num_of_dofs);
    plot_target_joint_pos.resize(this->params.num_of_dofs);
    for (auto &vector : plot_real_joint_pos) { vector = std::vector<double>(plot_size, 0); }
    for (auto &vector : plot_target_joint_pos) { vector = std::vector<double>(plot_size, 0); }
    loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_Real::Plot, this));
    loop_plot->start();
#endif

#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
}

RL_Real::~RL_Real()
{
    loop_keyboard->shutdown();
    loop_control->shutdown();
    loop_rl->shutdown();
#ifdef PLOT
    loop_plot->shutdown();
#endif

    if (titati_robot_)
    {
        std::vector<double> zero(motor_count_, 0.0);
        titati_robot_->set_target_joint_t(zero);
        titati_robot_->set_motors_sdk(false);
    }

    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<double> *state)
{
    const auto positions = titati_robot_->get_joint_q();
    const auto velocities = titati_robot_->get_joint_v();
    const auto torques = titati_robot_->get_joint_t();
    const auto quat_xyz_w = titati_robot_->get_imu_quaternion();
    const auto gyro = titati_robot_->get_imu_angular_velocity();

    state->imu.quaternion[0] = quat_xyz_w[3];
    state->imu.quaternion[1] = quat_xyz_w[0];
    state->imu.quaternion[2] = quat_xyz_w[1];
    state->imu.quaternion[3] = quat_xyz_w[2];

    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = gyro[i];
    }

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        const int mapped = this->params.joint_mapping[i];
        if (mapped < static_cast<int>(positions.size()))
        {
            state->motor_state.q[i] = positions[mapped];
        }
        if (mapped < static_cast<int>(velocities.size()))
        {
            state->motor_state.dq[i] = velocities[mapped];
        }
        if (mapped < static_cast<int>(torques.size()))
        {
            state->motor_state.tau_est[i] = torques[mapped];
        }
    }
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    std::vector<double> q(motor_count_, 0.0);
    std::vector<double> dq(motor_count_, 0.0);
    std::vector<double> kp(motor_count_, 0.0);
    std::vector<double> kd(motor_count_, 0.0);
    std::vector<double> tau(motor_count_, 0.0);

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        const int mapped = this->params.joint_mapping[i];
        if (mapped >= static_cast<int>(motor_count_))
        {
            continue;
        }
        q[mapped] = command->motor_command.q[i];
        dq[mapped] = command->motor_command.dq[i];
        kp[mapped] = command->motor_command.kp[i];
        kd[mapped] = command->motor_command.kd[i];
        tau[mapped] = command->motor_command.tau[i];
    }

    titati_robot_->set_target_joint_mit(q, dq, kp, kd, tau);
}

void RL_Real::RobotControl()
{
    motiontime++;

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
        std::cout << std::endl
                  << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
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
    auto myDict = this->model.named_parameters();
    plt::clf();

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        plt::subplot(4, 4, i + 1);
        plt::ylim(-2.5, 2.5);
        plt::title("Joint " + std::to_string(i));
        plt::named_plot("target", this->plot_t, this->plot_target_joint_pos[i], "r-");
        plt::named_plot("real", this->plot_t, this->plot_real_joint_pos[i], "b-");
        plt::legend();
    }
    plt::pause(0.001);
#endif
}

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

int main(int argc, char **argv)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    ros::init(argc, argv, "rl_real_titati");
    RL_Real rl_real;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RL_Real>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
#else
    try
    {
        RL_Real rl_real;
        while (true)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    return 0;
#endif
}

