#include "rl_real_titati.hpp"

#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <csignal>
#include <cstdlib>
#include <string>
#include <unistd.h>

#ifdef PLOT
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

RL_Real::RL_Real(const std::string& can_interface)
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

    hardware_ = std::make_unique<titati::hardware::TitatiHardware>(can_interface, this->params.num_of_dofs);
    if (!hardware_->Initialize())
    {
        throw std::runtime_error("Failed to initialise Titati CAN interface");
    }
    if (!hardware_->SetDirectControlMode(true))
    {
        std::cerr << LOGGER::WARNING << "Unable to switch Titati controller to direct mode" << std::endl;
    }

    this->InitOutputs();
    this->InitControl();

    command_q_.assign(this->params.num_of_dofs, 0.0);
    command_dq_.assign(this->params.num_of_dofs, 0.0);
    command_kp_.assign(this->params.num_of_dofs, 0.0);
    command_kd_.assign(this->params.num_of_dofs, 0.0);
    command_tau_.assign(this->params.num_of_dofs, 0.0);

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
    for (auto &vec : this->plot_real_joint_pos)
    {
        vec = std::vector<double>(this->plot_size, 0.0);
    }
    for (auto &vec : this->plot_target_joint_pos)
    {
        vec = std::vector<double>(this->plot_size, 0.0);
    }
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
    if (hardware_)
    {
        hardware_->Shutdown();
    }
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<double>* state)
{
    cached_state_ = hardware_->GetLatestState();

    state->imu.quaternion[0] = cached_state_.imu.quaternion[0];
    state->imu.quaternion[1] = cached_state_.imu.quaternion[1];
    state->imu.quaternion[2] = cached_state_.imu.quaternion[2];
    state->imu.quaternion[3] = cached_state_.imu.quaternion[3];

    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = cached_state_.imu.gyroscope[i];
        state->imu.accelerometer[i] = cached_state_.imu.acceleration[i];
    }

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        const std::size_t hardware_index = this->params.joint_mapping[i];
        if (hardware_index >= cached_state_.motors.size())
        {
            continue;
        }
        state->motor_state.q[i] = cached_state_.motors[hardware_index].position;
        state->motor_state.dq[i] = cached_state_.motors[hardware_index].velocity;
        state->motor_state.tau_est[i] = cached_state_.motors[hardware_index].torque;
    }
}

void RL_Real::SetCommand(const RobotCommand<double>* command)
{
    std::fill(command_q_.begin(), command_q_.end(), 0.0);
    std::fill(command_dq_.begin(), command_dq_.end(), 0.0);
    std::fill(command_kp_.begin(), command_kp_.end(), 0.0);
    std::fill(command_kd_.begin(), command_kd_.end(), 0.0);
    std::fill(command_tau_.begin(), command_tau_.end(), 0.0);

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        const std::size_t hardware_index = this->params.joint_mapping[i];
        if (hardware_index >= command_q_.size())
        {
            continue;
        }
        command_q_[hardware_index] = command->motor_command.q[i];
        command_dq_[hardware_index] = command->motor_command.dq[i];
        command_kp_[hardware_index] = command->motor_command.kp[i];
        command_kd_[hardware_index] = command->motor_command.kd[i];
        command_tau_[hardware_index] = command->motor_command.tau[i];
    }

    if (!hardware_->SendMitCommand(command_q_, command_dq_, command_kp_, command_kd_, command_tau_))
    {
        std::cerr << LOGGER::WARNING << "Failed to send Titati motor command" << std::endl;
    }
}

void RL_Real::RobotControl()
{
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
        std::cout << std::endl
                  << LOGGER::INFO << "Navigation mode: "
                  << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
        this->control.current_keyboard = this->control.last_keyboard;
    }

#if defined(USE_ROS) && (defined(USE_ROS1) || defined(USE_ROS2))
    if (this->control.navigation_mode)
    {
        this->control.x = this->cmd_vel.linear.x;
        this->control.y = this->cmd_vel.linear.y;
        this->control.yaw = this->cmd_vel.angular.z;
    }
#endif

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
#if defined(USE_ROS) && (defined(USE_ROS1) || defined(USE_ROS2))
        if (this->control.navigation_mode)
        {
            this->obs.commands = torch::tensor({{this->cmd_vel.linear.x, this->cmd_vel.linear.y, this->cmd_vel.angular.z}});
        }
        else
#endif
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

#ifdef PLOT
void RL_Real::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->episode_length_buf);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->robot_state.motor_state.q[i]);
        this->plot_target_joint_pos[i].push_back(this->output_dof_pos[0][i].item<double>());
        plt::subplot(this->params.num_of_dofs, 1, i + 1);
        plt::named_plot("real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    plt::pause(0.0001);
}
#else
void RL_Real::Plot() {}
#endif

#if defined(USE_ROS1) && defined(USE_ROS)
void RL_Real::CmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    this->cmd_vel = *msg;
}
#elif defined(USE_ROS2) && defined(USE_ROS)
void RL_Real::CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
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
    std::string can_interface = "can0";
    if (const char *env = std::getenv("CAN_INTERFACE"))
    {
        can_interface = env;
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
    while (1) { sleep(10); }
#endif
    return 0;
}

