/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_titati.hpp"

#include <cmath>
#include <vector>

RL_Real::RL_Real(const std::string &feedback_can_interface, const std::string &command_can_interface)
#if defined(USE_ROS2) && defined(USE_ROS)
    : rclcpp::Node("rl_real_node")
    , feedback_can_interface_(feedback_can_interface)
    , command_can_interface_(command_can_interface)
#else
    : feedback_can_interface_(feedback_can_interface)
    , command_can_interface_(command_can_interface)
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

    // read params from yaml
    this->ang_vel_type = "ang_vel_body";
    this->robot_name = "titati";
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

    // init robot hardware
    titati_robot_ = std::make_unique<tita_robot>(this->params.num_of_dofs, feedback_can_interface_, command_can_interface_);
    zero_torque_cmd_.assign(this->params.num_of_dofs, 0.0);
    if (titati_robot_)
    {
        motors_sdk_enabled_ = titati_robot_->set_motors_sdk(true);
        if (motors_sdk_enabled_)
        {
            std::cout << LOGGER::INFO << "Titati motors switched to SDK control via CAN interfaces ('"
                      << feedback_can_interface_ << "' feedback / '" << command_can_interface_ << "' command)." << std::endl;
        }
        else
        {
            std::cout << LOGGER::ERROR << "Failed to switch Titati motors to SDK control on CAN interface '"
                      << command_can_interface_ << "'." << std::endl;
            estop_engaged_ = true;
        }
    }
    joint_positions_.resize(this->params.num_of_dofs, 0.0);
    joint_velocities_.resize(this->params.num_of_dofs, 0.0);
    joint_torques_.resize(this->params.num_of_dofs, 0.0);

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
    if (titati_robot_)
    {
        ApplyZeroTorque();
        if (motors_sdk_enabled_.load())
        {
            titati_robot_->set_motors_sdk(false);
            motors_sdk_enabled_ = false;
        }
    }
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<double> *state)
{
    if (!state || !titati_robot_)
    {
        return;
    }

    joint_positions_ = titati_robot_->get_joint_q();
    joint_velocities_ = titati_robot_->get_joint_v();
    joint_torques_ = titati_robot_->get_joint_t();

    if (joint_positions_.size() < static_cast<size_t>(this->params.num_of_dofs))
    {
        return;
    }

    const size_t half_joints = joint_positions_.size() / 2;
    for (size_t id = 0; id < joint_positions_.size(); ++id)
    {
        if (id == 1 || id == 1 + half_joints)
        {
            if (joint_positions_[id] < -2.5)
            {
                joint_positions_[id] += 2 * M_PI;
            }
        }
    }

    auto quat = titati_robot_->get_imu_quaternion();
    auto accl = titati_robot_->get_imu_acceleration();
    auto gyro = titati_robot_->get_imu_angular_velocity();

    state->imu.quaternion[0] = quat[3]; // w
    state->imu.quaternion[1] = quat[0]; // x
    state->imu.quaternion[2] = quat[1]; // y
    state->imu.quaternion[3] = quat[2]; // z

    for (size_t i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = gyro[i];
        state->imu.accelerometer[i] = accl[i];
    }

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        int idx = this->params.joint_mapping[i];
        state->motor_state.q[i] = joint_positions_[idx];
        state->motor_state.dq[i] = joint_velocities_[idx];
        state->motor_state.tau_est[i] = joint_torques_[idx];
    }
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    if (!command || !titati_robot_ || estop_engaged_.load() || !motors_sdk_enabled_.load())
    {
        return;
    }
    std::scoped_lock<std::mutex> lock(command_mutex_);
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

    titati_robot_->set_target_joint_mit(q, dq, kp, kd, tau);
}

void RL_Real::ApplyZeroTorque()
{
    if (!titati_robot_)
    {
        return;
    }
    if (zero_torque_cmd_.size() != static_cast<size_t>(this->params.num_of_dofs))
    {
        zero_torque_cmd_.assign(this->params.num_of_dofs, 0.0);
    }
    std::scoped_lock<std::mutex> lock(command_mutex_);
    titati_robot_->set_target_joint_t(zero_torque_cmd_);
}

void RL_Real::ClearCommandQueues()
{
    torch::Tensor dump;
    while (output_dof_pos_queue.try_pop(dump))
    {
    }
    while (output_dof_vel_queue.try_pop(dump))
    {
    }
    while (output_dof_tau_queue.try_pop(dump))
    {
    }
}

void RL_Real::EngageEstop(const std::string &source)
{
    bool expected = false;
    if (!estop_engaged_.compare_exchange_strong(expected, true))
    {
        return;
    }

    std::cout << LOGGER::WARNING << "Soft e-stop engaged by " << source << "." << std::endl;
    ApplyZeroTorque();
    ClearCommandQueues();

    if (titati_robot_)
    {
        std::scoped_lock<std::mutex> lock(command_mutex_);
        if (motors_sdk_enabled_.load())
        {
            if (!titati_robot_->set_motors_sdk(false))
            {
                std::cout << LOGGER::WARNING << "Titati did not acknowledge SDK disable request during e-stop." << std::endl;
            }
            motors_sdk_enabled_ = false;
        }
    }
}

void RL_Real::ReleaseEstop(const std::string &source)
{
    bool expected = true;
    if (!estop_engaged_.compare_exchange_strong(expected, false))
    {
        return;
    }

    std::cout << LOGGER::INFO << "Soft e-stop cleared by " << source << "." << std::endl;

    if (!titati_robot_)
    {
        std::cout << LOGGER::ERROR << "Titati hardware interface is not initialised; unable to clear e-stop." << std::endl;
        estop_engaged_ = true;
        return;
    }

    bool sdk_enabled = motors_sdk_enabled_.load();
    if (!sdk_enabled)
    {
        std::scoped_lock<std::mutex> lock(command_mutex_);
        sdk_enabled = titati_robot_->set_motors_sdk(true);
        motors_sdk_enabled_ = sdk_enabled;
    }

    if (!sdk_enabled)
    {
        std::cout << LOGGER::ERROR << "Unable to re-enable Titati SDK control after e-stop; keeping motors disabled." << std::endl;
        ClearCommandQueues();
        estop_engaged_ = true;
        return;
    }

    auto current_q = titati_robot_->get_joint_q();
    auto current_dq = titati_robot_->get_joint_v();
    if (current_q.size() != static_cast<size_t>(this->params.num_of_dofs))
    {
        current_q = std::vector<double>(this->params.num_of_dofs, 0.0);
    }
    if (current_dq.size() != static_cast<size_t>(this->params.num_of_dofs))
    {
        current_dq = std::vector<double>(this->params.num_of_dofs, 0.0);
    }

    std::vector<double> kp(this->params.num_of_dofs, 0.0);
    std::vector<double> kd(this->params.num_of_dofs, 0.0);

    {
        std::scoped_lock<std::mutex> lock(command_mutex_);
        titati_robot_->set_target_joint_mit(current_q, current_dq, kp, kd, zero_torque_cmd_);
    }

    ClearCommandQueues();
}

void RL_Real::RobotControl()
{
    this->motiontime++;

    if (this->control.current_keyboard == Input::Keyboard::M)
    {
        EngageEstop("keyboard 'M'");
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Escape)
    {
        EngageEstop("keyboard 'ESC'");
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::K)
    {
        ReleaseEstop("keyboard 'K'");
        this->control.current_keyboard = this->control.last_keyboard;
    }

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

    if (estop_engaged_.load() || !motors_sdk_enabled_.load())
    {
        ApplyZeroTorque();
        return;
    }

    this->StateController(&this->robot_state, &this->robot_command);
    this->SetCommand(&this->robot_command);
}

void RL_Real::RunModel()
{
    if (estop_engaged_.load() || !motors_sdk_enabled_.load())
    {
        return;
    }

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
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

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

#ifdef PLOT
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
        int idx = this->params.joint_mapping[i];
        this->plot_real_joint_pos[i].push_back(joint_positions_[idx]);
        this->plot_target_joint_pos[i].push_back(this->output_dof_pos[0][i].item<double>());
        plt::subplot(this->params.num_of_dofs, 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    plt::pause(0.0001);
}
#endif

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
    std::string feedback_can = "can0";
    std::string command_can = "can0";
    bool show_help = false;
    std::vector<char *> passthrough_args;
    passthrough_args.push_back(argv[0]);

    for (int i = 1; i < argc; ++i)
    {
        std::string arg(argv[i]);
        if ((arg == "--can" || arg == "--interface") && (i + 1) < argc)
        {
            feedback_can = argv[++i];
            command_can = feedback_can;
            continue;
        }
        if (arg == "--feedback-can" && (i + 1) < argc)
        {
            feedback_can = argv[++i];
            continue;
        }
        if (arg == "--command-can" && (i + 1) < argc)
        {
            command_can = argv[++i];
            continue;
        }
        if (arg == "--help" || arg == "-h")
        {
            show_help = true;
            continue;
        }
        if (arg == "--can" || arg == "--interface" || arg == "--feedback-can" || arg == "--command-can")
        {
            std::cout << LOGGER::ERROR << "Missing value for option '" << arg << "'." << std::endl;
            return -1;
        }
        passthrough_args.push_back(argv[i]);
    }

    if (show_help)
    {
        std::cout << "Usage: " << argv[0] << " [--can CAN_IFACE] [--feedback-can CAN_IFACE] [--command-can CAN_IFACE]" << std::endl;
        std::cout << "       --can/-interface sets both the feedback and command interface (default: can0)" << std::endl;
        std::cout << "       --feedback-can sets the CAN FD interface for IMU/motor feedback" << std::endl;
        std::cout << "       --command-can sets the CAN FD interface for torque/mit commands" << std::endl;
        return 0;
    }

    int ros_argc = static_cast<int>(passthrough_args.size());
    char **ros_argv = passthrough_args.data();

#if defined(USE_ROS1) && defined(USE_ROS)
    signal(SIGINT, signalHandler);
    ros::init(ros_argc, ros_argv, "rl_sar");
    RL_Real rl_sar(feedback_can, command_can);
    ros::spin();
#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::init(ros_argc, ros_argv);
    rclcpp::spin(std::make_shared<RL_Real>(feedback_can, command_can));
    rclcpp::shutdown();
#elif defined(USE_CMAKE) || !defined(USE_ROS)
    RL_Real rl_sar(feedback_can, command_can);
    while (1) { sleep(10); }
#endif
    return 0;
}
