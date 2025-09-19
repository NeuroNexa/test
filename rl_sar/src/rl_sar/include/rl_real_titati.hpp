/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_TITATI_HPP
#define RL_REAL_TITATI_HPP

// #define PLOT
// #define CSV_LOGGER
// #define USE_ROS

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "fsm.hpp"

#include "tita_robot/tita_robot.hpp"
#include <atomic>
#include <csignal>
#include <mutex>
#include <string>
#include <vector>

#if defined(USE_ROS1) && defined(USE_ROS)
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#elif defined(USE_ROS2) && defined(USE_ROS)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#endif

#ifdef PLOT
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

class RL_Real : public RL
#if defined(USE_ROS2) && defined(USE_ROS)
    , public rclcpp::Node
#endif
{
public:
    RL_Real(const std::string &feedback_can_interface = "can0", const std::string &command_can_interface = "can0");
    ~RL_Real();

private:
    // rl functions
    torch::Tensor Forward() override;
    void GetState(RobotState<double> *state) override;
    void SetCommand(const RobotCommand<double> *command) override;
    void RunModel();
    void RobotControl();
    void EngageEstop(const std::string &source);
    void ReleaseEstop(const std::string &source);
    void ApplyZeroTorque();
    void ClearCommandQueues();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
#ifdef PLOT
    std::shared_ptr<LoopFunc> loop_plot;
#endif

#ifdef PLOT
    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<double>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();
#endif

    // titati interface
    std::unique_ptr<tita_robot> titati_robot_;
    std::string feedback_can_interface_;
    std::string command_can_interface_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_torques_;
    std::vector<double> zero_torque_cmd_;
    std::atomic<bool> estop_engaged_{false};
    std::atomic<bool> motors_sdk_enabled_{false};
    std::mutex command_mutex_;

    // others
    int motiontime = 0;

#if defined(USE_ROS1) && defined(USE_ROS)
    geometry_msgs::Twist cmd_vel;
    ros::Subscriber cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
#elif defined(USE_ROS2) && defined(USE_ROS)
    geometry_msgs::msg::Twist cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
#endif
};

#endif // RL_REAL_TITATI_HPP
