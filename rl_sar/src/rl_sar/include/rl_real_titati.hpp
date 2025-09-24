/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_TITATI_HPP
#define RL_REAL_TITATI_HPP

//#define CSV_LOGGER
//#define USE_ROS

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "fsm.hpp"

#include "tita_hardware/tita_robot.hpp"

#include <csignal>
#include <memory>
#include <vector>
#include <chrono>

#if defined(USE_ROS1) && defined(USE_ROS)
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#elif defined(USE_ROS2) && defined(USE_ROS)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#endif

class RL_Real : public RL
#if defined(USE_ROS2) && defined(USE_ROS)
    , public rclcpp::Node
#endif
{
public:
    RL_Real();
    ~RL_Real();

private:
    torch::Tensor Forward() override;
    void GetState(RobotState<double> *state) override;
    void SetCommand(const RobotCommand<double> *command) override;
    void RunModel();
    void RobotControl();
    bool EnsureMotorsSdkMode();

    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::unique_ptr<tita_robot> robot_;
    bool motors_sdk_enabled_{false};
    std::chrono::steady_clock::time_point last_sdk_retry_{};
    std::chrono::steady_clock::time_point last_sdk_warning_{};
    std::chrono::steady_clock::time_point last_command_debug_log_{};
    int motiontime{0};

    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_torques_;
    bool joint_mapping_valid_{false};
    bool state_size_warning_emitted_{false};
    bool state_value_warning_emitted_{false};
    std::size_t mit_send_failures_{0};
    std::size_t mit_send_success_{0};
    std::size_t sdk_retry_counter_{0};

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
