/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_TITATI_HPP
#define RL_REAL_TITATI_HPP

// #define CSV_LOGGER

#include <atomic>
#include <memory>
#include <vector>

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "fsm.hpp"

#include "titati_can_driver/tita_robot.hpp"

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
    // rl functions
    torch::Tensor Forward() override;
    void GetState(RobotState<double>* state) override;
    void SetCommand(const RobotCommand<double>* command) override;
    void RunModel();
    void RobotControl();

    // hardware helpers
    bool AcquireSDKControl();
    void ReleaseSDKControl();
    bool EnsureSDKControl();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard_;
    std::shared_ptr<LoopFunc> loop_control_;
    std::shared_ptr<LoopFunc> loop_rl_;

    // robot interface
    std::unique_ptr<tita_robot> robot_;
    std::vector<double> command_tau_;
    std::atomic<bool> sdk_enabled_{false};

#if defined(USE_ROS1) && defined(USE_ROS)
    geometry_msgs::Twist cmd_vel_{};
    ros::Subscriber cmd_vel_subscriber_;
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg);
#elif defined(USE_ROS2) && defined(USE_ROS)
    geometry_msgs::msg::Twist cmd_vel_{};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
#endif
};

#endif // RL_REAL_TITATI_HPP