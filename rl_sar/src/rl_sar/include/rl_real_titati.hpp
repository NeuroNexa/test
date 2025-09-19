/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_TITATI_HPP
#define RL_REAL_TITATI_HPP

//#define PLOT
//#define CSV_LOGGER
//#define USE_ROS

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "fsm.hpp"

#include "tita_robot/tita_robot.hpp"

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#if defined(USE_ROS1) && defined(USE_ROS)
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#elif defined(USE_ROS2) && defined(USE_ROS)
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

class RL_Real : public RL
#if defined(USE_ROS2) && defined(USE_ROS)
    , public rclcpp::Node
#endif
{
public:
    explicit RL_Real(const std::string &can_interface = "can0");
    ~RL_Real();

private:
    // rl functions
    torch::Tensor Forward() override;
    void GetState(RobotState<double> *state) override;
    void SetCommand(const RobotCommand<double> *command) override;
    void RunModel();
    void RobotControl();

    // initialization helpers
    void InitRobot();
    bool WaitForInitialState();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;

    // hardware interface
    std::unique_ptr<tita_robot> robot;
    std::string can_interface_;
    std::mutex state_mutex_;
    bool direct_mode_enabled_ = false;

    // cached hardware states
    std::vector<double> raw_joint_positions_;
    std::vector<double> raw_joint_velocities_;
    std::vector<double> raw_joint_torques_;
    std::array<double, 4> raw_quaternion_{};
    std::array<double, 3> raw_gyro_{};
    std::vector<double> command_position_buffer_;
    std::vector<double> command_velocity_buffer_;
    std::vector<double> command_kp_buffer_;
    std::vector<double> command_kd_buffer_;
    std::vector<double> command_tau_buffer_;
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
