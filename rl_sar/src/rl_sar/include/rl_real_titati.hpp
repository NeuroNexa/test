/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_TITATI_HPP
#define RL_REAL_TITATI_HPP

// #define CSV_LOGGER

#include <atomic>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
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
    enum class LegGroup
    {
        FrontRight,
        FrontLeft,
        RearRight,
        RearLeft,
        Unknown
    };

    enum class JointKind
    {
        Hip,
        Thigh,
        Calf,
        Wheel,
        Unknown
    };

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

    static bool IsRearLeg(LegGroup leg);
    static LegGroup ParseLegGroup(const std::string& joint_name);
    static JointKind ParseJointKind(const std::string& joint_name);
    double PolicyToLegdataPosition(int joint_index, double policy_pos) const;
    double PolicyToLegdataVelocity(int joint_index, double policy_vel) const;
    double PolicyToHardwareTorque(int joint_index, double policy_tau) const;
    double HardwareToPolicyTorque(int joint_index, double hardware_tau) const;

    std::vector<LegGroup> joint_leg_group_;
    std::vector<JointKind> joint_kind_;
    std::vector<double> legdata_joint_pos_;
    std::vector<double> legdata_joint_vel_;
    std::unordered_map<int, std::size_t> wheel_index_to_slot_;
    std::vector<int> wheel_round_count_;
    std::vector<double> previous_wheel_angle_;
    std::vector<double> initial_wheel_abs_;

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