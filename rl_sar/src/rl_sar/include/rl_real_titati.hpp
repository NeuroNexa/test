#pragma once

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "fsm.hpp"

#include "titati_hardware.hpp"

#if defined(USE_ROS1) && defined(USE_ROS)
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#elif defined(USE_ROS2) && defined(USE_ROS)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#endif

#include <memory>
#include <vector>

class RL_Real
    : public RL
#if defined(USE_ROS2) && defined(USE_ROS)
    , public rclcpp::Node
#endif
{
public:
    RL_Real(const std::string& can_interface = "can0");
    ~RL_Real();

private:
    torch::Tensor Forward() override;
    void GetState(RobotState<double>* state) override;
    void SetCommand(const RobotCommand<double>* command) override;
    void RunModel();
    void RobotControl();

    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    void Plot();
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<double>> plot_real_joint_pos;
    std::vector<std::vector<double>> plot_target_joint_pos;

    std::unique_ptr<titati::hardware::TitatiHardware> hardware_;
    titati::hardware::CombinedState cached_state_;

    std::vector<double> command_q_;
    std::vector<double> command_dq_;
    std::vector<double> command_kp_;
    std::vector<double> command_kd_;
    std::vector<double> command_tau_;

#if defined(USE_ROS1) && defined(USE_ROS)
    geometry_msgs::Twist cmd_vel;
    ros::Subscriber cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg);
#elif defined(USE_ROS2) && defined(USE_ROS)
    geometry_msgs::msg::Twist cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
#endif
};

