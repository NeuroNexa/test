#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "tita_robot/tita_robot.hpp"

using namespace std::chrono_literals;

class MotorTestNode : public rclcpp::Node, public std::enable_shared_from_this<MotorTestNode>
{
public:
  MotorTestNode()
  : rclcpp::Node("titati_motor_test"),
    num_motors_(this->declare_parameter<int>("num_motors", 16)),
    joint_index_(this->declare_parameter<int>("joint_index", 0)),
    mode_(this->declare_parameter<std::string>("mode", "torque")),
    torque_command_(this->declare_parameter<double>("torque", 0.0)),
    kp_(this->declare_parameter<double>("kp", 40.0)),
    kd_(this->declare_parameter<double>("kd", 2.0)),
    position_command_(this->declare_parameter<double>("position", 0.0)),
    velocity_command_(this->declare_parameter<double>("velocity", 0.0)),
    duration_(this->declare_parameter<double>("duration", 5.0)),
    command_rate_hz_(this->declare_parameter<double>("command_rate", 500.0)),
    status_rate_hz_(this->declare_parameter<double>("status_rate", 10.0))
  {
    if (joint_index_ < 0 || joint_index_ >= num_motors_)
    {
      RCLCPP_FATAL(this->get_logger(), "joint_index %d is outside of [0, %d)", joint_index_, num_motors_);
      throw std::runtime_error("joint_index out of range");
    }

    robot_ = std::make_unique<tita_robot>(static_cast<size_t>(num_motors_));

    bool enabled = false;
    for (int attempt = 0; attempt < 5 && rclcpp::ok(); ++attempt)
    {
      enabled = robot_->set_motors_sdk(true);
      if (enabled)
      {
        break;
      }
      RCLCPP_WARN(this->get_logger(), "Failed to enable SDK control on attempt %d, retrying...", attempt + 1);
      std::this_thread::sleep_for(100ms);
    }

    if (!enabled)
    {
      RCLCPP_WARN(this->get_logger(), "Could not acquire FORCE_DIRECT mode. Ensure titati_canfd_router is running.");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "SDK control enabled. Starting motor test in %s mode.", mode_.c_str());
    }

    zero_vector_ = std::vector<double>(num_motors_, 0.0);
  }

  ~MotorTestNode() override
  {
    try
    {
      robot_->set_target_joint_t(zero_vector_);
      robot_->set_motors_sdk(false);
      RCLCPP_INFO(this->get_logger(), "SDK control disabled.");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception while shutting down motor test: %s", e.what());
    }
  }

  void run()
  {
    auto command_period = std::chrono::duration<double>(1.0 / std::max(1.0, command_rate_hz_));
    auto status_period = std::chrono::duration<double>(1.0 / std::max(1.0, status_rate_hz_));

    auto start_time = std::chrono::steady_clock::now();
    auto next_command = start_time;
    auto next_status = start_time;

    while (rclcpp::ok())
    {
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(now - start_time).count();
      if (duration_ > 0.0 && elapsed >= duration_)
      {
        break;
      }

      if (now >= next_command)
      {
        send_command();
        next_command += command_period;
      }

      if (now >= next_status)
      {
        print_status();
        next_status += status_period;
      }

      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(1ms);
    }

    robot_->set_target_joint_t(zero_vector_);
    RCLCPP_INFO(this->get_logger(), "Motor test finished. All torques set to zero.");
  }

private:
  void send_command()
  {
    if (mode_ == "torque")
    {
      auto torques = zero_vector_;
      torques[joint_index_] = torque_command_;
      robot_->set_target_joint_t(torques);
    }
    else if (mode_ == "mit")
    {
      auto q = zero_vector_;
      auto dq = zero_vector_;
      auto kp = zero_vector_;
      auto kd = zero_vector_;
      auto tau = zero_vector_;

      q[joint_index_] = position_command_;
      dq[joint_index_] = velocity_command_;
      kp[joint_index_] = kp_;
      kd[joint_index_] = kd_;
      tau[joint_index_] = torque_command_;

      robot_->set_target_joint_mit(q, dq, kp, kd, tau);
    }
    else
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "Unknown mode '%s', defaulting to torque hold", mode_.c_str());
      robot_->set_target_joint_t(zero_vector_);
    }
  }

  void print_status()
  {
    auto positions = robot_->get_joint_q();
    auto velocities = robot_->get_joint_v();
    auto torques = robot_->get_joint_t();

    if (joint_index_ < static_cast<int>(positions.size()))
    {
      RCLCPP_INFO(this->get_logger(),
                  "Joint %d -> pos: %.3f rad, vel: %.3f rad/s, torque: %.3f Nm",
                  joint_index_, positions[joint_index_], velocities[joint_index_], torques[joint_index_]);
    }
  }

  std::unique_ptr<tita_robot> robot_;
  const int num_motors_;
  const int joint_index_;
  const std::string mode_;
  const double torque_command_;
  const double kp_;
  const double kd_;
  const double position_command_;
  const double velocity_command_;
  const double duration_;
  const double command_rate_hz_;
  const double status_rate_hz_;
  std::vector<double> zero_vector_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorTestNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
