#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tita_robot/tita_robot.hpp"

namespace
{
constexpr double kDefaultPi = 3.14159265358979323846;
}  // namespace

class TitatiMotorTestNode : public rclcpp::Node
{
public:
  TitatiMotorTestNode()
  : Node("titati_motor_test")
  {
    motor_count_ = static_cast<std::size_t>(this->declare_parameter<int>("motor_count", 16));
    command_rate_hz_ = this->declare_parameter<double>("command_rate_hz", 200.0);
    settle_seconds_ = this->declare_parameter<double>("settle_seconds", 2.0);
    motion_seconds_ = this->declare_parameter<double>("motion_seconds", 3.0);
    motion_cycles_ = this->declare_parameter<double>("motion_cycles", 1.5);
    swing_amplitude_rad_ = this->declare_parameter<double>("test_amplitude_rad", 0.15);
    wheel_swing_amplitude_rad_ = this->declare_parameter<double>("wheel_test_amplitude_rad", 0.3);
    kp_default_ = this->declare_parameter<double>("kp_default", 20.0);
    kd_default_ = this->declare_parameter<double>("kd_default", 1.0);
    kp_wheel_ = this->declare_parameter<double>("wheel_kp", 5.0);
    kd_wheel_ = this->declare_parameter<double>("wheel_kd", 0.2);

    const std::vector<int64_t> wheel_indices_param =
      this->declare_parameter<std::vector<int64_t>>("wheel_joint_indices", {3, 7, 11, 15});
    wheel_indices_.reserve(wheel_indices_param.size());
    for (int64_t raw_index : wheel_indices_param) {
      if (raw_index >= 0) {
        wheel_indices_.push_back(static_cast<std::size_t>(raw_index));
      }
    }

    if (motor_count_ == 0U) {
      throw std::runtime_error("motor_count parameter must be greater than zero");
    }
    if (command_rate_hz_ <= 0.0) {
      throw std::runtime_error("command_rate_hz must be positive");
    }

    robot_ = std::make_unique<tita_robot>(motor_count_);

    kp_gains_.assign(motor_count_, kp_default_);
    kd_gains_.assign(motor_count_, kd_default_);
    for (const std::size_t idx : wheel_indices_) {
      if (idx < motor_count_) {
        kp_gains_[idx] = kp_wheel_;
        kd_gains_[idx] = kd_wheel_;
      }
    }
    zero_vector_.assign(motor_count_, 0.0);

    RCLCPP_INFO(
      this->get_logger(),
      "Configured Titati motor test for %zu actuators (command rate %.1f Hz, sweep %.2f rad)",
      motor_count_, command_rate_hz_, swing_amplitude_rad_);
  }

  void run()
  {
    bool sdk_mode_enabled = false;
    auto deactivate_sdk = [this, &sdk_mode_enabled]() {
      if (sdk_mode_enabled) {
        if (!robot_->set_motors_sdk(false)) {
          RCLCPP_WARN(this->get_logger(), "Failed to restore MCU locomotion mode");
        }
      }
    };

    RCLCPP_INFO(this->get_logger(), "Requesting direct motor SDK control from MCU...");
    if (!robot_->set_motors_sdk(true)) {
      RCLCPP_ERROR(this->get_logger(), "Unable to switch Titati MCU to SDK control mode");
      deactivate_sdk();
      return;
    }
    sdk_mode_enabled = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    neutral_positions_ = robot_->get_joint_q();
    if (neutral_positions_.size() != motor_count_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Expected %zu joint positions but received %zu. Check CAN router and cabling.",
        motor_count_, neutral_positions_.size());
      deactivate_sdk();
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Captured neutral joint positions. Holding pose for %.1f s before testing...",
      settle_seconds_);
    if (!hold_position(settle_seconds_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to maintain neutral pose before sweep");
      deactivate_sdk();
      return;
    }

    for (std::size_t motor_index = 0; motor_index < motor_count_ && rclcpp::ok(); ++motor_index) {
      if (!test_single_motor(motor_index)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Aborting sweep: unable to command Titati motor %zu", motor_index);
        deactivate_sdk();
        return;
      }
      if (!hold_position(settle_seconds_)) {
        RCLCPP_WARN(
          this->get_logger(),
          "Stopping sweep early because maintaining neutral pose failed after joint %zu",
          motor_index);
        deactivate_sdk();
        return;
      }
    }

    if (!hold_position(1.0)) {
      RCLCPP_WARN(this->get_logger(), "Final neutral hold failed");
    }
    deactivate_sdk();
    RCLCPP_INFO(this->get_logger(), "Motor excitation sequence finished");
  }

private:
  bool hold_position(double seconds)
  {
    const std::size_t iterations = static_cast<std::size_t>(seconds * command_rate_hz_);
    if (iterations == 0U) {
      return true;
    }

    for (std::size_t i = 0; i < iterations && rclcpp::ok(); ++i) {
      if (!command_positions(neutral_positions_)) {
        return false;
      }
      spin_wait();
    }
    return true;
  }

  bool test_single_motor(std::size_t index)
  {
    const bool is_wheel = is_wheel_joint(index);
    const double amplitude = is_wheel ? wheel_swing_amplitude_rad_ : swing_amplitude_rad_;
    const std::size_t total_steps = static_cast<std::size_t>(motion_seconds_ * command_rate_hz_);
    if (total_steps == 0U) {
      RCLCPP_WARN(
        this->get_logger(),
        "Motion duration is too short (%.3f s). Skipping joint %zu", motion_seconds_, index);
      return true;
    }

    auto latest_positions = robot_->get_joint_q();
    if (latest_positions.size() == motor_count_) {
      neutral_positions_ = latest_positions;
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Unable to refresh neutral pose before testing joint %zu (received %zu positions)",
        index, latest_positions.size());
    }

    std::vector<double> command = neutral_positions_;
    const auto status = robot_->get_joint_status();
    if (!status.empty() && index < status.size()) {
      RCLCPP_INFO(
        this->get_logger(),
        "Testing joint %zu (wheel: %s, status byte: %u) with amplitude %.3f rad",
        index, is_wheel ? "yes" : "no", status.at(index), amplitude);
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Testing joint %zu (wheel: %s) with amplitude %.3f rad",
        index, is_wheel ? "yes" : "no", amplitude);
    }

    for (std::size_t step = 0; step < total_steps && rclcpp::ok(); ++step) {
      const double phase = static_cast<double>(step) / static_cast<double>(total_steps);
      const double offset = amplitude * std::sin(phase * motion_cycles_ * 2.0 * kDefaultPi);
      command = neutral_positions_;
      if (index < command.size()) {
        command[index] = neutral_positions_[index] + offset;
      }

      if (!command_positions(command)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "MIT command failed while exciting joint %zu at step %zu", index, step);
        return false;
      }

      if (step % static_cast<std::size_t>(command_rate_hz_) == 0U) {
        const auto feedback = robot_->get_joint_q();
        if (index < feedback.size()) {
          RCLCPP_INFO(
            this->get_logger(),
            "  joint %zu target %.3f rad, measured %.3f rad",
            index, command[index], feedback[index]);
        }
      }
      spin_wait();
    }

    RCLCPP_INFO(this->get_logger(), "Completed sweep for joint %zu", index);
    return true;
  }

  bool command_positions(const std::vector<double> & positions)
  {
    if (positions.size() != motor_count_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Command vector size mismatch: expected %zu, received %zu",
        motor_count_, positions.size());
      return false;
    }
    return robot_->set_target_joint_mit(positions, zero_vector_, kp_gains_, kd_gains_, zero_vector_);
  }

  bool is_wheel_joint(std::size_t index) const
  {
    return std::find(wheel_indices_.begin(), wheel_indices_.end(), index) != wheel_indices_.end();
  }

  void spin_wait() const
  {
    const std::chrono::duration<double> sleep_time(1.0 / command_rate_hz_);
    std::this_thread::sleep_for(sleep_time);
  }

  std::size_t motor_count_;
  double command_rate_hz_;
  double settle_seconds_;
  double motion_seconds_;
  double motion_cycles_;
  double swing_amplitude_rad_;
  double wheel_swing_amplitude_rad_;
  double kp_default_;
  double kd_default_;
  double kp_wheel_;
  double kd_wheel_;

  std::vector<std::size_t> wheel_indices_;
  std::vector<double> kp_gains_;
  std::vector<double> kd_gains_;
  std::vector<double> zero_vector_;
  std::vector<double> neutral_positions_;
  std::unique_ptr<tita_robot> robot_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TitatiMotorTestNode>();
    node->run();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("titati_motor_test"), "Unhandled exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}

