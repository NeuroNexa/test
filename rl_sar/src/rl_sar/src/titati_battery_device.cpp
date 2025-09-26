#include <chrono>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tita_system_interfaces/srv/power_heart_beat_srv.hpp"
#include "tita_system_interfaces/srv/power_self_test_srv.hpp"
#include "tita_system_interfaces/srv/power_state_set_srv.hpp"
#include "titati/battery_device/power_management_sender.hpp"
#include "titati/battery_device/power_management_types.hpp"

namespace
{
using diagnostic_msgs::msg::KeyValue;

std::optional<std::string> find_value(const std::vector<KeyValue> & values, const std::string & key)
{
  for (const auto & kv : values) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  return std::nullopt;
}

bool parse_int8(const std::optional<std::string> & text, int8_t & out)
{
  if (!text) {
    return false;
  }
  try {
    const auto value = std::stoi(*text);
    if (value < -128 || value > 127) {
      return false;
    }
    out = static_cast<int8_t>(value);
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_uint16(const std::optional<std::string> & text, uint16_t & out)
{
  if (!text) {
    return false;
  }
  try {
    const auto value = std::stoul(*text);
    if (value > std::numeric_limits<uint16_t>::max()) {
      return false;
    }
    out = static_cast<uint16_t>(value);
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_uint8(const std::optional<std::string> & text, uint8_t & out)
{
  uint16_t temp = 0U;
  if (!parse_uint16(text, temp)) {
    return false;
  }
  if (temp > std::numeric_limits<uint8_t>::max()) {
    return false;
  }
  out = static_cast<uint8_t>(temp);
  return true;
}
}  // namespace

class TitatiBatteryDeviceNode : public rclcpp::Node
{
public:
  explicit TitatiBatteryDeviceNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("battery_device_node", options)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    power_state_service_ = this->create_service<tita_system_interfaces::srv::PowerStateSetSrv>(
      "system/power_supply/power_state_set_srv",
      std::bind(&TitatiBatteryDeviceNode::handle_power_state_set, this, _1, _2));

    power_heartbeat_service_ = this->create_service<tita_system_interfaces::srv::PowerHeartBeatSrv>(
      "system/power_supply/power_heart_beat_srv",
      std::bind(&TitatiBatteryDeviceNode::handle_power_heartbeat, this, _1, _2));

    power_self_test_service_ = this->create_service<tita_system_interfaces::srv::PowerSelfTestSrv>(
      "system/power_supply/power_self_test_srv",
      std::bind(&TitatiBatteryDeviceNode::handle_power_self_test, this, _1, _2));

    heartbeat_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&TitatiBatteryDeviceNode::send_periodic_heartbeat, this));

    RCLCPP_INFO(this->get_logger(), "Titati battery_device service bridge started");
  }

private:
  void handle_power_state_set(
    const tita_system_interfaces::srv::PowerStateSetSrv::Request::SharedPtr request,
    tita_system_interfaces::srv::PowerStateSetSrv::Response::SharedPtr response)
  {
    titati::battery::PowerStateCommand command{};

    if (request && !request->power_state_set.status.empty()) {
      const auto & values = request->power_state_set.status.front().values;

      if (!parse_int8(find_value(values, "power_5v"), command.power_5v)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Failed to parse power_5v from diagnostic request, defaulting to %d", command.power_5v);
      }
      if (!parse_int8(find_value(values, "power_12v"), command.power_12v)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Failed to parse power_12v from diagnostic request, defaulting to %d", command.power_12v);
      }
      if (!parse_int8(find_value(values, "power_24v"), command.power_24v)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Failed to parse power_24v from diagnostic request, defaulting to %d", command.power_24v);
      }
      if (!parse_int8(find_value(values, "power_motor_48v"), command.power_motor_48v)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Failed to parse power_motor_48v from diagnostic request, defaulting to %d",
          command.power_motor_48v);
      }
      if (!parse_int8(find_value(values, "power_extern_48v"), command.power_extern_48v)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Failed to parse power_extern_48v from diagnostic request, defaulting to %d",
          command.power_extern_48v);
      }

      if (!parse_uint16(find_value(values, "power_5v_operation_delay_ms"), command.power_5v_delay_ms)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Failed to parse power_5v_operation_delay_ms, defaulting to %u", command.power_5v_delay_ms);
      }
      if (!parse_uint16(find_value(values, "power_12v_operation_delay_ms"), command.power_12v_delay_ms)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Failed to parse power_12v_operation_delay_ms, defaulting to %u", command.power_12v_delay_ms);
      }
      if (!parse_uint16(find_value(values, "power_24v_operation_delay_ms"), command.power_24v_delay_ms)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Failed to parse power_24v_operation_delay_ms, defaulting to %u", command.power_24v_delay_ms);
      }
      if (!parse_uint16(
          find_value(values, "power_motor_48v_operation_delay_ms"), command.power_motor_48v_delay_ms)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Failed to parse power_motor_48v_operation_delay_ms, defaulting to %u",
          command.power_motor_48v_delay_ms);
      }
      if (!parse_uint16(
          find_value(values, "power_extern_48v_operation_delay_ms"), command.power_extern_48v_delay_ms)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Failed to parse power_extern_48v_operation_delay_ms, defaulting to %u",
          command.power_extern_48v_delay_ms);
      }
    }

    sender_.send_state(command);
    response->success = true;
  }

  void handle_power_heartbeat(
    const tita_system_interfaces::srv::PowerHeartBeatSrv::Request::SharedPtr request,
    tita_system_interfaces::srv::PowerHeartBeatSrv::Response::SharedPtr response)
  {
    titati::battery::PowerHeartbeat heartbeat{};
    heartbeat.cur_mode = 0x00U;

    if (request && !request->power_heart_beat.status.empty()) {
      const auto & values = request->power_heart_beat.status.front().values;
      (void)parse_uint8(find_value(values, "get_right_battery_history_info"), heartbeat.right_history);
      (void)parse_uint8(find_value(values, "get_left_battery_history_info"), heartbeat.left_history);
    }

    sender_.send_heartbeat(heartbeat);
    response->success = true;
  }

  void handle_power_self_test(
    const tita_system_interfaces::srv::PowerSelfTestSrv::Request::SharedPtr request,
    tita_system_interfaces::srv::PowerSelfTestSrv::Response::SharedPtr response)
  {
    (void)request;
    titati::battery::PowerSelfTestReport report{};
    report.rst = 0x1FULL;
    report.app_version = 0x2FU;
    report.build_timestamp = 0x3FU;

    sender_.send_self_test(report);
    response->success = true;
  }

  void send_periodic_heartbeat()
  {
    titati::battery::PowerHeartbeat heartbeat{};
    heartbeat.cur_mode = 0x00U;
    sender_.send_heartbeat(heartbeat);
  }

  titati::battery::PowerManagementSender sender_;

  rclcpp::Service<tita_system_interfaces::srv::PowerStateSetSrv>::SharedPtr power_state_service_;
  rclcpp::Service<tita_system_interfaces::srv::PowerHeartBeatSrv>::SharedPtr power_heartbeat_service_;
  rclcpp::Service<tita_system_interfaces::srv::PowerSelfTestSrv>::SharedPtr power_self_test_service_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TitatiBatteryDeviceNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
