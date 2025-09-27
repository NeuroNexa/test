// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TITATI_POWER_SERVICES__TITATI_POWER_SERVICES_NODE_HPP_
#define TITATI_POWER_SERVICES__TITATI_POWER_SERVICES_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "titati_power_services/battery.hpp"
#include "titati_power_services/battery_device_node.hpp"
#include "titati_power_services/battery_info.hpp"
#include "titati_power_services/power_management.hpp"
#include "titati_power_services/power_management_info.hpp"
#include "titati_power_services/update_battery_info.hpp"
#include "titati_power_services/update_power_info.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "titati_system_interfaces/srv/power_heart_beat_srv.hpp"
#include "titati_system_interfaces/srv/power_self_test_srv.hpp"
#include "titati_system_interfaces/srv/power_state_set_srv.hpp"
#include "titati_topics/topic_names.hpp"
namespace tita
{
namespace battery_device
{

class BatteryDeviceNode : public rclcpp::Node
{
public:
  explicit BatteryDeviceNode(const rclcpp::NodeOptions & option);
  ~BatteryDeviceNode() = default;

private:
  std::atomic<bool> is_running{false};
  std::string namespace_ = "";
  bool is_record_power_info = false;

  std::shared_ptr<tita::battery_device::PowerManagementInfo> power_management_info_api =
    std::make_shared<tita::battery_device::PowerManagementInfo>();

  std::shared_ptr<tita::battery_device::PowerManagement> power_management_api =
    std::make_shared<tita::battery_device::PowerManagement>(power_management_info_api);

  std::shared_ptr<tita::battery_device::BatteryInfo> battery_info_api =
    std::make_shared<tita::battery_device::BatteryInfo>();

  std::shared_ptr<tita::battery_device::BatteryHistory> battery_history_info_api =
    std::make_shared<tita::battery_device::BatteryHistory>();

  std::shared_ptr<tita::battery_device::UpdateBatteryInfo> update_battery_info_api =
    std::make_shared<tita::battery_device::UpdateBatteryInfo>(
      battery_info_api, battery_history_info_api);

  std::shared_ptr<tita::battery_device::UpdatePowerInfo> update_power_info_api =
    std::make_shared<tita::battery_device::UpdatePowerInfo>(power_management_info_api);

  std::shared_ptr<tita::battery_device::BatteryRecord> battery_record_api =
    std::make_shared<tita::battery_device::BatteryRecord>(
      battery_history_info_api, is_record_power_info, 0);

  // publisher
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr left_battery_state_topic_ = nullptr;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    left_battery_diagnostic_topic_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr right_battery_state_topic_ = nullptr;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    right_battery_diagnostic_topic_ = nullptr;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    power_domain_diagnostic_topic_ = nullptr;

  // service for power management system
  rclcpp::Service<titati_system_interfaces::srv::PowerStateSetSrv>::SharedPtr power_state_set_srv_ =
    nullptr;
  rclcpp::Service<titati_system_interfaces::srv::PowerHeartBeatSrv>::SharedPtr power_heart_beat_srv_ =
    nullptr;
  rclcpp::Service<titati_system_interfaces::srv::PowerSelfTestSrv>::SharedPtr power_self_test_srv_ =
    nullptr;

  rclcpp::TimerBase::SharedPtr timer_ = nullptr;
  rclcpp::TimerBase::SharedPtr faster_timer_ = nullptr;

  void power_state_set_callback(
    const std::shared_ptr<titati_system_interfaces::srv::PowerStateSetSrv::Request> req,
    std::shared_ptr<titati_system_interfaces::srv::PowerStateSetSrv::Response> res);

  void power_self_test_callback(
    const std::shared_ptr<titati_system_interfaces::srv::PowerSelfTestSrv::Request> req,
    std::shared_ptr<titati_system_interfaces::srv::PowerSelfTestSrv::Response> res);

  void power_heartbeat_callback(
    const std::shared_ptr<titati_system_interfaces::srv::PowerHeartBeatSrv::Request> req,
    std::shared_ptr<titati_system_interfaces::srv::PowerHeartBeatSrv::Response> res);

  bool pub_left_battery_info();
  bool pub_right_battery_info();
  bool pub_left_battery_diagnostic_info();
  bool pub_right_battery_diagnostic_info();
  bool pub_power_domain_info();
  void timer_callback();
  void faster_timer_callback();
};
}  // namespace battery_device
}  // namespace tita

#endif  // TITATI_POWER_SERVICES__TITATI_POWER_SERVICES_NODE_HPP_
