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

#include "battery_device/battery_device_node.hpp"

#include <functional>
#include <iostream>
namespace tita
{
namespace battery_device
{

BatteryDeviceNode::BatteryDeviceNode(const rclcpp::NodeOptions & options)
: Node("battery_device_node", options)
{
  namespace_ = this->get_namespace();
  if (!namespace_.empty() && namespace_[0] == '/') {
    namespace_ = namespace_.substr(1);
  }
  RCLCPP_INFO(this->get_logger(), "Battery device node is started");

  this->is_running.store(true);

  rclcpp::QoS battery_info_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  battery_info_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  battery_info_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  battery_info_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST).keep_last(10);

  // pub topic
  this->left_battery_state_topic_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
    battery_device::topics::kLeftBattery, battery_info_qos);

  this->right_battery_state_topic_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
    battery_device::topics::kRightBattery, battery_info_qos);

  this->left_battery_diagnostic_topic_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      battery_device::topics::kLeftBatteryDiag, battery_info_qos);

  this->right_battery_diagnostic_topic_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      battery_device::topics::kRightBatteryDiag, battery_info_qos);

  this->power_domain_diagnostic_topic_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      battery_device::topics::kPowerDomainDiag, battery_info_qos);

  // client service
  this->power_state_set_srv_ = this->create_service<tita_system_interfaces::srv::PowerStateSetSrv>(
    battery_device::topics::kPowerStateSetSrv, std::bind(
                                            &BatteryDeviceNode::power_state_set_callback, this,
                                            std::placeholders::_1, std::placeholders::_2));
  this->power_heart_beat_srv_ =
    this->create_service<tita_system_interfaces::srv::PowerHeartBeatSrv>(
      battery_device::topics::kPowerHeartBeatSrv, std::bind(
                                               &BatteryDeviceNode::power_heartbeat_callback, this,
                                               std::placeholders::_1, std::placeholders::_2));
  this->power_self_test_srv_ = this->create_service<tita_system_interfaces::srv::PowerSelfTestSrv>(
    battery_device::topics::kPowerSelfTestSrv, std::bind(
                                            &BatteryDeviceNode::power_self_test_callback, this,
                                            std::placeholders::_1, std::placeholders::_2));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&BatteryDeviceNode::timer_callback, this));  // 5hz

  faster_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(30),
    std::bind(&BatteryDeviceNode::faster_timer_callback, this));  // 30hz
}

void BatteryDeviceNode::power_state_set_callback(
  const std::shared_ptr<tita_system_interfaces::srv::PowerStateSetSrv::Request> req,
  std::shared_ptr<tita_system_interfaces::srv::PowerStateSetSrv::Response> res)
{
  (void)req;
  (void)res;

  if (req == nullptr) {
    res->success = false;
    return;
  }

  // make sure string to uint16
  auto string2uint16 = [](const std::string & str, uint16_t & val) {
    uint16_t origin_u16_val = val;
    std::stringstream ss;
    ss << str;
    ss >> val;
    if (ss.eof() && !ss.fail()) {
      return true;
    } else {
      val = origin_u16_val;
      return false;
    }
    return false;
  };

  // for test
  std::shared_ptr<tita::battery_device::power_state_info_t> power_state_info =
    std::make_shared<tita::battery_device::power_state_info_t>();

  if (req->power_state_set.status[0].values[0].key != std::string("power_5v")) {
    res->success = false;
    return;
  } else {
    if (!string2uint16(
          req->power_state_set.status[0].values[5].value,
          power_state_info->power_5v_operation_delay_ms)) {
      RCLCPP_INFO(this->get_logger(), "power_5v_operation_delay_ms string2uint16 failed");
      res->success = false;
      return;
    }
  }

  if (req->power_state_set.status[0].values[1].key != std::string("power_12v")) {
    res->success = false;
    return;
  } else {
    if (!string2uint16(
          req->power_state_set.status[0].values[6].value,
          power_state_info->power_12v_operation_delay_ms)) {
      RCLCPP_INFO(this->get_logger(), "power_12v_operation_delay_ms string2uint16 failed");
      res->success = false;
      return;
    }
  }

  if (req->power_state_set.status[0].values[2].key != std::string("power_24v")) {
    res->success = false;
    return;
  } else {
    if (!string2uint16(
          req->power_state_set.status[0].values[7].value,
          power_state_info->power_24v_operation_delay_ms)) {
      RCLCPP_INFO(this->get_logger(), "power_24v_operation_delay_ms string2uint16 failed");
      res->success = false;
      return;
    }
  }

  if (req->power_state_set.status[0].values[3].key != std::string("power_motor_48v")) {
    res->success = false;
    return;
  } else {
    if (!string2uint16(
          req->power_state_set.status[0].values[8].value,
          power_state_info->power_motor_48v_operation_delay_ms)) {
      RCLCPP_INFO(this->get_logger(), "power_motor_48v_operation_delay_ms string2uint16 failed");
      res->success = false;
      return;
    }
  }

  if (req->power_state_set.status[0].values[4].key != std::string("power_extern_48v")) {
    res->success = false;
    return;
  } else {
    if (!string2uint16(
          req->power_state_set.status[0].values[9].value,
          power_state_info->power_extern_48v_operation_delay_ms)) {
      RCLCPP_INFO(this->get_logger(), "power_extern_48v_operation_delay_ms string2uint16 failed");
      res->success = false;
      return;
    }
  }

  this->power_management_api->send_power_management_state_can_data(*power_state_info);

  res->success = true;

  return;
}

void BatteryDeviceNode::power_heartbeat_callback(
  const std::shared_ptr<tita_system_interfaces::srv::PowerHeartBeatSrv::Request> req,
  std::shared_ptr<tita_system_interfaces::srv::PowerHeartBeatSrv::Response> res)
{
  (void)req;
  (void)res;

  if (req == nullptr) {
    res->success = false;
    return;
  }

  // for test
  std::shared_ptr<tita::battery_device::power_heart_beat_t> power_heartbeat =
    std::make_shared<tita::battery_device::power_heart_beat_t>();

  power_heartbeat->cur_mode = 0x00U;
  if (!this->is_record_power_info) {
    power_heartbeat->get_right_battery_history_info = 0x00U;
    power_heartbeat->get_left_battery_history_info = 0x00U;
  } else {
    power_heartbeat->get_right_battery_history_info =
      this->battery_info_api->is_right_battery_online_detect.load();
    power_heartbeat->get_left_battery_history_info =
      this->battery_info_api->is_left_battery_online_detect.load();
  }

  this->power_management_api->send_power_management_heart_beat_can_data(*power_heartbeat);

  res->success = true;

  return;
}

void BatteryDeviceNode::power_self_test_callback(
  const std::shared_ptr<tita_system_interfaces::srv::PowerSelfTestSrv::Request> req,
  std::shared_ptr<tita_system_interfaces::srv::PowerSelfTestSrv::Response> res)
{
  (void)req;
  (void)res;

  if (req == nullptr) {
    res->success = false;
    return;
  }

  // for test
  std::shared_ptr<tita::battery_device::power_selftest_t> power_selftest =
    std::make_shared<tita::battery_device::power_selftest_t>();

  power_selftest->rst = 0x1FU;
  power_selftest->app_version = 0x2FU;
  power_selftest->build_timestamp = 0x3FU;

  this->power_management_api->send_power_management_self_test_can_data(*power_selftest);

  res->success = true;

  return;
}

bool BatteryDeviceNode::pub_left_battery_info()
{
  auto left_battery_info = sensor_msgs::msg::BatteryState();

  if (!this->battery_info_api->is_left_battery_online_detect.load()) {
    left_battery_info.header.stamp = rclcpp::Clock().now();
    left_battery_info.header.frame_id = namespace_ + std::string("/left_battery_info");
    left_battery_info.present = false;
    left_battery_info.location = std::string("offline");
    left_battery_state_topic_->publish(left_battery_info);
    return false;
  }

  left_battery_info.header.stamp = rclcpp::Clock().now();
  left_battery_info.header.frame_id = namespace_ + std::string("/left_battery_info");
  left_battery_info.present = true;
  left_battery_info.voltage =
    static_cast<float>(this->battery_info_api->get_left_battery_info_voltage());
  left_battery_info.temperature =
    static_cast<float>(this->battery_info_api->get_left_battery_info_temperatureMOS());
  // The current value is negative at discharge
  left_battery_info.current =
    (-1) * static_cast<float>(this->battery_info_api->get_left_battery_info_current_cadc());
  // ignore: charge | capacity | desigh_capacity
  left_battery_info.percentage =
    static_cast<float>(this->battery_info_api->get_left_battery_info_remain_soc());
  left_battery_info.power_supply_technology =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  left_battery_info.location = std::string("online");

  left_battery_info.cell_voltage = {
    static_cast<float>(this->battery_info_api->get_left_battery_info_vcell1()),
    static_cast<float>(this->battery_info_api->get_left_battery_info_vcell2()),
    static_cast<float>(this->battery_info_api->get_left_battery_info_vcell3()),
    static_cast<float>(this->battery_info_api->get_left_battery_info_vcell4()),
    static_cast<float>(this->battery_info_api->get_left_battery_info_vcell5()),
    static_cast<float>(this->battery_info_api->get_left_battery_info_vcell6()),
    static_cast<float>(this->battery_info_api->get_left_battery_info_vcell7()),
    static_cast<float>(this->battery_info_api->get_left_battery_info_vcell8()),
    static_cast<float>(this->battery_info_api->get_left_battery_info_vcell9()),
    static_cast<float>(this->battery_info_api->get_left_battery_info_vcell10()),
    static_cast<float>(this->battery_info_api->get_left_battery_info_vcell11()),
    static_cast<float>(this->battery_info_api->get_left_battery_info_vcell12()),
  };

  left_battery_state_topic_->publish(left_battery_info);

  return true;
}

bool BatteryDeviceNode::pub_right_battery_info()
{
  auto right_battery_info = sensor_msgs::msg::BatteryState();

  if (!this->battery_info_api->is_right_battery_online_detect.load()) {
    right_battery_info.header.stamp = rclcpp::Clock().now();
    right_battery_info.header.frame_id = namespace_ + std::string("/right_battery_info");
    right_battery_info.present = false;
    right_battery_info.location = std::string("offline");
    right_battery_state_topic_->publish(right_battery_info);
    return false;
  }

  right_battery_info.header.stamp = rclcpp::Clock().now();
  right_battery_info.header.frame_id = namespace_ + std::string("/right_battery_info");
  right_battery_info.present = true;
  right_battery_info.voltage =
    static_cast<float>(this->battery_info_api->get_right_battery_info_voltage());
  right_battery_info.temperature =
    static_cast<float>(this->battery_info_api->get_right_battery_info_temperatureMOS());
  // The current value is negative at discharge
  right_battery_info.current =
    (-1) * static_cast<float>(this->battery_info_api->get_right_battery_info_current_cadc());
  // ignore: charge | capacity | desigh_capacity
  right_battery_info.percentage =
    static_cast<float>(this->battery_info_api->get_right_battery_info_remain_soc());
  right_battery_info.power_supply_technology =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  right_battery_info.location = std::string("online");

  right_battery_info.cell_voltage = {
    static_cast<float>(this->battery_info_api->get_right_battery_info_vcell1()),
    static_cast<float>(this->battery_info_api->get_right_battery_info_vcell2()),
    static_cast<float>(this->battery_info_api->get_right_battery_info_vcell3()),
    static_cast<float>(this->battery_info_api->get_right_battery_info_vcell4()),
    static_cast<float>(this->battery_info_api->get_right_battery_info_vcell5()),
    static_cast<float>(this->battery_info_api->get_right_battery_info_vcell6()),
    static_cast<float>(this->battery_info_api->get_right_battery_info_vcell7()),
    static_cast<float>(this->battery_info_api->get_right_battery_info_vcell8()),
    static_cast<float>(this->battery_info_api->get_right_battery_info_vcell9()),
    static_cast<float>(this->battery_info_api->get_right_battery_info_vcell10()),
    static_cast<float>(this->battery_info_api->get_right_battery_info_vcell11()),
    static_cast<float>(this->battery_info_api->get_right_battery_info_vcell12()),
  };

  right_battery_state_topic_->publish(right_battery_info);

  return true;
}

#define BOOL2STRING(x) ((x) ? std::string("true") : std::string("false"))

bool BatteryDeviceNode::pub_left_battery_diagnostic_info()
{
  auto left_battery_diagnostic_info = diagnostic_msgs::msg::DiagnosticArray();
  battery_status_t temp_left_battery_status =
    this->battery_info_api->get_left_battery_info_battery_status();

  if (!this->battery_info_api->is_left_battery_online_detect.load()) {
    left_battery_diagnostic_info.status.resize(1);
    left_battery_diagnostic_info.status[0].values.resize(1);

    left_battery_diagnostic_info.header.stamp = rclcpp::Clock().now();
    left_battery_diagnostic_info.header.frame_id =
      namespace_ + std::string("/left_battery_diagnostic_info");

    left_battery_diagnostic_info.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    left_battery_diagnostic_info.status[0].name = std::string("Left Battery Diagnostic Info");
    left_battery_diagnostic_info.status[0].message = std::string("Offline");

    left_battery_diagnostic_topic_->publish(left_battery_diagnostic_info);
    return false;
  }

  left_battery_diagnostic_info.status.resize(1);
  left_battery_diagnostic_info.status[0].values.resize(11);

  left_battery_diagnostic_info.header.stamp = rclcpp::Clock().now();
  left_battery_diagnostic_info.header.frame_id =
    namespace_ + std::string("/left_battery_diagnostic_info");

  left_battery_diagnostic_info.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;

  left_battery_diagnostic_info.status[0].name = std::string("Left Battery Diagnostic Info");
  left_battery_diagnostic_info.status[0].message = std::string("Left Battery Online");

  left_battery_diagnostic_info.status[0].values[0].key =
    std::string("Left Battery Overvoltage Protection");
  left_battery_diagnostic_info.status[0].values[0].value =
    BOOL2STRING(temp_left_battery_status.high_voltage);

  left_battery_diagnostic_info.status[0].values[1].key =
    std::string("Left Battery Undervoltage Protection");
  left_battery_diagnostic_info.status[0].values[1].value =
    BOOL2STRING(temp_left_battery_status.low_voltage);

  left_battery_diagnostic_info.status[0].values[2].key =
    std::string("Left Battery Discharge Overcurrent 1 Protection");
  left_battery_diagnostic_info.status[0].values[2].value =
    BOOL2STRING(temp_left_battery_status.discharge_over_current1);

  left_battery_diagnostic_info.status[0].values[3].key =
    std::string("Left Battery Discharge Overcurrent 2 Protection");
  left_battery_diagnostic_info.status[0].values[3].value =
    BOOL2STRING(temp_left_battery_status.discharge_over_current2);

  left_battery_diagnostic_info.status[0].values[4].key =
    std::string("Left Battery Discharge Charge Overcurrent Protection");
  left_battery_diagnostic_info.status[0].values[4].value =
    BOOL2STRING(temp_left_battery_status.charge_over_current);

  left_battery_diagnostic_info.status[0].values[5].key =
    std::string("Left Battery Discharge Short Circuit Protection");
  left_battery_diagnostic_info.status[0].values[5].value =
    BOOL2STRING(temp_left_battery_status.short_circuit);

  left_battery_diagnostic_info.status[0].values[6].key =
    std::string("Left Battery Discharge Secondary Protection");
  left_battery_diagnostic_info.status[0].values[6].value =
    BOOL2STRING(temp_left_battery_status.abnormal_voltage);

  left_battery_diagnostic_info.status[0].values[7].key =
    std::string("Left Battery Discharge Charge Low Temperature Protection");
  left_battery_diagnostic_info.status[0].values[7].value =
    BOOL2STRING(temp_left_battery_status.charge_low_temperature);

  left_battery_diagnostic_info.status[0].values[8].key =
    std::string("Left Battery Discharge Charge High Temperature Protection");
  left_battery_diagnostic_info.status[0].values[8].value =
    BOOL2STRING(temp_left_battery_status.charge_high_temperature);

  left_battery_diagnostic_info.status[0].values[9].key =
    std::string("Left Battery Discharge Discharge Low Temperature Protection");
  left_battery_diagnostic_info.status[0].values[9].value =
    BOOL2STRING(temp_left_battery_status.discharge_low_temperature);

  left_battery_diagnostic_info.status[0].values[10].key =
    std::string("Left Battery Discharge Discharge High Temperature Protection");
  left_battery_diagnostic_info.status[0].values[10].value =
    BOOL2STRING(temp_left_battery_status.discharge_high_temperature);

  left_battery_diagnostic_topic_->publish(left_battery_diagnostic_info);

  return true;
}

bool BatteryDeviceNode::pub_right_battery_diagnostic_info()
{
  auto right_battery_diagnostic_info = diagnostic_msgs::msg::DiagnosticArray();
  battery_status_t temp_right_battery_status =
    this->battery_info_api->get_right_battery_info_battery_status();

  if (!this->battery_info_api->is_right_battery_online_detect.load()) {
    right_battery_diagnostic_info.status.resize(1);
    right_battery_diagnostic_info.status[0].values.resize(1);

    right_battery_diagnostic_info.header.stamp = rclcpp::Clock().now();
    right_battery_diagnostic_info.header.frame_id =
      namespace_ + std::string("/right_battery_diagnostic_info");

    right_battery_diagnostic_info.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    right_battery_diagnostic_info.status[0].name = std::string("Right Battery Diagnostic Info");
    right_battery_diagnostic_info.status[0].message = std::string("Offline");

    right_battery_diagnostic_topic_->publish(right_battery_diagnostic_info);
    return false;
  }

  right_battery_diagnostic_info.status.resize(1);
  right_battery_diagnostic_info.status[0].values.resize(11);

  right_battery_diagnostic_info.header.stamp = rclcpp::Clock().now();
  right_battery_diagnostic_info.header.frame_id =
    namespace_ + std::string("/right_battery_diagnostic_info");

  right_battery_diagnostic_info.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  right_battery_diagnostic_info.status[0].name = std::string("Right Battery Diagnostic Info");
  right_battery_diagnostic_info.status[0].message = std::string("Right Battery Online");

  right_battery_diagnostic_info.status[0].values[0].key =
    std::string("Right Battery Overvoltage Protection");
  right_battery_diagnostic_info.status[0].values[0].value =
    BOOL2STRING(temp_right_battery_status.high_voltage);

  right_battery_diagnostic_info.status[0].values[1].key =
    std::string("Right Battery Undervoltage Protection");
  right_battery_diagnostic_info.status[0].values[1].value =
    BOOL2STRING(temp_right_battery_status.low_voltage);

  right_battery_diagnostic_info.status[0].values[2].key =
    std::string("Right Battery Discharge Overcurrent 1 Protection");
  right_battery_diagnostic_info.status[0].values[2].value =
    BOOL2STRING(temp_right_battery_status.discharge_over_current1);

  right_battery_diagnostic_info.status[0].values[3].key =
    std::string("Right Battery Discharge Overcurrent 2 Protection");
  right_battery_diagnostic_info.status[0].values[3].value =
    BOOL2STRING(temp_right_battery_status.discharge_over_current2);

  right_battery_diagnostic_info.status[0].values[4].key =
    std::string("Right Battery Discharge Charge Overcurrent Protection");
  right_battery_diagnostic_info.status[0].values[4].value =
    BOOL2STRING(temp_right_battery_status.charge_over_current);

  right_battery_diagnostic_info.status[0].values[5].key =
    std::string("Right Battery Discharge Short Circuit Protection");
  right_battery_diagnostic_info.status[0].values[5].value =
    BOOL2STRING(temp_right_battery_status.short_circuit);

  right_battery_diagnostic_info.status[0].values[6].key =
    std::string("Right Battery Discharge Secondary Protection");
  right_battery_diagnostic_info.status[0].values[6].value =
    BOOL2STRING(temp_right_battery_status.abnormal_voltage);

  right_battery_diagnostic_info.status[0].values[7].key =
    std::string("Right Battery Discharge Charge Low Temperature Protection");
  right_battery_diagnostic_info.status[0].values[7].value =
    BOOL2STRING(temp_right_battery_status.charge_low_temperature);

  right_battery_diagnostic_info.status[0].values[8].key =
    std::string("Right Battery Discharge Charge High Temperature Protection");
  right_battery_diagnostic_info.status[0].values[8].value =
    BOOL2STRING(temp_right_battery_status.charge_high_temperature);

  right_battery_diagnostic_info.status[0].values[9].key =
    std::string("Right Battery Discharge Discharge Low Temperature Protection");
  right_battery_diagnostic_info.status[0].values[9].value =
    BOOL2STRING(temp_right_battery_status.discharge_low_temperature);

  right_battery_diagnostic_info.status[0].values[10].key =
    std::string("Right Battery Discharge Discharge High Temperature Protection");
  right_battery_diagnostic_info.status[0].values[10].value =
    BOOL2STRING(temp_right_battery_status.discharge_high_temperature);

  right_battery_diagnostic_topic_->publish(right_battery_diagnostic_info);

  return true;
}

bool BatteryDeviceNode::pub_power_domain_info()
{
// #define IBUS_MV2MA(x) (x)           // format (mV) to (mA)
// #define XT30_MV2MA(x) (x)           // format (mV) to (mA)
#define IBUS_MV2MA(x) ((2480.0 - (x)) / (20.0) * (1000))  // format (mV) to (mA)
#define XT30_MV2MA(x) (((x)-1560.0) / (0.044))            // format (mV) to (mA)
#define VBUS_MV(x) (x / (10.1 / (200.0 + 10.1)))          // vbus mv

  auto power_domain_info = diagnostic_msgs::msg::DiagnosticArray();

  power_domain_info.status.resize(1);
  power_domain_info.status[0].values.resize(8);

  power_domain_info.header.stamp = rclcpp::Clock().now();
  power_domain_info.header.frame_id = namespace_ + std::string("/power_domain_info");

  power_domain_info.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  power_domain_info.status[0].name = std::string("Power Domain Info");
  power_domain_info.status[0].message = std::string(
    "The power domain bus voltage and current value of the power domain and "
    "the user bus voltage and current value");

  power_domain_info.status[0].values[0].key =
    std::string("Power Domain Complete Bus Voltage Value(mV)");
  power_domain_info.status[0].values[0].value =
    std::to_string(VBUS_MV(this->power_management_info_api->get_power_domain_info_get_vbus_vol()));

  power_domain_info.status[0].values[1].key =
    std::string("Power Domain Complete Bus Current Value(mA)");
  power_domain_info.status[0].values[1].value = std::to_string(
    IBUS_MV2MA(this->power_management_info_api->get_power_domain_info_get_ibus_vol()));

  //! CH704: Current Sensor IC
  //! Lower voltage, Higher Ampere
  power_domain_info.status[0].values[2].key =
    std::string("Power Domain Complete Bus Current Max Value(mA)");
  power_domain_info.status[0].values[2].value = std::to_string(IBUS_MV2MA(
    this->power_management_info_api->get_power_domain_info_get_ibus_vol_min()));  // lower vol means
                                                                                  // higher amp

  //! CH704: Current Sensor IC
  //! Lower voltage, Higher Ampere
  power_domain_info.status[0].values[3].key =
    std::string("Power Domain Complete Bus Current Min Value(mA)");
  power_domain_info.status[0].values[3].value =
    std::to_string(IBUS_MV2MA(this->power_management_info_api
                                ->get_power_domain_info_get_ibus_vol_max()));  // higher vol means
                                                                               // lower amp

  power_domain_info.status[0].values[4].key =
    std::string("Power Domain User Bus Current Value(mV)");
  power_domain_info.status[0].values[4].value = std::to_string(24000);

  power_domain_info.status[0].values[5].key =
    std::string("Power Domain User Bus Current Value(mA)");
  power_domain_info.status[0].values[5].value = std::to_string(
    XT30_MV2MA(this->power_management_info_api->get_power_domain_info_get_xt30_vol()));

  power_domain_info.status[0].values[6].key =
    std::string("Power Domain User Bus Current Max Value(mA)");
  power_domain_info.status[0].values[6].value = std::to_string(
    XT30_MV2MA(this->power_management_info_api->get_power_domain_info_get_xt30_vol_max()));

  power_domain_info.status[0].values[7].key =
    std::string("Power Domain User Bus Current Min Value(mA)");
  power_domain_info.status[0].values[7].value = std::to_string(
    XT30_MV2MA(this->power_management_info_api->get_power_domain_info_get_xt30_vol_min()));

  power_domain_diagnostic_topic_->publish(power_domain_info);

  return true;
}

void BatteryDeviceNode::timer_callback()
{
  if (this->is_running.load() != true) {
    return;
  }

  pub_left_battery_info();
  pub_right_battery_info();
  pub_left_battery_diagnostic_info();
  pub_right_battery_diagnostic_info();
}

void BatteryDeviceNode::faster_timer_callback()
{
  if (this->is_running.load() != true) {
    return;
  }

  pub_power_domain_info();
}

}  // namespace battery_device
}  // namespace tita
