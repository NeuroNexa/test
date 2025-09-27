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

#ifndef BATTERY_DEVICE__POWER_MANAGEMENT_HPP_
#define BATTERY_DEVICE__POWER_MANAGEMENT_HPP_

#include <algorithm>
#include <atomic>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "battery_device/power_management_info.hpp"
#include "titati/protocol/can_utils.hpp"

namespace tita
{
namespace battery_device
{
class PowerManagement
{
public:
  PowerManagement(
    std::shared_ptr<tita::battery_device::PowerManagementInfo> power_management_info_api_)
  {
    try {
      this->is_running.store(true);
      power_management_info_api = power_management_info_api_;
      register_power_management_can_data();
    } catch (const std::runtime_error & e) {
      std::cerr << "Caught an exception: " << e.what() << std::endl;
    }
  }
  ~PowerManagement() = default;

  void send_power_management_state_can_data(
    tita::battery_device::power_state_info_t power_state_info)
  {
    return send_power_management_state_can_data_impl(power_state_info);
  }
  void send_power_management_self_test_can_data(
    tita::battery_device::power_selftest_t power_selftest)
  {
    return send_power_management_self_test_can_data_impl(power_selftest);
  }
  void send_power_management_heart_beat_can_data(
    tita::battery_device::power_heart_beat_t power_heart_beat)
  {
    return send_power_management_heart_beat_can_data_impl(power_heart_beat);
  }

private:
#define MIN_TIME_OUT_US 1'000L      // 1ms
#define MAX_TIME_OUT_US 3'000'000L  // 3s
#define CAN_ID_AS_POWER_MANAGEMENT (0x80U)
#define CAN_MASK_AS_POWER_MANAGEMENT (0x7E0U)
  std::atomic<bool> is_running{false};
  std::string power_management_interface = "can0";
  std::string power_management_name = "power_management_device";
  bool power_management_can_extended_frame = false;
  bool power_management_can_rx_is_block = false;
  int64_t power_management_timeout_us = MAX_TIME_OUT_US;
  uint8_t power_management_can_id_offset = 0x00U;

  std::shared_ptr<tita::battery_device::PowerManagementInfo> power_management_info_api = nullptr;

  tita::socket_can::can_fd_callback power_management_can_receive_callback =
    std::bind(&PowerManagement::get_power_management_can_data, this, std::placeholders::_1);

  std::shared_ptr<tita::socket_can::CanDev> power_management_can_controller_api =
    std::make_shared<tita::socket_can::CanDev>(
      power_management_interface, power_management_name, power_management_can_extended_frame,
      power_management_can_receive_callback, power_management_can_rx_is_block,
      power_management_timeout_us, power_management_can_id_offset);

  void register_power_management_can_data();
  void get_power_management_can_data(std::shared_ptr<struct canfd_frame> recv_frame);
  // receive can data
  void update_power_management_state_can_data(std::shared_ptr<struct canfd_frame> recv_frame);
  void update_power_management_self_test_can_data(std::shared_ptr<struct canfd_frame> recv_frame);
  // send can data
  void send_power_management_state_can_data_impl(
    tita::battery_device::power_state_info_t power_state_info);
  void send_power_management_self_test_can_data_impl(
    tita::battery_device::power_selftest_t power_selftest);
  void send_power_management_heart_beat_can_data_impl(
    tita::battery_device::power_heart_beat_t power_heart_beat);
};
}  // namespace battery_device
}  // namespace tita

#endif  // BATTERY_DEVICE__POWER_MANAGEMENT_HPP_
