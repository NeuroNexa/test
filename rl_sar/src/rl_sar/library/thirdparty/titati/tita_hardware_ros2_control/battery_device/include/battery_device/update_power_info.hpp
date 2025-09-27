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

#ifndef BATTERY_DEVICE__UPDATE_POWER_INFO_HPP_
#define BATTERY_DEVICE__UPDATE_POWER_INFO_HPP_

#include <algorithm>
#include <atomic>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "battery_device/battery_info.hpp"
#include "battery_device/power_management_info.hpp"
#include "protocol/can_utils.hpp"
namespace tita
{
namespace battery_device
{
class UpdatePowerInfo
{
public:
  UpdatePowerInfo(
    std::shared_ptr<tita::battery_device::PowerManagementInfo> power_management_info_api_)
  {
    try {
      if (power_management_info_api_ == nullptr) {
        throw std::runtime_error("power_management_info_api_ is nullptr");
      }

      this->is_running.store(true);
      power_management_info_api = power_management_info_api_;
      get_power_domain_can_config();
      register_power_device_can_filter();
    } catch (const std::runtime_error & e) {
      std::cerr << "Caught an exception: " << e.what() << std::endl;
    }
  }
  ~UpdatePowerInfo() = default;

private:
#define MIN_TIME_OUT_US 1'000L      // 1ms
#define MAX_TIME_OUT_US 3'000'000L  // 3s
#define CAN_ID_AS_POWER_INFO (0x80U)
#define CAN_MASK_AS_POWER_INFO (0x7E0U)
  std::atomic<bool> is_running{false};
  std::string power_can_interface = "can0";
  std::string power_can_name = "power_device";
  bool power_can_extended_frame = false;
  bool power_can_rx_is_block = false;
  int64_t power_timeout_us = MAX_TIME_OUT_US;
  uint8_t power_can_id_offset = 0x00U;
  BasicCANConfig_T power_domain_info_can_config;

  std::shared_ptr<tita::battery_device::PowerManagementInfo> power_management_info_api = nullptr;

  tita::socket_can::can_fd_callback power_can_receive_callback =
    std::bind(&UpdatePowerInfo::get_power_can_data, this, std::placeholders::_1);

  std::shared_ptr<tita::socket_can::CanDev> power_can_receive_api =
    std::make_shared<tita::socket_can::CanDev>(
      power_can_interface, power_can_name, power_can_extended_frame, power_can_receive_callback,
      power_can_rx_is_block, power_timeout_us, power_can_id_offset);

  void register_power_device_can_filter();
  void get_power_can_data(std::shared_ptr<struct canfd_frame> recv_frame);
  void get_power_domain_can_config();
  void update_power_domain_info(std::shared_ptr<struct canfd_frame> recv_frame);
};
}  // namespace battery_device
}  // namespace tita

#endif  // BATTERY_DEVICE__UPDATE_POWER_INFO_HPP_
