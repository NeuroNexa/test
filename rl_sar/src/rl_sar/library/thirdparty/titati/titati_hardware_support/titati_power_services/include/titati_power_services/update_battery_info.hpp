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

#ifndef TITATI_POWER_SERVICES__UPDATE_BATTERY_INFO_HPP_
#define TITATI_POWER_SERVICES__UPDATE_BATTERY_INFO_HPP_

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

#include "titati_power_services/battery_info.hpp"
#include "protocol/can_utils.hpp"
namespace tita
{
namespace battery_device
{
class UpdateBatteryInfo
{
public:
  UpdateBatteryInfo(
    std::shared_ptr<tita::battery_device::BatteryInfo> battery_info_api_,
    std::shared_ptr<tita::battery_device::BatteryHistory> battery_history_info_api_)
  {
    try {
      if ((battery_info_api_ == nullptr) || (battery_history_info_api_ == nullptr)) {
        throw std::runtime_error("battery_info_api_ or battery_history_info_api_ is nullptr");
      }

      this->is_running.store(true);
      battery_info_api = battery_info_api_;
      battery_history_info_api = battery_history_info_api_;
      get_battery_can_config();
      register_battery_device_can_filter();
    } catch (const std::runtime_error & e) {
      std::cerr << "Caught an exception: " << e.what() << std::endl;
    }
  }
  ~UpdateBatteryInfo() = default;

private:
#define MIN_TIME_OUT_US 1'000L      // 1ms
#define MAX_TIME_OUT_US 3'000'000L  // 3s
#define CAN_ID_AS_BATTERY_INFO (0x80U)
#define CAN_MASK_AS_BATTERY_INFO (0x7E0U)
  std::atomic<bool> is_running{false};
  std::string battery_can_interface = "can0";
  std::string battery_can_name = "battery_device";
  bool battery_can_extended_frame = false;
  bool battery_can_rx_is_block = false;
  int64_t battery_timeout_us = MAX_TIME_OUT_US;
  uint8_t battery_can_id_offset = 0x00U;
  BasicCANConfig_T left_battery_can_config;
  BasicCANConfig_T right_battery_can_config;
  BasicCANConfig_T battery_history_can_config;

  std::shared_ptr<tita::battery_device::BatteryInfo> battery_info_api = nullptr;
  std::shared_ptr<tita::battery_device::BatteryHistory> battery_history_info_api = nullptr;

  tita::socket_can::can_fd_callback battery_can_receive_callback =
    std::bind(&UpdateBatteryInfo::get_battery_can_data, this, std::placeholders::_1);

  std::shared_ptr<tita::socket_can::CanDev> battery_can_receive_api =
    std::make_shared<tita::socket_can::CanDev>(
      battery_can_interface, battery_can_name, battery_can_extended_frame,
      battery_can_receive_callback, battery_can_rx_is_block, battery_timeout_us,
      battery_can_id_offset);

  void register_battery_device_can_filter();
  void get_battery_can_data(std::shared_ptr<struct canfd_frame> recv_frame);
  void get_battery_can_config();
  void update_right_battery_info(std::shared_ptr<struct canfd_frame> recv_frame);
  void update_left_battery_info(std::shared_ptr<struct canfd_frame> recv_frame);
  void update_battery_history_info(std::shared_ptr<struct canfd_frame> recv_frame);
};
}  // namespace battery_device
}  // namespace tita

#endif  // TITATI_POWER_SERVICES__UPDATE_BATTERY_INFO_HPP_
