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

#ifndef CANFD_ROUTER_CAN_API_HPP_
#define CANFD_ROUTER_CAN_API_HPP_

#include <algorithm>
#include <atomic>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "protocol/can_utils.hpp"

namespace can_device
{
#define CAN_ID_CANFD_ROUTER (0x09FU)

class CanfdRouterCanReceiveApi
{
public:
  CanfdRouterCanReceiveApi()
  {
    register_canfd_router_device_can_filter();
  }
  ~CanfdRouterCanReceiveApi() {}

  void set_forcedirect_mode(bool init_flag) {init_flag_ = init_flag;}

private:
#define MIN_TIME_OUT_US 1'000L      // 1ms
#define MAX_TIME_OUT_US 3'000'000L  // 3s
#define CAN_ID_AS_CANFD_ROUTER_INFO (0x09FU)
#define CAN_MASK_AS_CANFD_ROUTER_INFO (0x0FFU)
  std::string canfd_router_can_interface = "can0";
  std::string canfd_router_can_name = "canfd_can";
  bool canfd_router_can_extended_frame = false;
  bool canfd_router_can_rx_is_block = false;
  int64_t canfd_router_timeout_us = MAX_TIME_OUT_US;
  uint8_t canfd_router_can_id_offset = 0x00U;

  void register_canfd_router_device_can_filter();
  tita::socket_can::can_fd_callback canfd_router_can_receive_callback =
    std::bind(&CanfdRouterCanReceiveApi::get_board_can_data, this, std::placeholders::_1);

  std::shared_ptr<tita::socket_can::CanDev> canfd_router_can_receive_api =
    std::make_shared<tita::socket_can::CanDev>(
      canfd_router_can_interface, canfd_router_can_name, canfd_router_can_extended_frame, canfd_router_can_receive_callback,
      canfd_router_can_rx_is_block, canfd_router_timeout_us, canfd_router_can_id_offset);

  void get_board_can_data(std::shared_ptr<struct canfd_frame> recv_frame);

  uint32_t timestamp_;
  uint32_t mode_;
  uint32_t heart_cnt_;

  std::string set_forcedirect_can_interface = "can0";
  std::string set_forcedirect_can_name = "set_forcedirect_can";
  int64_t set_forcedirect_timeout_us = MAX_TIME_OUT_US;
  uint8_t set_forcedirect_can_id_offset = 0x00U;
  bool set_forcedirect_can_extended_frame = false;
  bool set_forcedirect_can_fd_mode = true;

  std::shared_ptr<tita::socket_can::CanDev> set_forcedirect_can_send_api =
    std::make_shared<tita::socket_can::CanDev>(
      set_forcedirect_can_interface, set_forcedirect_can_name, set_forcedirect_can_extended_frame, set_forcedirect_can_fd_mode,
      set_forcedirect_timeout_us, set_forcedirect_can_id_offset);
  inline uint32_t get_current_time()
  {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return std::move(tv.tv_sec * 1000000 + tv.tv_usec);
  }

  bool init_flag_ = false;
 
};

}

#endif  // CANFD_ROUTER_CAN_API_HPP_