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

#include "titati_canfd_router/canfd_router_can_receive_api.hpp"

namespace can_device
{

void CanfdRouterCanReceiveApi::register_canfd_router_device_can_filter()
{

  struct can_filter canfd_router_device_rx_filter;
  canfd_router_device_rx_filter.can_id = CAN_ID_AS_CANFD_ROUTER_INFO;
  canfd_router_device_rx_filter.can_mask = CAN_MASK_AS_CANFD_ROUTER_INFO;
  this->canfd_router_can_receive_api->set_filter(&canfd_router_device_rx_filter, sizeof(struct can_filter));

  // struct canfd_frame frame;
  // frame.can_id = 0x170U;
  // frame.len = 10U;
  // memset(frame.data, 0x00U, sizeof(frame.data));

  // uint32_t timestamp = get_current_time();
  // uint16_t key = 0x200;
  // uint32_t value = 0x00U;

  // std::memcpy(frame.data, &timestamp, sizeof(timestamp));
  // std::memcpy(frame.data + 4, &key, sizeof(key));
  // std::memcpy(frame.data + 6, &value, sizeof(value));

  // set_forcedirect_can_send_api->send_can_message(frame);

  // timestamp = get_current_time();
  // key = 0x200;
  // value = 0x03U;

  // std::memcpy(frame.data, &timestamp, sizeof(timestamp));
  // std::memcpy(frame.data + 4, &key, sizeof(key));
  // std::memcpy(frame.data + 6, &value, sizeof(value));

  // set_forcedirect_can_send_api->send_can_message(frame);
}

void CanfdRouterCanReceiveApi::get_board_can_data(std::shared_ptr<struct canfd_frame> recv_frame)
{
    if (recv_frame->can_id == CAN_ID_CANFD_ROUTER) {
        std::memcpy(&mode_, recv_frame->data + 4, sizeof(mode_));
        std::memcpy(&heart_cnt_, recv_frame->data + 8, sizeof(heart_cnt_));
        std::cerr << "mode:" << mode_ << std::endl;
        if(init_flag_ && (mode_ == 2 || mode_ == 1)) {
          struct canfd_frame frame;
          frame.can_id = 0x170U;
          frame.len = 10U;
          memset(frame.data, 0x00U, sizeof(frame.data));

          uint32_t timestamp = get_current_time();
          uint16_t key = 0x200;
          uint32_t value = 0x00U;

          std::memcpy(frame.data, &timestamp, sizeof(timestamp));
          std::memcpy(frame.data + 4, &key, sizeof(key));
          std::memcpy(frame.data + 6, &value, sizeof(value));

          set_forcedirect_can_send_api->send_can_message(frame);

          std::this_thread::sleep_for(std::chrono::milliseconds(100));

          timestamp = get_current_time();
          key = 0x200;
          value = 0x03U;

          std::memcpy(frame.data, &timestamp, sizeof(timestamp));
          std::memcpy(frame.data + 4, &key, sizeof(key));
          std::memcpy(frame.data + 6, &value, sizeof(value));

          set_forcedirect_can_send_api->send_can_message(frame);
          init_flag_ = false;
      }
    }
}

}