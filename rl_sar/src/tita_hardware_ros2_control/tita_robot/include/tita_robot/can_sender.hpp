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

#ifndef MOTORS_CAN__MOTORS_CAN_SEND_API_HPP_
#define MOTORS_CAN__MOTORS_CAN_SEND_API_HPP_

#include <algorithm>
#include <atomic>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <cstdint>

#include "protocol/can_utils.hpp"

namespace can_device
{
namespace detail
{
inline std::string get_env_or_default(const char *name, const std::string &default_value)
{
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == 0) {
    return default_value;
  }
  return std::string(value);
}

inline uint8_t get_env_u8(const char *name, uint8_t default_value)
{
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == 0) {
    return default_value;
  }
  char *end = nullptr;
  long parsed = std::strtol(value, &end, 0);
  if (end == value || parsed < 0 || parsed > 255) {
    return default_value;
  }
  return static_cast<uint8_t>(parsed);
}
}


#define CAN_ID_SEND_MOTORS (0x120U)
#define CAN_ID_CHANNEL_INPUT (0x12DU)
#define CAN_ID_RPC_REQUEST (0x170U)

  enum ApiRpcKey
  {
    RPC_UNDEFINED = 0x000,
    GET_MODEL_INFO = 0x100,
    GET_SERIAL_NUMBER = 0x101,
    SET_READY_NEXT = 0x200,
    SET_BOARDCAST = 0x201,
    SET_INPUT_MODE = 0x221,
    SET_STAND_MODE = 0x222,
    SET_HEAD_MODE = 0x223,
    SET_JUMP = 0x231,
    SET_MOTOR_ZERO = 0x280,
  };
  struct ChannelInput
  {
    uint32_t timestamp;
    float forward;
    float yaw;
    float pitch;
    float roll;
    float height;
    float split;
    float tilt;
    float forward_accel;
    float yaw_accel;
  };
  struct RpcRequest
  {
    uint32_t timestamp;
    uint16_t key;
    uint32_t value;
  };
  struct motor_out
  {
    uint32_t timestamp;
    float position;
    float kp;
    float velocity;
    float kd;
    float torque;
  };

  class MotorsCanSendApi
  {
  public:
    MotorsCanSendApi(size_t size)
    {
      if (size % 8 == 0)
        leg_dof_ = 4;
      else if (size % 6 == 0)
        leg_dof_ = 3;
      leg_num_ = size / leg_dof_;
    }
    ~MotorsCanSendApi() = default;
    bool send_motors_can(std::vector<motor_out> motors);
    bool send_command_can_channel_input(ChannelInput data);
    bool send_command_can_rpc_request(RpcRequest data);

  private:
#define MIN_TIME_OUT_US 1'000L     // 1ms
#define MAX_TIME_OUT_US 3'000'000L // 3s
    std::string can_interface = detail::get_env_or_default("TITATI_CAN_TX_INTERFACE", detail::get_env_or_default("TITATI_CAN_INTERFACE", "can0"));
    std::string can_name = detail::get_env_or_default("TITATI_CAN_TX_NAME", "motors_can_send");
    int64_t timeout_us = MAX_TIME_OUT_US;
    uint8_t can_id_offset = detail::get_env_u8("TITATI_CAN_TX_ID_OFFSET", detail::get_env_u8("TITATI_CAN_ID_OFFSET", 0x00U));
    bool can_extended_frame = false;
    bool can_fd_mode = true;

    std::shared_ptr<can_device::socket_can::CanDev> can_send_api_ =
        std::make_shared<can_device::socket_can::CanDev>(
            can_interface, can_name, can_extended_frame, can_fd_mode, timeout_us, can_id_offset);

    inline uint32_t get_current_time()
    {
      struct timeval tv;
      gettimeofday(&tv, NULL);
      return std::move(tv.tv_sec * 1000000 + tv.tv_usec);
    }
    size_t leg_dof_{4}, leg_num_{2};
  };
} // namespace can_device

#endif // MOTORS_CAN__MOTORS_CAN_SEND_API_HPP_
