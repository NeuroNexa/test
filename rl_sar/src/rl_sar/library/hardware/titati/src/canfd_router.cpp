/* Copyright (c) 2023 Direct Drive Technology Co., Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "titati/canfd_router.hpp"

#include <chrono>
#include <cstring>
#include <sys/time.h>
#include <thread>

namespace titati
{
namespace
{
constexpr uint32_t kRouterCanId = 0x09FU;
constexpr uint32_t kRouterCanMask = 0x0FFU;
constexpr uint32_t kRpcCommandId = 0x170U;
constexpr uint16_t kSetReadyNext = 0x200U;
constexpr uint32_t kForceDirect = 0x03U;
constexpr uint32_t kResetReady = 0x00U;
}  // namespace

CanfdRouter::CanfdRouter()
{
  router_dev_ = std::make_shared<can_device::socket_can::CanDev>(
    router_interface_, router_name_, router_extended_frame_,
    std::bind(&CanfdRouter::HandleBoardFrame, this, std::placeholders::_1),
    router_rx_block_, router_timeout_us_, router_id_offset_);

  command_dev_ = std::make_shared<can_device::socket_can::CanDev>(
    command_interface_, command_name_, command_extended_frame_, command_fd_mode_,
    command_timeout_us_, command_id_offset_);

  RegisterFilter();
}

void CanfdRouter::RegisterFilter()
{
  struct can_filter router_filter;
  router_filter.can_id = kRouterCanId;
  router_filter.can_mask = kRouterCanMask;
  router_dev_->set_filter(&router_filter, sizeof(struct can_filter));
}

void CanfdRouter::RequestForceDirectMode()
{
  monitor_force_direct_.store(true);
  request_force_direct_.store(true);
}

void CanfdRouter::CancelForceDirectMode()
{
  monitor_force_direct_.store(false);
  request_force_direct_.store(false);
}

void CanfdRouter::HandleBoardFrame(std::shared_ptr<struct canfd_frame> recv_frame)
{
  if (recv_frame == nullptr || recv_frame->can_id != kRouterCanId)
  {
    return;
  }

  std::memcpy(&mode_, recv_frame->data + 4, sizeof(mode_));
  std::memcpy(&heart_cnt_, recv_frame->data + 8, sizeof(heart_cnt_));

  if (!monitor_force_direct_.load())
  {
    last_mode_ = mode_;
    return;
  }

  if (mode_ == kForceDirect)
  {
    request_force_direct_.store(false);
  }
  else if (last_mode_ == kForceDirect)
  {
    request_force_direct_.store(true);
  }

  if (!request_force_direct_.load())
  {
    last_mode_ = mode_;
    return;
  }

  if (mode_ == 1U || mode_ == 2U)
  {
    SendForceDirectCommand(kResetReady);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    SendForceDirectCommand(kForceDirect);
    request_force_direct_.store(false);
  }

  last_mode_ = mode_;
}

void CanfdRouter::SendForceDirectCommand(uint32_t value)
{
  if (!command_dev_)
  {
    return;
  }

  struct canfd_frame frame;
  frame.can_id = kRpcCommandId;
  frame.len = 10U;
  std::memset(frame.data, 0x00U, sizeof(frame.data));

  uint32_t timestamp = GetCurrentTime();
  uint16_t key = kSetReadyNext;

  std::memcpy(frame.data, &timestamp, sizeof(timestamp));
  std::memcpy(frame.data + 4, &key, sizeof(key));
  std::memcpy(frame.data + 6, &value, sizeof(value));

  command_dev_->send_can_message(frame);
}

uint32_t CanfdRouter::GetCurrentTime() const
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return static_cast<uint32_t>(tv.tv_sec * 1000000 + tv.tv_usec);
}

}  // namespace titati
