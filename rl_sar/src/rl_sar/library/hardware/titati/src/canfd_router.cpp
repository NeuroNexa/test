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
#include <mutex>
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
constexpr auto kHeartbeatTimeout = std::chrono::milliseconds(500);
constexpr auto kMonitorInterval = std::chrono::milliseconds(100);
constexpr auto kMinCommandInterval = std::chrono::milliseconds(200);
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
  StartMonitorThread();
}

CanfdRouter::~CanfdRouter()
{
  StopMonitorThread();
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
  direct_control_requested_.store(true);
  request_force_direct_.store(true);
}

void CanfdRouter::CancelForceDirectMode()
{
  direct_control_requested_.store(false);
  direct_mode_active_.store(false);
  request_force_direct_.store(false);
  last_heart_cnt_.store(0U);
  last_heartbeat_time_us_.store(0);
  last_command_time_us_.store(0);
}

void CanfdRouter::HandleBoardFrame(std::shared_ptr<struct canfd_frame> recv_frame)
{
  if (recv_frame == nullptr || recv_frame->can_id != kRouterCanId)
  {
    return;
  }

  std::memcpy(&mode_, recv_frame->data + 4, sizeof(mode_));
  std::memcpy(&heart_cnt_, recv_frame->data + 8, sizeof(heart_cnt_));
  last_heart_cnt_.store(heart_cnt_);
  const auto now = std::chrono::steady_clock::now();
  const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
  last_heartbeat_time_us_.store(now_us);

  if (!request_force_direct_.load())
  {
    return;
  }

  if (mode_ == 1U || mode_ == 2U)
  {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      SendHandshakeSequence();
    }
    request_force_direct_.store(false);
    direct_mode_active_.store(true);
  }
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

void CanfdRouter::SendHandshakeSequence()
{
  const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto last_sent_us = last_command_time_us_.load();
  const auto min_interval_us = std::chrono::duration_cast<std::chrono::microseconds>(kMinCommandInterval).count();

  if (last_sent_us != 0 && (now_us - last_sent_us) < min_interval_us)
  {
    return;
  }

  last_command_time_us_.store(now_us);
  SendForceDirectCommand(kResetReady);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  SendForceDirectCommand(kForceDirect);
}

void CanfdRouter::StartMonitorThread()
{
  running_.store(true);
  monitor_thread_ = std::thread(&CanfdRouter::MonitorLoop, this);
}

void CanfdRouter::StopMonitorThread()
{
  running_.store(false);
  if (monitor_thread_.joinable())
  {
    monitor_thread_.join();
  }
}

void CanfdRouter::MonitorLoop()
{
  uint32_t previous_heart_cnt = 0U;
  std::size_t stale_cycles = 0U;

  while (running_.load())
  {
    std::this_thread::sleep_for(kMonitorInterval);

    if (!direct_control_requested_.load())
    {
      direct_mode_active_.store(false);
      stale_cycles = 0U;
      previous_heart_cnt = last_heart_cnt_.load();
      continue;
    }

    const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    const auto last_us = last_heartbeat_time_us_.load();
    const auto elapsed = now_us - last_us;

    uint32_t current_heart_cnt = last_heart_cnt_.load();
    if (current_heart_cnt == previous_heart_cnt)
    {
      ++stale_cycles;
    }
    else
    {
      stale_cycles = 0U;
      previous_heart_cnt = current_heart_cnt;
    }

    const bool heartbeat_timed_out =
      (last_us == 0) ? false : (elapsed > std::chrono::duration_cast<std::chrono::microseconds>(kHeartbeatTimeout).count());
    const bool stale_heartbeat = stale_cycles >= 5U;

    if (!direct_mode_active_.load())
    {
      if (heartbeat_timed_out || request_force_direct_.load())
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        SendHandshakeSequence();
      }
      continue;
    }

    if (heartbeat_timed_out || stale_heartbeat)
    {
      direct_mode_active_.store(false);
      request_force_direct_.store(true);
      std::lock_guard<std::mutex> lock(command_mutex_);
      SendHandshakeSequence();
    }
  }
}

}  // namespace titati
