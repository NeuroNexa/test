/*
 * Copyright (c) 2023 Direct Drive Technology Co., Ltd.
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

#ifndef TITATI_CANFD_ROUTER_HPP_
#define TITATI_CANFD_ROUTER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "protocol/can_utils.hpp"

namespace titati
{

class CanfdRouter
{
public:
  CanfdRouter();
  ~CanfdRouter();

  /// @brief Request the MCU to switch into ForceDirect (SDK) mode.
  void RequestForceDirectMode();

  /// @brief Cancel previously requested direct control mode.
  void CancelForceDirectMode();

private:
  void RegisterFilter();
  void HandleBoardFrame(std::shared_ptr<struct canfd_frame> recv_frame);
  void SendForceDirectCommand(uint32_t value);
  void SendHandshakeSequence();
  uint32_t GetCurrentTime() const;
  void StartMonitorThread();
  void StopMonitorThread();
  void MonitorLoop();

  std::atomic<bool> request_force_direct_{false};
  std::atomic<bool> direct_control_requested_{false};
  std::atomic<bool> direct_mode_active_{false};
  std::atomic<uint32_t> last_heart_cnt_{0};
  std::atomic<int64_t> last_heartbeat_time_us_{0};
  std::atomic<int64_t> last_command_time_us_{0};
  std::atomic<bool> running_{true};

  std::string router_interface_{"can0"};
  std::string router_name_{"titati_canfd"};
  bool router_extended_frame_{false};
  bool router_rx_block_{false};
  int64_t router_timeout_us_{3'000'000L};
  uint8_t router_id_offset_{0x00U};

  std::string command_interface_{"can0"};
  std::string command_name_{"titati_force_direct"};
  int64_t command_timeout_us_{3'000'000L};
  uint8_t command_id_offset_{0x00U};
  bool command_extended_frame_{false};
  bool command_fd_mode_{true};
  std::mutex command_mutex_;
  std::thread monitor_thread_;

  std::shared_ptr<can_device::socket_can::CanDev> router_dev_;
  std::shared_ptr<can_device::socket_can::CanDev> command_dev_;

  uint32_t mode_{0U};
  uint32_t heart_cnt_{0U};
};

}  // namespace titati

#endif  // TITATI_CANFD_ROUTER_HPP_
