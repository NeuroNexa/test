#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include "titati/protocol/can_utils.hpp"

namespace titati
{

class ForceDirectManager
{
public:
  ForceDirectManager();
  ~ForceDirectManager();

  void start();
  void stop();

private:
  void handle_frame(std::shared_ptr<struct canfd_frame> frame);
  void send_force_direct_sequence();
  uint32_t current_time_us() const;
  void initial_wakeup();

  std::atomic<bool> running_{false};
  std::atomic<bool> pending_sequence_{false};
  std::shared_ptr<can_device::socket_can::CanDev> router_receiver_;
  std::shared_ptr<can_device::socket_can::CanDev> rpc_sender_;
  std::thread wake_thread_;
};

}  // namespace titati
