#ifndef TITA_ROBOT_CANFD_ROUTER_BRIDGE_HPP_
#define TITA_ROBOT_CANFD_ROUTER_BRIDGE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "protocol/can_utils.hpp"

namespace can_device
{

class CanfdRouterBridge
{
public:
  explicit CanfdRouterBridge(const std::string &can_interface = "can0",
                             uint8_t can_id_offset = 0x00U);
  ~CanfdRouterBridge() = default;

  void request_force_direct();

private:
  void register_canfd_router_device_can_filter();
  void handle_router_frame(std::shared_ptr<struct canfd_frame> recv_frame);
  void send_force_direct_sequence();
  uint32_t get_current_time() const;

  std::string can_interface_;
  std::string can_name_;
  bool can_extended_frame_;
  bool can_rx_block_;
  int64_t timeout_us_;
  uint8_t can_id_offset_;

  std::shared_ptr<can_device::socket_can::CanDev> router_can_;
  std::shared_ptr<can_device::socket_can::CanDev> rpc_sender_;
  std::atomic<bool> pending_force_direct_;
  std::mutex rpc_mutex_;
};

}  // namespace can_device

#endif  // TITA_ROBOT_CANFD_ROUTER_BRIDGE_HPP_
