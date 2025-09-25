#ifndef TITA_ROBOT_CANFD_ROUTER_BRIDGE_HPP_
#define TITA_ROBOT_CANFD_ROUTER_BRIDGE_HPP_

#include <atomic>
#include <cstdint>
#include <chrono>
#include <functional>
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
                             uint8_t can_id_offset = 0x00U,
                             bool auto_retry = true);
  ~CanfdRouterBridge() = default;

  void request_force_direct();
  void set_auto_retry(bool enable);
  void set_status_callback(std::function<void(const std::string &)> callback);

private:
  void register_canfd_router_device_can_filter();
  void handle_router_frame(std::shared_ptr<struct canfd_frame> recv_frame);
  void send_force_direct_sequence();
  uint32_t get_current_time() const;
  void emit_status(const std::string &message);

  std::string can_interface_;
  std::string can_name_;
  bool can_extended_frame_;
  bool can_rx_block_;
  int64_t timeout_us_;
  uint8_t can_id_offset_;

  std::shared_ptr<can_device::socket_can::CanDev> router_can_;
  std::shared_ptr<can_device::socket_can::CanDev> rpc_sender_;
  std::atomic<bool> pending_force_direct_;
  std::atomic<bool> force_direct_latched_;
  std::atomic<uint32_t> last_router_mode_;
  bool auto_retry_;
  std::chrono::steady_clock::time_point last_force_direct_request_;
  std::chrono::milliseconds min_retry_interval_;
  std::function<void(const std::string &)> status_callback_;
  std::mutex rpc_mutex_;
};

}  // namespace can_device

#endif  // TITA_ROBOT_CANFD_ROUTER_BRIDGE_HPP_
