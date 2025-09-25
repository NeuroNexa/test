#include "tita_robot/canfd_router_bridge.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <utility>

#include <sys/time.h>

namespace can_device
{
namespace
{
constexpr uint32_t CAN_ID_CANFD_ROUTER = 0x09FU;
constexpr uint32_t CAN_ID_RPC_REQUEST = 0x170U;
constexpr uint16_t RPC_KEY_SET_READY_NEXT = 0x200U;
constexpr uint32_t RPC_VALUE_READY_WAITING = 0x00U;
constexpr uint32_t RPC_VALUE_FORCE_DIRECT = 0x03U;
constexpr int64_t MAX_TIME_OUT_US = 3'000'000L;
}  // namespace

CanfdRouterBridge::CanfdRouterBridge(const std::string &can_interface,
                                     uint8_t can_id_offset)
  : can_interface_(can_interface),
    can_name_("canfd_router"),
    can_extended_frame_(false),
    can_rx_block_(false),
    timeout_us_(MAX_TIME_OUT_US),
    can_id_offset_(can_id_offset),
    pending_force_direct_(false)
{
  using namespace std::placeholders;
  router_can_ = std::make_shared<can_device::socket_can::CanDev>(
    can_interface_, can_name_, can_extended_frame_,
    std::bind(&CanfdRouterBridge::handle_router_frame, this, _1),
    can_rx_block_, timeout_us_, can_id_offset_);

  rpc_sender_ = std::make_shared<can_device::socket_can::CanDev>(
    can_interface_, "canfd_router_rpc", false, true, timeout_us_, can_id_offset_);

  register_canfd_router_device_can_filter();
}

void CanfdRouterBridge::request_force_direct()
{
  pending_force_direct_.store(true);
}

void CanfdRouterBridge::register_canfd_router_device_can_filter()
{
  struct can_filter filter;
  filter.can_id = CAN_ID_CANFD_ROUTER;
  filter.can_mask = 0x0FFU;
  if (router_can_)
  {
    router_can_->set_filter(&filter, sizeof(struct can_filter));
  }
}

void CanfdRouterBridge::handle_router_frame(std::shared_ptr<struct canfd_frame> recv_frame)
{
  if (!recv_frame || recv_frame->can_id != CAN_ID_CANFD_ROUTER)
  {
    return;
  }

  if (!pending_force_direct_.load())
  {
    return;
  }

  uint32_t mode = 0U;
  std::memcpy(&mode, recv_frame->data + 4, sizeof(mode));
  if (mode != 1U && mode != 2U)
  {
    return;
  }

  send_force_direct_sequence();
  pending_force_direct_.store(false);
}

void CanfdRouterBridge::send_force_direct_sequence()
{
  if (!rpc_sender_)
  {
    std::cerr << "[CanfdRouterBridge] RPC sender not available" << std::endl;
    return;
  }

  std::lock_guard<std::mutex> lock(rpc_mutex_);

  struct canfd_frame frame;
  std::memset(&frame, 0x00, sizeof(frame));
  frame.can_id = CAN_ID_RPC_REQUEST + can_id_offset_;
  frame.len = 10U;

  uint16_t key = RPC_KEY_SET_READY_NEXT;
  uint32_t value = RPC_VALUE_READY_WAITING;
  uint32_t timestamp = get_current_time();
  std::memcpy(frame.data, &timestamp, sizeof(timestamp));
  std::memcpy(frame.data + 4, &key, sizeof(key));
  std::memcpy(frame.data + 6, &value, sizeof(value));

  try
  {
    rpc_sender_->send_can_message(frame);
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[CanfdRouterBridge] Failed to request READY_WAITING: " << ex.what() << std::endl;
    return;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  value = RPC_VALUE_FORCE_DIRECT;
  timestamp = get_current_time();
  std::memcpy(frame.data, &timestamp, sizeof(timestamp));
  std::memcpy(frame.data + 4, &key, sizeof(key));
  std::memcpy(frame.data + 6, &value, sizeof(value));

  try
  {
    rpc_sender_->send_can_message(frame);
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[CanfdRouterBridge] Failed to request FORCE_DIRECT: " << ex.what() << std::endl;
  }
}

uint32_t CanfdRouterBridge::get_current_time() const
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return static_cast<uint32_t>(tv.tv_sec * 1000000 + tv.tv_usec);
}

}  // namespace can_device
