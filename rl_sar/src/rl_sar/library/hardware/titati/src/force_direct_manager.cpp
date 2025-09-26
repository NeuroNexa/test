#include "titati/force_direct_manager.hpp"

#include <chrono>
#include <functional>
#include <cstring>
#include <sys/time.h>

namespace titati
{

namespace
{
constexpr uint16_t kRpcKeyReadyNext = 0x200;
constexpr uint32_t kRpcValueIdle = 0x00U;
constexpr uint32_t kRpcValueForceDirect = 0x03U;
constexpr uint32_t kCanIdRouter = 0x09FU;
constexpr uint32_t kCanIdRpc = 0x170U;
constexpr uint32_t kModeAuto = 0x01U;
constexpr uint32_t kModeForce = 0x02U;
constexpr int64_t kTimeoutUs = 3'000'000L;
}  // namespace

ForceDirectManager::ForceDirectManager()
{
  auto callback = std::bind(&ForceDirectManager::handle_frame, this, std::placeholders::_1);
  router_receiver_ = std::make_shared<can_device::socket_can::CanDev>(
    "can0", "titati_router_listener", false, callback, false, kTimeoutUs, 0x00U);

  struct can_filter filter
  {
  };
  filter.can_id = kCanIdRouter;
  filter.can_mask = 0x0FFU;
  router_receiver_->set_filter(&filter, sizeof(filter));

  rpc_sender_ = std::make_shared<can_device::socket_can::CanDev>(
    "can0", "titati_router_sender", false, true, kTimeoutUs, 0x00U);
}

ForceDirectManager::~ForceDirectManager()
{
  stop();
}

void ForceDirectManager::start()
{
  if (running_.exchange(true)) {
    return;
  }
  pending_sequence_.store(true);
  initial_wakeup();
}

void ForceDirectManager::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  pending_sequence_.store(false);
  if (wake_thread_.joinable()) {
    wake_thread_.join();
  }
}

void ForceDirectManager::initial_wakeup()
{
  wake_thread_ = std::thread([this]() {
    for (int i = 0; i < 5 && running_.load(); ++i) {
      send_force_direct_sequence();
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  });
}

void ForceDirectManager::handle_frame(std::shared_ptr<struct canfd_frame> frame)
{
  if (!running_.load() || frame == nullptr) {
    return;
  }

  if (frame->can_id != kCanIdRouter) {
    return;
  }

  uint32_t mode = 0;
  std::memcpy(&mode, frame->data + 4, sizeof(mode));
  if (mode == kModeAuto || mode == kModeForce) {
    pending_sequence_.store(true);
    send_force_direct_sequence();
  }
}

void ForceDirectManager::send_force_direct_sequence()
{
  if (!running_.load() || !pending_sequence_.exchange(false)) {
    return;
  }

  struct canfd_frame frame
  {
  };
  frame.can_id = kCanIdRpc;
  frame.len = 10U;
  std::memset(frame.data, 0x00, sizeof(frame.data));

  uint32_t timestamp = current_time_us();
  uint16_t key = kRpcKeyReadyNext;
  uint32_t value = kRpcValueIdle;

  std::memcpy(frame.data, &timestamp, sizeof(timestamp));
  std::memcpy(frame.data + 4, &key, sizeof(key));
  std::memcpy(frame.data + 6, &value, sizeof(value));
  rpc_sender_->send_can_message(frame);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  timestamp = current_time_us();
  value = kRpcValueForceDirect;
  std::memcpy(frame.data, &timestamp, sizeof(timestamp));
  std::memcpy(frame.data + 6, &value, sizeof(value));
  rpc_sender_->send_can_message(frame);
}

uint32_t ForceDirectManager::current_time_us() const
{
  struct timeval tv
  {
  };
  gettimeofday(&tv, nullptr);
  return static_cast<uint32_t>(tv.tv_sec * 1'000'000 + tv.tv_usec);
}

}  // namespace titati
