#include "tita_hardware/canfd_router.hpp"

#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <sys/time.h>
#include <thread>
#include <string>

namespace can_device
{

namespace
{
constexpr uint32_t kCanIdCanfdRouter = 0x09FU;
constexpr uint32_t kCanMaskCanfdRouterInfo = 0x0FFU;
constexpr uint32_t kRpcCanId = 0x170U;
constexpr uint16_t kRpcKeyReadyNext = 0x200U;
constexpr uint32_t kReadyWaiting = 0x00U;
constexpr uint32_t kForceDirect = 0x03U;
} // namespace

CanfdRouterCanReceiveApi::CanfdRouterCanReceiveApi()
{
    auto callback = std::bind(&CanfdRouterCanReceiveApi::get_board_can_data, this, std::placeholders::_1);
    canfd_router_can_receive_api_ = std::make_shared<can_device::socket_can::CanDev>(
        canfd_router_can_interface_,
        canfd_router_can_name_,
        canfd_router_can_extended_frame_,
        callback,
        canfd_router_can_rx_is_block_,
        canfd_router_timeout_us_,
        canfd_router_can_id_offset_
    );

    set_forcedirect_can_send_api_ = std::make_shared<can_device::socket_can::CanDev>(
        set_forcedirect_can_interface_,
        set_forcedirect_can_name_,
        set_forcedirect_can_extended_frame_,
        set_forcedirect_can_fd_mode_,
        set_forcedirect_timeout_us_,
        set_forcedirect_can_id_offset_
    );

    register_canfd_router_device_can_filter();
}

CanfdRouterCanReceiveApi::~CanfdRouterCanReceiveApi() = default;

void CanfdRouterCanReceiveApi::set_forcedirect_mode(bool init_flag)
{
    init_flag_.store(init_flag);
    auto_retry_.store(init_flag);
    if (init_flag)
    {
        last_mode_.store(std::numeric_limits<uint32_t>::max());
    }
}

void CanfdRouterCanReceiveApi::register_canfd_router_device_can_filter()
{
    struct can_filter canfd_router_device_rx_filter;
    canfd_router_device_rx_filter.can_id = kCanIdCanfdRouter;
    canfd_router_device_rx_filter.can_mask = kCanMaskCanfdRouterInfo;
    if (canfd_router_can_receive_api_)
    {
        canfd_router_can_receive_api_->set_filter(&canfd_router_device_rx_filter, sizeof(struct can_filter));
    }
}

void CanfdRouterCanReceiveApi::get_board_can_data(std::shared_ptr<struct canfd_frame> recv_frame)
{
    if (!recv_frame || recv_frame->can_id != kCanIdCanfdRouter)
    {
        return;
    }

    std::memcpy(&mode_, recv_frame->data + 4, sizeof(mode_));
    std::memcpy(&heart_cnt_, recv_frame->data + 8, sizeof(heart_cnt_));

    const uint32_t previous_mode = last_mode_.exchange(mode_);
    const uint32_t previous_heart = last_heart_cnt_.exchange(heart_cnt_);

    const auto now = std::chrono::steady_clock::now();
    last_heartbeat_timestamp_us_.store(
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count()));

    static auto last_frame_time = std::chrono::steady_clock::time_point{};
    static auto last_gap_log = std::chrono::steady_clock::time_point{};
    auto gap_duration = std::chrono::steady_clock::duration::zero();
    bool heartbeat_gap_detected = false;
    if (last_frame_time.time_since_epoch().count() != 0)
    {
        gap_duration = now - last_frame_time;
        if (gap_duration > std::chrono::milliseconds(200))
        {
            heartbeat_gap_detected = true;
            if (now - last_gap_log > std::chrono::milliseconds(500))
            {
                last_gap_log = now;
                std::cout << "[ROUTER] Heartbeat gap detected: "
                          << std::chrono::duration_cast<std::chrono::milliseconds>(gap_duration).count()
                          << " ms (mode=" << mode_ << ", auto_retry=" << (auto_retry_.load() ? "true" : "false") << ")."
                          << std::endl;
            }
        }
    }
    last_frame_time = now;

    static auto last_status_log = std::chrono::steady_clock::time_point{};
    if (now - last_status_log > std::chrono::seconds(1))
    {
        std::cout << "[ROUTER] mode=" << mode_ << ", heart_cnt=" << heart_cnt_
                  << ", auto_retry=" << (auto_retry_.load() ? "true" : "false")
                  << ", init_flag=" << (init_flag_.load() ? "true" : "false") << std::endl;
        last_status_log = now;
    }

    bool trigger_handshake = false;
    std::string handshake_reason;
    if (auto_retry_.load() && (mode_ == 1U || mode_ == 2U))
    {
        if (previous_mode != mode_)
        {
            trigger_handshake = true;
            handshake_reason = "mode change";
        }
        else if (previous_heart != std::numeric_limits<uint32_t>::max() && heart_cnt_ < previous_heart)
        {
            trigger_handshake = true;
            handshake_reason = "heartbeat reset";
        }
        else if (heartbeat_gap_detected && gap_duration > std::chrono::milliseconds(500))
        {
            trigger_handshake = true;
            handshake_reason = "heartbeat gap " +
                                std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(gap_duration).count()) +
                                " ms";
        }
    }

    if (trigger_handshake)
    {
        init_flag_.store(true);
        std::cout << "[ROUTER] Triggering FORCE_DIRECT handshake due to " << handshake_reason
                  << ". mode=" << mode_ << ", heart_cnt=" << heart_cnt_ << std::endl;
    }

    if (!init_flag_.load())
    {
        return;
    }

    if (mode_ != 1U && mode_ != 2U)
    {
        std::cout << "[ROUTER] Waiting for MCU to enter locomotion mode before forcing direct control. Current mode="
                  << mode_ << std::endl;
        return;
    }

    if (!set_forcedirect_can_send_api_)
    {
        std::cerr << "[ROUTER] CAN send API not initialised. Cannot request FORCE_DIRECT." << std::endl;
        return;
    }

    struct canfd_frame frame;
    frame.can_id = kRpcCanId;
    frame.len = 10U;
    std::memset(frame.data, 0x00U, sizeof(frame.data));

    uint32_t timestamp = get_current_time();
    uint16_t key = kRpcKeyReadyNext;
    uint32_t value = kReadyWaiting;

    std::memcpy(frame.data, &timestamp, sizeof(timestamp));
    std::memcpy(frame.data + 4, &key, sizeof(key));
    std::memcpy(frame.data + 6, &value, sizeof(value));

    bool ready_waiting_sent = set_forcedirect_can_send_api_->send_can_message(frame);
    if (!ready_waiting_sent)
    {
        std::cerr << "[ROUTER] Failed to send READY_WAITING command to MCU." << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    timestamp = get_current_time();
    value = kForceDirect;

    std::memcpy(frame.data, &timestamp, sizeof(timestamp));
    std::memcpy(frame.data + 4, &key, sizeof(key));
    std::memcpy(frame.data + 6, &value, sizeof(value));

    bool force_direct_sent = set_forcedirect_can_send_api_->send_can_message(frame);
    if (!force_direct_sent)
    {
        std::cerr << "[ROUTER] Failed to send FORCE_DIRECT command to MCU." << std::endl;
    }
    else
    {
        std::cout << "[ROUTER] FORCE_DIRECT handshake sent (ready_waiting="
                  << (ready_waiting_sent ? "ok" : "fail")
                  << ", force_direct=" << (force_direct_sent ? "ok" : "fail")
                  << ")." << std::endl;
    }
    init_flag_.store(false);
}

uint32_t CanfdRouterCanReceiveApi::get_current_time()
{
    struct timeval tv
    {
        0
    };
    gettimeofday(&tv, nullptr);
    return static_cast<uint32_t>(tv.tv_sec * 1000000 + tv.tv_usec);
}

} // namespace can_device

