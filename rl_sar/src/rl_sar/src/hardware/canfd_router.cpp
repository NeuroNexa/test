#include "tita_hardware/canfd_router.hpp"

#include <cstring>
#include <functional>
#include <limits>
#include <sys/time.h>
#include <thread>

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

    const uint32_t previous_mode = last_mode_.load();
    if (auto_retry_.load() && (mode_ == 1U || mode_ == 2U) && previous_mode != mode_)
    {
        init_flag_.store(true);
    }
    last_mode_.store(mode_);

    if (!init_flag_.load())
    {
        return;
    }

    if (mode_ != 1U && mode_ != 2U)
    {
        return;
    }

    if (!set_forcedirect_can_send_api_)
    {
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

    set_forcedirect_can_send_api_->send_can_message(frame);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    timestamp = get_current_time();
    value = kForceDirect;

    std::memcpy(frame.data, &timestamp, sizeof(timestamp));
    std::memcpy(frame.data + 4, &key, sizeof(key));
    std::memcpy(frame.data + 6, &value, sizeof(value));

    set_forcedirect_can_send_api_->send_can_message(frame);
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

