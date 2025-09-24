#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <limits>

#include "tita_hardware/protocol/can_utils.hpp"

namespace can_device
{

class CanfdRouterCanReceiveApi
{
public:
    CanfdRouterCanReceiveApi();
    ~CanfdRouterCanReceiveApi();

    void set_forcedirect_mode(bool init_flag);

private:
    void register_canfd_router_device_can_filter();
    void get_board_can_data(std::shared_ptr<struct canfd_frame> recv_frame);
    inline uint32_t get_current_time();

    std::string canfd_router_can_interface_ {"can0"};
    std::string canfd_router_can_name_ {"canfd_can"};
    bool canfd_router_can_extended_frame_ {false};
    bool canfd_router_can_rx_is_block_ {false};
    int64_t canfd_router_timeout_us_ {3'000'000L};
    uint8_t canfd_router_can_id_offset_ {0x00U};

    std::shared_ptr<can_device::socket_can::CanDev> canfd_router_can_receive_api_;

    std::string set_forcedirect_can_interface_ {"can0"};
    std::string set_forcedirect_can_name_ {"set_forcedirect_can"};
    int64_t set_forcedirect_timeout_us_ {3'000'000L};
    uint8_t set_forcedirect_can_id_offset_ {0x00U};
    bool set_forcedirect_can_extended_frame_ {false};
    bool set_forcedirect_can_fd_mode_ {true};

    std::shared_ptr<can_device::socket_can::CanDev> set_forcedirect_can_send_api_;

    uint32_t mode_ {0U};
    uint32_t heart_cnt_ {0U};
    std::atomic_bool init_flag_ {false};
    std::atomic_bool auto_retry_ {false};
    std::atomic<uint32_t> last_mode_ {std::numeric_limits<uint32_t>::max()};
    std::atomic<uint32_t> last_heart_cnt_ {std::numeric_limits<uint32_t>::max()};
    std::atomic<uint64_t> last_heartbeat_timestamp_us_ {0};
};

} // namespace can_device

