#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace titati::hardware
{

struct MotorState
{
    double position{0.0};
    double velocity{0.0};
    double torque{0.0};
    uint32_t timestamp{0U};
};

struct ImuState
{
    std::array<double, 4> quaternion{ {1.0, 0.0, 0.0, 0.0} };  // w, x, y, z
    std::array<double, 3> gyroscope{ {0.0, 0.0, 0.0} };
    std::array<double, 3> acceleration{ {0.0, 0.0, 0.0} };
    uint32_t timestamp{0U};
};

struct CombinedState
{
    std::vector<MotorState> motors;
    ImuState imu;
    uint64_t sequence{0U};
};

class TitatiHardware
{
public:
    TitatiHardware(std::string can_interface, std::size_t motor_count);
    ~TitatiHardware();

    TitatiHardware(const TitatiHardware&) = delete;
    TitatiHardware& operator=(const TitatiHardware&) = delete;

    bool Initialize();
    void Shutdown();

    bool IsInitialized() const { return initialized_.load(); }

    CombinedState GetLatestState() const;

    bool SendMitCommand(const std::vector<double>& position,
                        const std::vector<double>& velocity,
                        const std::vector<double>& kp,
                        const std::vector<double>& kd,
                        const std::vector<double>& torque);

    bool SendTorqueCommand(const std::vector<double>& torque);

    bool SetDirectControlMode(bool enable);

private:
    void EnsureDirectMode();
    bool RequestForceDirect();
    void ReceiverLoop();
    void HandleMotorFeedback(std::uint32_t can_id, const std::uint8_t* data, std::uint8_t dlc);
    void HandleImuFeedback(const std::uint8_t* data, std::uint8_t dlc);
    void HandleRouterFeedback(const std::uint8_t* data, std::uint8_t dlc);
    bool SendRpcCommand(std::uint16_t key, std::uint32_t value);
    bool SendFrame(const void* frame, std::size_t length) const;
    std::uint32_t AcquireTimestampUs() const;
    std::uint64_t AcquireSteadyTimestampUs() const;

    const std::string interface_;
    const std::size_t motor_count_;
    std::size_t dof_per_leg_{0};
    std::size_t leg_count_{0};

    int socket_fd_{-1};
    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> direct_mode_requested_{false};
    std::atomic<std::uint64_t> last_force_direct_request_us_{0U};

    std::thread receiver_thread_;

    mutable std::mutex state_mutex_;
    std::vector<MotorState> motor_state_;
    ImuState imu_state_{};
    std::vector<std::uint64_t> leg_last_update_us_;
    std::atomic<uint64_t> state_sequence_{0U};

    std::atomic<std::uint32_t> router_mode_{0U};
    std::atomic<std::uint32_t> router_heartbeat_{0U};
    std::atomic<std::uint64_t> last_router_status_us_{0U};
    mutable std::mutex router_mutex_;
};

}  // namespace titati::hardware
