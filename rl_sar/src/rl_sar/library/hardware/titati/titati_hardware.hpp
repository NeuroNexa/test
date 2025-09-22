#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "titati_robot.hpp"

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
    std::array<double, 4> quaternion{{1.0, 0.0, 0.0, 0.0}};  // w, x, y, z
    std::array<double, 3> gyroscope{{0.0, 0.0, 0.0}};
    std::array<double, 3> acceleration{{0.0, 0.0, 0.0}};
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
    const std::string interface_;
    const std::size_t motor_count_;

    std::unique_ptr<TitatiRobot> robot_;

    std::atomic<bool> initialized_{false};
    std::atomic<uint64_t> sequence_counter_{0U};
};

}  // namespace titati::hardware
