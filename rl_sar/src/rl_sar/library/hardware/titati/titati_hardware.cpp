#include "titati_hardware.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace titati::hardware
{
namespace
{
constexpr float kGravity = 9.81f;
}

TitatiHardware::TitatiHardware(std::string can_interface, std::size_t motor_count)
    : interface_(std::move(can_interface)),
      motor_count_(motor_count)
{
    if (motor_count_ == 0)
    {
        throw std::invalid_argument("motor_count must be greater than zero");
    }
}

TitatiHardware::~TitatiHardware()
{
    Shutdown();
}

bool TitatiHardware::Initialize()
{
    if (initialized_.load())
    {
        return true;
    }

    try
    {
        robot_ = std::make_unique<TitatiRobot>(motor_count_, interface_);
    }
    catch (const std::exception& ex)
    {
        std::cerr << "[TitatiHardware] Failed to create TitatiRobot: " << ex.what() << std::endl;
        return false;
    }

    initialized_.store(true);
    return true;
}

void TitatiHardware::Shutdown()
{
    robot_.reset();
    initialized_.store(false);
}

CombinedState TitatiHardware::GetLatestState() const
{
    CombinedState output;
    output.motors.resize(motor_count_);

    if (!robot_)
    {
        return output;
    }

    const auto& packets = robot_->get_motor_packets();
    for (std::size_t i = 0; i < motor_count_ && i < packets.size(); ++i)
    {
        output.motors[i].position = packets[i].position;
        output.motors[i].velocity = packets[i].velocity;
        output.motors[i].torque = packets[i].torque;
        output.motors[i].timestamp = packets[i].timestamp;
    }

    const auto& imu = robot_->get_raw_imu();
    output.imu.timestamp = imu.timestamp;
    output.imu.acceleration = {imu.accl[0] * kGravity, imu.accl[1] * kGravity, imu.accl[2] * kGravity};
    output.imu.gyroscope = {imu.gyro[0], imu.gyro[1], imu.gyro[2]};
    output.imu.quaternion = {imu.quaternion[3], imu.quaternion[0], imu.quaternion[1], imu.quaternion[2]};

    output.sequence = sequence_counter_.fetch_add(1, std::memory_order_relaxed) + 1;
    return output;
}

bool TitatiHardware::SendMitCommand(const std::vector<double>& position,
                                    const std::vector<double>& velocity,
                                    const std::vector<double>& kp,
                                    const std::vector<double>& kd,
                                    const std::vector<double>& torque)
{
    if (!initialized_.load() || !robot_)
    {
        return false;
    }

    if (position.size() != motor_count_ || velocity.size() != motor_count_ ||
        kp.size() != motor_count_ || kd.size() != motor_count_ || torque.size() != motor_count_)
    {
        std::cerr << "[TitatiHardware] Invalid command vector size" << std::endl;
        return false;
    }

    return robot_->set_target_joint_mit(position, velocity, kp, kd, torque);
}

bool TitatiHardware::SendTorqueCommand(const std::vector<double>& torque)
{
    std::vector<double> zeros(motor_count_, 0.0);
    return SendMitCommand(zeros, zeros, zeros, zeros, torque);
}

bool TitatiHardware::SetDirectControlMode(bool enable)
{
    if (!initialized_.load() || !robot_)
    {
        return false;
    }

    if (!enable)
    {
        return robot_->set_motors_sdk(false);
    }

    constexpr int kMaxAttempts = 6;
    constexpr auto kAttemptPause = std::chrono::milliseconds(150);
    constexpr auto kStreamTimeout = std::chrono::milliseconds(800);
    constexpr auto kPollInterval = std::chrono::milliseconds(50);

    for (int attempt = 0; attempt < kMaxAttempts; ++attempt)
    {
        if (!robot_->set_motors_sdk(true))
        {
            std::this_thread::sleep_for(kAttemptPause);
            continue;
        }

        const auto deadline = std::chrono::steady_clock::now() + kStreamTimeout;
        while (std::chrono::steady_clock::now() < deadline)
        {
            const auto& packets = robot_->get_motor_packets();
            std::size_t ready = 0;
            for (std::size_t i = 0; i < motor_count_ && i < packets.size(); ++i)
            {
                if (packets[i].timestamp != 0U)
                {
                    ++ready;
                }
            }

            if (ready >= motor_count_)
            {
                return true;
            }

            std::this_thread::sleep_for(kPollInterval);
        }
    }

    return false;
}

}  // namespace titati::hardware
