#pragma once

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "can_device/can_receiver.hpp"
#include "can_device/can_sender.hpp"

namespace titati::hardware
{

class TitatiRobot
{
public:
    enum ReadyNext
    {
        READY_WAITING = 0x00U,
        AUTO_LOCOMOTION = 0x01U,
        FORCE_LOCOMOTION = 0x02U,
        FORCE_DIRECT = 0x03U,
        MOTOR_ZERO_CAL = 0x04U,
        BOOT_RECOVERY_MODE = 0x05U,
        ESTOP_MODE = 0x06U,
    };
    struct ChannelInput
    {
        uint32_t timestamp;
        float forward;
        float yaw;
        float pitch;
        float roll;
        float height;
        float split;
        float tilt;
        float forward_accel;
        float yaw_accel;
    };
    explicit TitatiRobot(size_t num_motors, const std::string& interface)
    {
        can_receiver_ = std::make_unique<can_device::MotorsImuCanReceiveApi>(num_motors, interface);
        can_sender_ = std::make_unique<can_device::MotorsCanSendApi>(num_motors, interface);
        motor_num_ = num_motors;
    }

    const std::vector<can_device::api_motor_in_t>& get_motor_packets() const;
    const can_device::api_imu_data_t& get_raw_imu() const;

    std::vector<double> get_joint_q() const;
    std::vector<double> get_joint_v() const;
    std::vector<double> get_joint_t() const;
    std::vector<uint8_t> get_joint_status() const;
    std::array<double, 4> get_imu_quaternion() const;
    std::array<double, 3> get_imu_acceleration() const;
    std::array<double, 3> get_imu_angular_velocity() const;

    bool set_target_joint_t(const std::vector<double> & t);
    bool set_target_joint_mit(const std::vector<double> & q,
                              const std::vector<double> & v,
                              const std::vector<double> & kp,
                              const std::vector<double> & kd,
                              const std::vector<double> & t);
    bool set_motors_sdk(bool if_sdk);

private:
    std::unique_ptr<can_device::MotorsImuCanReceiveApi> can_receiver_;
    std::unique_ptr<can_device::MotorsCanSendApi> can_sender_;
    size_t motor_num_;
};

}  // namespace titati::hardware
