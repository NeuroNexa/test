#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include "titati_sdk/can_receiver.hpp"
#include "titati_sdk/can_sender.hpp"

namespace titati
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

    explicit TitatiRobot(std::size_t num_motors)
    {
        can_receiver_ = std::make_unique<can_device::MotorsImuCanReceiveApi>(num_motors);
        can_sender_ = std::make_unique<can_device::MotorsCanSendApi>(num_motors);
        motor_num_ = num_motors;
    }

    TitatiRobot(const TitatiRobot &) = delete;
    TitatiRobot &operator=(const TitatiRobot &) = delete;
    TitatiRobot(TitatiRobot &&) = delete;
    TitatiRobot &operator=(TitatiRobot &&) = delete;
    ~TitatiRobot() = default;

    std::vector<double> get_joint_q() const;
    std::vector<double> get_joint_v() const;
    std::vector<double> get_joint_t() const;
    std::vector<uint8_t> get_joint_status() const;

    std::array<double, 4> get_imu_quaternion() const;
    std::array<double, 3> get_imu_acceleration() const;
    std::array<double, 3> get_imu_angular_velocity() const;

    bool set_target_joint_t(const std::vector<double> &t);
    bool set_target_joint_mit(const std::vector<double> &q, const std::vector<double> &v,
        const std::vector<double> &kp, const std::vector<double> &kd, const std::vector<double> &t);

    bool set_motors_sdk(bool if_sdk);
    bool set_rc_input(ChannelInput input);
    bool set_robot_stand(bool stand);
    bool set_robot_jump(bool jump);
    bool set_robot_stop();

    std::size_t motor_count() const { return motor_num_; }

private:
    std::unique_ptr<can_device::MotorsImuCanReceiveApi> can_receiver_;
    std::unique_ptr<can_device::MotorsCanSendApi> can_sender_;
    std::size_t motor_num_;
};

} // namespace titati

