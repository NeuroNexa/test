#include "titati_robot.hpp"

namespace titati::hardware
{
TitatiRobot::TitatiRobot(size_t num_motors, const std::string& interface)
{
    can_receiver_ = std::make_unique<can_device::MotorsImuCanReceiveApi>(num_motors, interface);
    can_sender_ = std::make_unique<can_device::MotorsCanSendApi>(num_motors, interface);
    motor_num_ = num_motors;
}

const std::vector<can_device::api_motor_in_t>& TitatiRobot::get_motor_packets() const
{
    return *can_receiver_->get_motors_in();
}

const can_device::api_imu_data_t& TitatiRobot::get_raw_imu() const
{
    return *can_receiver_->get_imu_data();
}

std::vector<double> TitatiRobot::get_joint_q() const
{
    auto infos = can_receiver_->get_motors_in();
    std::vector<double> joint;
    joint.reserve(infos->size());
    for (const auto & info : *infos)
    {
        joint.push_back(info.position);
    }
    return joint;
}

std::vector<double> TitatiRobot::get_joint_v() const
{
    auto infos = can_receiver_->get_motors_in();
    std::vector<double> joint;
    joint.reserve(infos->size());
    for (const auto & info : *infos)
    {
        joint.push_back(info.velocity);
    }
    return joint;
}

std::vector<double> TitatiRobot::get_joint_t() const
{
    auto infos = can_receiver_->get_motors_in();
    std::vector<double> joint;
    joint.reserve(infos->size());
    for (const auto & info : *infos)
    {
        joint.push_back(info.torque);
    }
    return joint;
}

std::vector<uint8_t> TitatiRobot::get_joint_status() const
{
    auto infos = can_receiver_->get_motors_status();
    std::vector<uint8_t> joint;
    joint.reserve(8);
    for (size_t id = 0; id < 8; id++)
    {
        joint.push_back(infos->value[id]);
    }
    return joint;
}

std::array<double, 4> TitatiRobot::get_imu_quaternion() const
{
    auto infos = can_receiver_->get_imu_data();
    std::array<double, 4> data{};
    for (size_t i = 0; i < 4; i++)
    {
        data[i] = infos->quaternion[i];
    }
    return data;
}

std::array<double, 3> TitatiRobot::get_imu_acceleration() const
{
    auto infos = can_receiver_->get_imu_data();
    std::array<double, 3> data{};
    for (size_t i = 0; i < 3; i++)
    {
        data[i] = infos->accl[i];
    }
    return data;
}

std::array<double, 3> TitatiRobot::get_imu_angular_velocity() const
{
    auto infos = can_receiver_->get_imu_data();
    std::array<double, 3> data{};
    for (size_t i = 0; i < 3; i++)
    {
        data[i] = infos->gyro[i];
    }
    return data;
}

bool TitatiRobot::set_target_joint_t(const std::vector<double> & t)
{
    std::vector<can_device::motor_out> motors;
    motors.reserve(motor_num_);
    for (size_t id = 0; id < motor_num_; id++)
    {
        can_device::motor_out motor{};
        motor.torque = static_cast<float>(t[id]);
        motor.kp = 0.0f;
        motor.kd = 0.0f;
        motor.velocity = 0.0f;
        motor.position = 0.0f;
        motors.push_back(motor);
    }
    return can_sender_->send_motors_can(motors);
}

bool TitatiRobot::set_target_joint_mit(const std::vector<double> & q,
                                       const std::vector<double> & v,
                                       const std::vector<double> & kp,
                                       const std::vector<double> & kd,
                                       const std::vector<double> & t)
{
    std::vector<can_device::motor_out> motors;
    motors.reserve(motor_num_);
    for (size_t id = 0; id < motor_num_; id++)
    {
        can_device::motor_out motor{};
        motor.torque = static_cast<float>(t[id]);
        motor.kp = static_cast<float>(kp[id]);
        motor.kd = static_cast<float>(kd[id]);
        motor.velocity = static_cast<float>(v[id]);
        motor.position = static_cast<float>(q[id]);
        motors.push_back(motor);
    }
    return can_sender_->send_motors_can(motors);
}

bool TitatiRobot::set_motors_sdk(bool if_sdk)
{
    can_device::RpcRequest rpc_request{};
    rpc_request.key = can_device::SET_READY_NEXT;
    rpc_request.value = READY_WAITING;
    bool return_ok = can_sender_->send_command_can_rpc_request(rpc_request);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    if (if_sdk)
    {
        rpc_request.value = FORCE_DIRECT;
    }
    else
    {
        rpc_request.value = AUTO_LOCOMOTION;
    }
    return_ok &= can_sender_->send_command_can_rpc_request(rpc_request);
    return return_ok;
}

}  // namespace titati::hardware
