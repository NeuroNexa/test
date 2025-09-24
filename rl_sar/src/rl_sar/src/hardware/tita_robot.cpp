#include "tita_hardware/tita_robot.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <thread>

std::vector<double> tita_robot::get_joint_q() const
{
  auto infos = can_receiver_->get_motors_in();
  std::vector<double> joint;
  for (const auto & info : *infos) {
    joint.push_back(info.position);
  }
  return joint;
}

std::vector<double> tita_robot::get_joint_v() const
{
  auto infos = can_receiver_->get_motors_in();
  std::vector<double> joint;
  for (const auto & info : *infos) {
    joint.push_back(info.velocity);
  }
  return joint;
}

std::vector<double> tita_robot::get_joint_t() const
{
  auto infos = can_receiver_->get_motors_in();
  std::vector<double> joint;
  for (const auto & info : *infos) {
    joint.push_back(info.torque);
  }
  return joint;
}

std::vector<uint8_t> tita_robot::get_joint_status() const  // TODO: why 8
{
  auto infos = can_receiver_->get_motors_status();
  std::vector<uint8_t> joint;
  for (size_t id = 0; id < 8; id++) {
    joint.push_back(infos->value[id]);
  }
  return joint;
}

std::array<double, 4> tita_robot::get_imu_quaternion() const
{
  auto infos = can_receiver_->get_imu_data();
  std::array<double, 4> data;
  for (size_t i = 0; i < 4; i++) {
    data[i] = infos->quaternion[i];
  }
  return data;
}

std::array<double, 3> tita_robot::get_imu_acceleration() const
{
  auto infos = can_receiver_->get_imu_data();
  std::array<double, 3> data;
  for (size_t i = 0; i < 3; i++) {
    data[i] = infos->accl[i];
  }
  return data;
}

std::array<double, 3> tita_robot::get_imu_angular_velocity() const
{
  auto infos = can_receiver_->get_imu_data();
  std::array<double, 3> data;
  for (size_t i = 0; i < 3; i++) {
    data[i] = infos->gyro[i];
  }
  return data;
}

bool tita_robot::set_target_joint_t(const std::vector<double> & t)
{
  if (t.size() != motor_num_) {
    std::cerr << "[tita_robot] Torque vector size mismatch. Expected " << motor_num_
              << " values, received " << t.size() << std::endl;
    return false;
  }

  std::vector<can_device::motor_out> motors;
  motors.reserve(motor_num_);
  bool non_finite_detected = false;
  std::ostringstream non_finite_details;

  for (size_t id = 0; id < motor_num_; id++) {
    can_device::motor_out motor{};
    const double torque_value = t[id];
    if (!std::isfinite(torque_value)) {
      non_finite_detected = true;
      if (non_finite_details.tellp() == std::streampos(0)) {
        non_finite_details << "Non-finite torques:";
      }
      non_finite_details << " (" << id << "=" << torque_value << ")";
    }

    motor.torque = std::isfinite(torque_value) ? static_cast<float>(torque_value) : 0.0f;
    motor.kp = 0.0f;
    motor.kd = 0.0f;
    motor.velocity = 0.0f;
    motor.position = 0.0f;
    motors.push_back(motor);
  }

  if (non_finite_detected) {
    std::cerr << "[tita_robot] " << non_finite_details.str()
              << ". Replaced with zero before sending." << std::endl;
  }

  return can_sender_->send_motors_can(motors);
}

bool tita_robot::set_target_joint_mit(
  const std::vector<double> & q, const std::vector<double> & v, const std::vector<double> & kp,
  const std::vector<double> & kd, const std::vector<double> & t)
{
  auto validate_size = [&](const std::vector<double> & data, const char * name) {
    if (data.size() != motor_num_) {
      std::cerr << "[tita_robot] " << name << " vector size mismatch. Expected " << motor_num_
                << " values, received " << data.size() << std::endl;
      return false;
    }
    return true;
  };

  if (!validate_size(q, "position") || !validate_size(v, "velocity") ||
      !validate_size(kp, "kp") || !validate_size(kd, "kd") || !validate_size(t, "torque")) {
    return false;
  }

  std::vector<can_device::motor_out> motors;
  motors.reserve(motor_num_);
  bool non_finite_detected = false;
  std::ostringstream non_finite_details;

  for (size_t id = 0; id < motor_num_; id++) {
    can_device::motor_out motor{};
    const double values[5] = {q[id], v[id], kp[id], kd[id], t[id]};
    bool has_nan = false;
    for (double value : values) {
      if (!std::isfinite(value)) {
        has_nan = true;
        break;
      }
    }

    if (has_nan) {
      non_finite_detected = true;
      if (non_finite_details.tellp() == std::streampos(0)) {
        non_finite_details << "Non-finite MIT targets:";
      }
      non_finite_details << " (joint " << id
                         << ": q=" << q[id]
                         << ", dq=" << v[id]
                         << ", kp=" << kp[id]
                         << ", kd=" << kd[id]
                         << ", tau=" << t[id] << ")";
    }

    motor.position = std::isfinite(q[id]) ? static_cast<float>(q[id]) : 0.0f;
    motor.velocity = std::isfinite(v[id]) ? static_cast<float>(v[id]) : 0.0f;
    motor.kp = std::isfinite(kp[id]) ? static_cast<float>(kp[id]) : 0.0f;
    motor.kd = std::isfinite(kd[id]) ? static_cast<float>(kd[id]) : 0.0f;
    motor.torque = std::isfinite(t[id]) ? static_cast<float>(t[id]) : 0.0f;
    motors.push_back(motor);
  }

  if (non_finite_detected) {
    std::cerr << "[tita_robot] " << non_finite_details.str()
              << ". Replaced with zero before sending." << std::endl;
  }

  return can_sender_->send_motors_can(motors);
}

// bool tita_robot::set_board_mode(ReadyNext mode)
// {
//   can_device::RpcRequest rpc_request;
//   rpc_request.key = can_device::SET_READY_NEXT;
//   rpc_request.value = mode;
//   return can_sender_->send_command_can_rpc_request(rpc_request);
// }

bool tita_robot::set_motors_sdk(bool if_sdk)
{
  can_device::RpcRequest rpc_request;
  rpc_request.key = can_device::SET_READY_NEXT;
  rpc_request.value = READY_WAITING;
  bool return_ok = can_sender_->send_command_can_rpc_request(rpc_request);
  std::this_thread::sleep_for(std::chrono::microseconds(100));
  if (if_sdk) {
    rpc_request.value = FORCE_DIRECT;
  } else {
    rpc_request.value = AUTO_LOCOMOTION;
  }
  return_ok &= can_sender_->send_command_can_rpc_request(rpc_request);
  return return_ok;
}

bool tita_robot::set_rc_input(ChannelInput input)
{
  can_device::ChannelInput can_input;
  can_input.forward = input.forward;
  can_input.yaw = input.yaw;
  can_input.pitch = input.pitch;
  can_input.roll = input.roll;
  can_input.height = input.height;
  can_input.split = input.split;
  can_input.tilt = input.tilt;
  can_input.forward_accel = input.forward_accel;
  can_input.yaw_accel = input.yaw_accel;
  return can_sender_->send_command_can_channel_input(can_input);
}

bool tita_robot::set_robot_stand(bool stand)
{
  can_device::RpcRequest rpc_request;
  rpc_request.key = can_device::SET_STAND_MODE;
  rpc_request.value = !stand ? 0x01U : 0x02U;
  return can_sender_->send_command_can_rpc_request(rpc_request);
}

bool tita_robot::set_robot_jump(bool jump)
{
  can_device::RpcRequest rpc_request;
  rpc_request.key = can_device::SET_JUMP;
  rpc_request.value = !jump ? 0x01U : 0x02U;
  return can_sender_->send_command_can_rpc_request(rpc_request);
}

bool tita_robot::set_robot_stop()
{
  can_device::RpcRequest rpc_request;
  rpc_request.value = 0x01U;
  rpc_request.key = can_device::SET_STAND_MODE;
  bool return_ok = can_sender_->send_command_can_rpc_request(rpc_request);
  std::this_thread::sleep_for(std::chrono::microseconds(200));
  rpc_request.value = 0x02U;
  rpc_request.key = can_device::SET_JUMP;
  return_ok &= can_sender_->send_command_can_rpc_request(rpc_request);
  return true;
}
