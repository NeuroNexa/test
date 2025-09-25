#include "tita_robot/tita_robot.hpp"
#include "tita_robot/canfd_router_bridge.hpp"

#include <iostream>
#include <utility>

tita_robot::tita_robot(size_t num_motors,
                       const std::string &can_interface,
                       bool enable_canfd_router)
{
  can_receiver_ = std::make_unique<can_device::MotorsImuCanReceiveApi>(num_motors, can_interface);
  can_sender_ = std::make_unique<can_device::MotorsCanSendApi>(num_motors, can_interface);
  if (enable_canfd_router)
  {
    try
    {
      can_router_ = std::make_unique<can_device::CanfdRouterBridge>(can_interface);
    }
    catch (const std::exception &ex)
    {
      std::cerr << "[tita_robot] Failed to initialize CAN-FD router bridge on interface '"
                << can_interface << "': " << ex.what() << std::endl;
    }
  }
  motor_num_ = num_motors;
}

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
  std::vector<can_device::motor_out> motors;
  for (size_t id = 0; id < motor_num_; id++) {
    can_device::motor_out motor;
    motor.torque = t[id];
    motor.kp = 0.0f;
    motor.kd = 0.0f;
    motor.velocity = 0.0f;
    motor.position = 0.0f;
    motors.push_back(motor);
  }
  return can_sender_->send_motors_can(motors);
}

bool tita_robot::set_target_joint_mit(
  const std::vector<double> & q, const std::vector<double> & v, const std::vector<double> & kp,
  const std::vector<double> & kd, const std::vector<double> & t)
{
  std::vector<can_device::motor_out> motors;
  for (size_t id = 0; id < motor_num_; id++) {
    can_device::motor_out motor;
    motor.torque = t[id];
    motor.kp = kp[id];
    motor.kd = kd[id];
    motor.velocity = v[id];
    motor.position = q[id];
    motors.push_back(motor);
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
  if (if_sdk && can_router_)
  {
    can_router_->request_force_direct();
  }
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

bool tita_robot::has_canfd_router() const
{
  return static_cast<bool>(can_router_);
}

void tita_robot::request_router_force_direct()
{
  if (can_router_)
  {
    can_router_->request_force_direct();
  }
}

void tita_robot::set_router_status_callback(std::function<void(const std::string &)> callback)
{
  if (can_router_)
  {
    can_router_->set_status_callback(std::move(callback));
  }
}
