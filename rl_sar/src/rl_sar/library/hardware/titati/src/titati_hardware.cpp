#include "titati/titati_hardware.hpp"

#include <algorithm>

namespace rl_sar
{

TitatiHardware::TitatiHardware(std::size_t motor_count)
: robot_(std::make_unique<tita_robot>(motor_count)),
  router_(std::make_shared<titati::CanfdRouter>()),
  motor_count_(motor_count)
{
}

TitatiHardware::~TitatiHardware()
{
  DisableDirectControl();
}

bool TitatiHardware::EnableDirectControl()
{
  if (router_)
  {
    router_->RequestForceDirectMode();
  }
  direct_control_enabled_ = robot_->set_motors_sdk(true);
  return direct_control_enabled_;
}

bool TitatiHardware::DisableDirectControl()
{
  if (!direct_control_enabled_)
  {
    return true;
  }
  direct_control_enabled_ = !robot_->set_motors_sdk(false);
  if (router_)
  {
    router_->CancelForceDirectMode();
  }
  return !direct_control_enabled_;
}

TitatiJointState TitatiHardware::ReadJointState() const
{
  TitatiJointState state;
  state.position = robot_->get_joint_q();
  state.velocity = robot_->get_joint_v();
  state.torque = robot_->get_joint_t();
  return state;
}

TitatiImuState TitatiHardware::ReadImuState() const
{
  TitatiImuState imu;
  const auto raw_quat = robot_->get_imu_quaternion();
  imu.quaternion_wxyz = {raw_quat[3], raw_quat[0], raw_quat[1], raw_quat[2]};
  const auto raw_gyro = robot_->get_imu_angular_velocity();
  const auto raw_acc = robot_->get_imu_acceleration();
  std::copy(raw_gyro.begin(), raw_gyro.end(), imu.angular_velocity.begin());
  std::copy(raw_acc.begin(), raw_acc.end(), imu.linear_acceleration.begin());
  return imu;
}

bool TitatiHardware::SendTorqueCommand(const std::vector<double> &torque) const
{
  if (torque.size() != motor_count_)
  {
    return false;
  }
  return robot_->set_target_joint_t(torque);
}

bool TitatiHardware::SendMITCommand(
  const std::vector<double> &position,
  const std::vector<double> &velocity,
  const std::vector<double> &kp,
  const std::vector<double> &kd,
  const std::vector<double> &torque) const
{
  if (position.size() != motor_count_ || velocity.size() != motor_count_ ||
      kp.size() != motor_count_ || kd.size() != motor_count_ ||
      torque.size() != motor_count_)
  {
    return false;
  }
  return robot_->set_target_joint_mit(position, velocity, kp, kd, torque);
}

}  // namespace rl_sar
