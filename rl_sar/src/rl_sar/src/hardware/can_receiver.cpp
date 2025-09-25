// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tita_hardware/can_receiver.hpp"

namespace can_device
{

void MotorsImuCanReceiveApi::register_motors_device_can_filter()
{
  // if (!this->is_running.load()) {
  //   throw std::runtime_error("MotorsImuCanReceiveApi is not running");
  // }
  struct can_filter motors_device_rx_filter;
  motors_device_rx_filter.can_id = CAN_ID_MOTOR_IN;
  motors_device_rx_filter.can_mask = CAN_MASK_AS_MOTORS_INFO;
  this->motors_can_receive_api->set_filter(&motors_device_rx_filter, sizeof(struct can_filter));
}

void MotorsImuCanReceiveApi::board_can_data_callback(std::shared_ptr<struct canfd_frame> recv_frame)
{
  if (recv_frame->can_id >= CAN_ID_MOTOR_IN &&
      recv_frame->can_id < CAN_ID_MOTOR_IN + total_motors_) {
    motors_data_callback(recv_frame);
  } else if (recv_frame->can_id == CAN_ID_IMU1) {
    imu_data_callback(recv_frame);
  } else {
    return;
  }
}
void MotorsImuCanReceiveApi::motors_data_callback(std::shared_ptr<struct canfd_frame> recv_frame)
{
  constexpr size_t kBytesPerMotor = sizeof(api_motor_in_t);
  if (recv_frame->len < kBytesPerMotor) {
    return;
  }

  const size_t motors_in_frame = recv_frame->len / kBytesPerMotor;
  if (motors_in_frame == 0) {
    return;
  }

  const size_t frame_index = recv_frame->can_id - CAN_ID_MOTOR_IN;

  std::unique_lock<std::shared_mutex> lock(motors_in_mutex_);
  observed_frame_count_ = std::max(observed_frame_count_, frame_index + 1);
  size_t frames_in_cycle = observed_frame_count_ == 0 ? 1 : observed_frame_count_;
  size_t motors_per_frame = (total_motors_ + frames_in_cycle - 1) / frames_in_cycle;
  motors_per_frame = std::max<size_t>(1, std::min(motors_per_frame, motors_in_frame));

  size_t motor_index_start = frame_index * motors_per_frame;
  if (motor_index_start >= motors_in_.size()) {
    return;
  }

  for (size_t i = 0; i < motors_in_frame; i++) {
    api_motor_in_t motor;
    std::memcpy(&motor, recv_frame->data + kBytesPerMotor * i, sizeof(api_motor_in_t));
    size_t motor_index = motor_index_start + i;
    if (motor_index >= motors_in_.size()) {
      break;
    }
    motors_in_[motor_index] = motor;
  }
}
void MotorsImuCanReceiveApi::imu_data_callback(std::shared_ptr<struct canfd_frame> recv_frame)
{
  std::unique_lock<std::shared_mutex> lock(imu_mutex_);
  std::memcpy(&imu_data_, recv_frame->data, sizeof(api_imu_data_t));
  for (size_t i(0); i < 3; ++i) {
    imu_data_.accl[i] *= GRAVITY;
  }
  return;
}
void MotorsImuCanReceiveApi::motors_status_callback(std::shared_ptr<struct canfd_frame> recv_frame)
{
  std::memcpy(&motors_status_, recv_frame->data, sizeof(api_motor_status_t));
}

}  // namespace can_device
