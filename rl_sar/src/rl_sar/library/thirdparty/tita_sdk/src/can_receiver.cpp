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

#include "tita_robot/can_receiver.hpp"

#include <vector>

namespace can_device
{

void MotorsImuCanReceiveApi::register_motors_device_can_filter()
{
  std::vector<struct can_filter> filters(board_count_);
  for (size_t board = 0; board < board_count_; ++board) {
    filters[board].can_id = CAN_ID_MOTOR_IN + board * can_board_stride_;
    filters[board].can_mask = CAN_MASK_AS_MOTORS_INFO;
  }
  this->motors_can_receive_api->set_filter(filters.data(), filters.size() * sizeof(struct can_filter));
}

void MotorsImuCanReceiveApi::board_can_data_callback(std::shared_ptr<struct canfd_frame> recv_frame)
{
  if (recv_frame->can_id >= CAN_ID_MOTOR_IN &&
      recv_frame->can_id < CAN_ID_MOTOR_IN + board_count_ * can_board_stride_) {
    auto raw_offset = recv_frame->can_id - CAN_ID_MOTOR_IN;
    size_t board = raw_offset / can_board_stride_;
    size_t frame_index = raw_offset % can_board_stride_;
    if (board >= board_count_ || frame_index >= leg_num_per_board_) {
      return;
    }
    motors_data_callback(recv_frame, board * leg_num_per_board_ + frame_index);
  } else if (recv_frame->can_id == CAN_ID_IMU1) {
    imu_data_callback(recv_frame);
  } else {
    return;
  }
}
void MotorsImuCanReceiveApi::motors_data_callback(std::shared_ptr<struct canfd_frame> recv_frame, size_t frame_index)
{
  std::unique_lock<std::shared_mutex> lock(motors_in_mutex_);
  for (size_t i = 0; i < leg_dof_; i++) {
    api_motor_in_t motor;
    std::memcpy(&motor, recv_frame->data + 16 * i, sizeof(api_motor_in_t));
    motors_in_[i + leg_dof_ * frame_index] = motor;
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
