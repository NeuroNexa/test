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

#include "titati_power_services/power_management.hpp"

namespace tita
{
namespace battery_device
{

void PowerManagement::register_power_management_can_data()
{
  if (!this->is_running.load()) {
    throw std::runtime_error("PowerManagement is not running");
  }

  struct can_filter power_management_rx_filter;
  power_management_rx_filter.can_id = CAN_ID_AS_POWER_MANAGEMENT;
  power_management_rx_filter.can_mask = CAN_MASK_AS_POWER_MANAGEMENT;
  this->power_management_can_controller_api->set_filter(
    &power_management_rx_filter, sizeof(struct can_filter));
}

void PowerManagement::get_power_management_can_data(std::shared_ptr<struct canfd_frame> recv_frame)
{
  if (!this->is_running.load()) {
    throw std::runtime_error("PowerManagement is not running");
  }

  if (recv_frame->can_id == this->power_management_info_api->get_power_state_info_get_can_id()) {
    this->update_power_management_state_can_data(recv_frame);
  } else if (
    recv_frame->can_id == this->power_management_info_api->get_power_selftest_get_can_id()) {
    this->update_power_management_self_test_can_data(recv_frame);
  } else {
    return;
  }
}

void PowerManagement::update_power_management_state_can_data(
  std::shared_ptr<struct canfd_frame> recv_frame)
{
  if (!this->is_running.load()) {
    throw std::runtime_error("PowerManagement is not running");
  }

  if (this->power_management_info_api->is_power_state_info_get_rw_busy.load()) {
    return;
  }

  this->power_management_info_api->is_power_state_info_get_rw_busy.store(true);

  //! ignore timestamp
  auto temp_power_5v = recv_frame->data[4];
  auto temp_power_12v = recv_frame->data[5];
  auto temp_power_24v = recv_frame->data[6];
  auto temp_power_motor_48v = recv_frame->data[7];
  auto temp_power_extern_48v = recv_frame->data[8];
  auto temp_power_5v_operation_delay_ms = (recv_frame->data[9] << 8) | (recv_frame->data[10]);
  auto temp_power_12v_operation_delay_ms = (recv_frame->data[11] << 8) | (recv_frame->data[12]);
  auto temp_power_24v_operation_delay_ms = (recv_frame->data[13] << 8) | (recv_frame->data[14]);
  auto temp_power_motor_48v_operation_delay_ms =
    (recv_frame->data[15] << 8) | (recv_frame->data[16]);
  auto temp_power_extern_48v_operation_delay_ms =
    (recv_frame->data[17] << 8) | (recv_frame->data[18]);
  auto temp_fixed_0x55 = recv_frame->data[19];

  this->power_management_info_api->set_power_state_info_get_power_5v(temp_power_5v);
  this->power_management_info_api->set_power_state_info_get_power_12v(temp_power_12v);
  this->power_management_info_api->set_power_state_info_get_power_24v(temp_power_24v);
  this->power_management_info_api->set_power_state_info_get_power_motor_48v(temp_power_motor_48v);
  this->power_management_info_api->set_power_state_info_get_power_extern_48v(temp_power_extern_48v);
  this->power_management_info_api->set_power_state_info_get_power_5v_operation_delay_ms(
    temp_power_5v_operation_delay_ms);
  this->power_management_info_api->set_power_state_info_get_power_12v_operation_delay_ms(
    temp_power_12v_operation_delay_ms);
  this->power_management_info_api->set_power_state_info_get_power_24v_operation_delay_ms(
    temp_power_24v_operation_delay_ms);
  this->power_management_info_api->set_power_state_info_get_power_motor_48v_operation_delay_ms(
    temp_power_motor_48v_operation_delay_ms);
  this->power_management_info_api->set_power_state_info_get_power_extern_48v_operation_delay_ms(
    temp_power_extern_48v_operation_delay_ms);
  this->power_management_info_api->set_power_state_info_get_fixed_0x55(temp_fixed_0x55);

  this->power_management_info_api->is_power_state_info_get_rw_busy.store(false);
}

void PowerManagement::update_power_management_self_test_can_data(
  std::shared_ptr<struct canfd_frame> recv_frame)
{
  if (!this->is_running.load()) {
    throw std::runtime_error("PowerManagement is not running");
  }

  if (this->power_management_info_api->is_power_selftest_get_rw_busy.load()) {
    return;
  }

  this->power_management_info_api->is_power_selftest_get_rw_busy.store(true);

  //! ignore timestamp
  // little edian
  uint8_t temp_rst_1 = recv_frame->data[4];
  uint8_t temp_rst_2 = recv_frame->data[5];
  uint8_t temp_rst_3 = recv_frame->data[6];
  uint8_t temp_rst_4 = recv_frame->data[7];
  uint8_t temp_rst_5 = recv_frame->data[8];
  uint8_t temp_rst_6 = recv_frame->data[9];
  uint8_t temp_rst_7 = recv_frame->data[10];
  uint8_t temp_rst_8 = recv_frame->data[11];
  auto temp_app_version_1 = recv_frame->data[12];
  auto temp_app_version_2 = recv_frame->data[13];
  auto temp_app_version_3 = recv_frame->data[14];
  auto temp_app_version_4 = recv_frame->data[15];
  auto temp_build_timestamp_1 = recv_frame->data[16];
  auto temp_build_timestamp_2 = recv_frame->data[17];
  auto temp_build_timestamp_3 = recv_frame->data[18];
  auto temp_build_timestamp_4 = recv_frame->data[19];
  auto temp_uuid_0_1 = recv_frame->data[20];
  auto temp_uuid_0_2 = recv_frame->data[21];
  auto temp_uuid_0_3 = recv_frame->data[22];
  auto temp_uuid_0_4 = recv_frame->data[23];
  auto temp_uuid_1_1 = recv_frame->data[24];
  auto temp_uuid_1_2 = recv_frame->data[25];
  auto temp_uuid_1_3 = recv_frame->data[26];
  auto temp_uuid_1_4 = recv_frame->data[27];
  auto temp_uuid_2_1 = recv_frame->data[28];
  auto temp_uuid_2_2 = recv_frame->data[29];
  auto temp_uuid_2_3 = recv_frame->data[30];
  auto temp_uuid_2_4 = recv_frame->data[31];

  uint64_t temp_rst =
    (static_cast<uint64_t>(temp_rst_1) << 56) | (static_cast<uint64_t>(temp_rst_2) << 48) |
    (static_cast<uint64_t>(temp_rst_3) << 40) | (static_cast<uint64_t>(temp_rst_4) << 32) |
    (static_cast<uint64_t>(temp_rst_5) << 24) | (static_cast<uint64_t>(temp_rst_6) << 16) |
    (static_cast<uint64_t>(temp_rst_7) << 8) | (static_cast<uint64_t>(temp_rst_8));
  auto app_version = (temp_app_version_1 << 24) | (temp_app_version_2 << 16) |
                     (temp_app_version_3 << 8) | (temp_app_version_4);

  auto build_timestamp = (temp_build_timestamp_1 << 24) | (temp_build_timestamp_2 << 16) |
                         (temp_build_timestamp_3 << 8) | (temp_build_timestamp_4);

  auto uuid_0 =
    (temp_uuid_0_1 << 24) | (temp_uuid_0_2 << 16) | (temp_uuid_0_3 << 8) | (temp_uuid_0_4);

  auto uuid_1 =
    (temp_uuid_1_1 << 24) | (temp_uuid_1_2 << 16) | (temp_uuid_1_3 << 8) | (temp_uuid_1_4);

  auto uuid_2 =
    (temp_uuid_2_1 << 24) | (temp_uuid_2_2 << 16) | (temp_uuid_2_3 << 8) | (temp_uuid_2_4);

  this->power_management_info_api->set_power_selftest_get_rst(temp_rst);
  this->power_management_info_api->set_power_selftest_get_app_version(app_version);
  this->power_management_info_api->set_power_selftest_get_build_timestamp(build_timestamp);
  this->power_management_info_api->set_power_selftest_get_uuid_0(uuid_0);
  this->power_management_info_api->set_power_selftest_get_uuid_1(uuid_1);
  this->power_management_info_api->set_power_selftest_get_uuid_2(uuid_2);

  this->power_management_info_api->is_power_selftest_get_rw_busy.store(false);
}

void PowerManagement::send_power_management_state_can_data_impl(
  tita::battery_device::power_state_info_t power_state_info)
{
  struct canfd_frame tx_frame;

  tx_frame.can_id = CAN_ID_AS_POWER_STATE_INFO_SET;
  tx_frame.len = CAN_DLC_AS_POWER_STATE_INFO_SET;

  //! ignore the timestamp
  tx_frame.data[4] = power_state_info.power_5v;
  tx_frame.data[5] = power_state_info.power_12v;
  tx_frame.data[6] = power_state_info.power_24v;
  tx_frame.data[7] = power_state_info.power_motor_48v;
  tx_frame.data[8] = power_state_info.power_extern_48v;
  tx_frame.data[9] = power_state_info.power_5v_operation_delay_ms & 0xFF;
  tx_frame.data[10] = power_state_info.power_5v_operation_delay_ms >> 8;
  tx_frame.data[11] = power_state_info.power_12v_operation_delay_ms & 0xFF;
  tx_frame.data[12] = power_state_info.power_12v_operation_delay_ms >> 8;
  tx_frame.data[13] = power_state_info.power_24v_operation_delay_ms & 0xFF;
  tx_frame.data[14] = power_state_info.power_24v_operation_delay_ms >> 8;
  tx_frame.data[15] = power_state_info.power_motor_48v_operation_delay_ms & 0xFF;
  tx_frame.data[16] = power_state_info.power_motor_48v_operation_delay_ms >> 8;
  tx_frame.data[17] = power_state_info.power_extern_48v_operation_delay_ms & 0xFF;
  tx_frame.data[18] = power_state_info.power_extern_48v_operation_delay_ms >> 8;
  tx_frame.data[19] = power_state_info.fixed_0x55;

  power_management_can_controller_api->send_can_message(tx_frame);
}

void PowerManagement::send_power_management_self_test_can_data_impl(
  tita::battery_device::power_selftest_t power_selftest)
{
  struct canfd_frame tx_frame;

  tx_frame.can_id = CAN_ID_AS_POWER_SELFTEST;
  tx_frame.len = CAN_DLC_AS_POWER_SELFTEST;

  //! ignore the timestamp
  tx_frame.data[4] = (power_selftest.rst >> 56) & 0xFFU;
  tx_frame.data[5] = (power_selftest.rst >> 48) & 0xFFU;
  tx_frame.data[6] = (power_selftest.rst >> 40) & 0xFFU;
  tx_frame.data[7] = (power_selftest.rst >> 32) & 0xFFU;
  tx_frame.data[8] = (power_selftest.rst >> 24) & 0xFFU;
  tx_frame.data[9] = (power_selftest.rst >> 16) & 0xFFU;
  tx_frame.data[10] = (power_selftest.rst >> 8) & 0xFFU;
  tx_frame.data[11] = (power_selftest.rst) & 0xFFU;
  tx_frame.data[12] = (power_selftest.app_version >> 24) & 0xFFU;
  tx_frame.data[13] = (power_selftest.app_version >> 16) & 0xFFU;
  tx_frame.data[14] = (power_selftest.app_version >> 8) & 0xFFU;
  tx_frame.data[15] = (power_selftest.app_version) & 0xFFU;
  tx_frame.data[16] = (power_selftest.build_timestamp >> 24) & 0xFFU;
  tx_frame.data[17] = (power_selftest.build_timestamp >> 16) & 0xFFU;
  tx_frame.data[18] = (power_selftest.build_timestamp >> 8) & 0xFFU;
  tx_frame.data[19] = (power_selftest.build_timestamp) & 0xFFU;
  tx_frame.data[20] = (power_selftest.uuid_0 >> 24) & 0xFFU;
  tx_frame.data[21] = (power_selftest.uuid_0 >> 16) & 0xFFU;
  tx_frame.data[22] = (power_selftest.uuid_0 >> 8) & 0xFFU;
  tx_frame.data[23] = (power_selftest.uuid_0) & 0xFFU;
  tx_frame.data[24] = (power_selftest.uuid_1 >> 24) & 0xFFU;
  tx_frame.data[25] = (power_selftest.uuid_1 >> 16) & 0xFFU;
  tx_frame.data[26] = (power_selftest.uuid_1 >> 8) & 0xFFU;
  tx_frame.data[27] = (power_selftest.uuid_1) & 0xFFU;
  tx_frame.data[28] = (power_selftest.uuid_2 >> 24) & 0xFFU;
  tx_frame.data[29] = (power_selftest.uuid_2 >> 16) & 0xFFU;
  tx_frame.data[30] = (power_selftest.uuid_2 >> 8) & 0xFFU;
  tx_frame.data[31] = (power_selftest.uuid_2) & 0xFFU;

  power_management_can_controller_api->send_can_message(tx_frame);
}

void PowerManagement::send_power_management_heart_beat_can_data_impl(
  tita::battery_device::power_heart_beat_t power_heart_beat)
{
  struct canfd_frame tx_frame;

  tx_frame.can_id = CAN_ID_AS_POWER_HEARTBEAT;
  tx_frame.len = CAN_DLC_AS_POWER_HEARTEAT;

  //! ignore the timestamp
  tx_frame.data[4] = (power_heart_beat.cur_mode >> 24) & 0xFFU;
  tx_frame.data[5] = (power_heart_beat.cur_mode >> 16) & 0xFFU;
  tx_frame.data[6] = (power_heart_beat.cur_mode >> 8) & 0xFFU;
  tx_frame.data[7] = (power_heart_beat.cur_mode) & 0xFFU;
  tx_frame.data[8] = power_heart_beat.get_right_battery_history_info;
  tx_frame.data[9] = power_heart_beat.get_left_battery_history_info;

  power_management_can_controller_api->send_can_message(tx_frame);
}

}  // namespace battery_device
}  // namespace tita
