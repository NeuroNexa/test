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

#include "titati_power_services/update_battery_info.hpp"

#include "titati_power_services/protocol/can_utils.hpp"

namespace tita
{
namespace battery_device
{

void UpdateBatteryInfo::register_battery_device_can_filter()
{
  if (!this->is_running.load()) {
    throw std::runtime_error("UpdateBatteryInfo is not running");
  }
  struct can_filter battery_device_rx_filter;
  battery_device_rx_filter.can_id = CAN_ID_AS_BATTERY_INFO;
  battery_device_rx_filter.can_mask = CAN_MASK_AS_BATTERY_INFO;
  this->battery_can_receive_api->set_filter(&battery_device_rx_filter, sizeof(struct can_filter));
}

void UpdateBatteryInfo::get_battery_can_config()
{
  if (!this->is_running.load()) {
    throw std::runtime_error("UpdateBatteryInfo is not running");
  }

  if (battery_info_api->is_right_battery_info_rw_busy.load()) {
    return;
  }
  if (battery_info_api->is_left_battery_info_rw_busy.load()) {
    return;
  }

  battery_info_api->is_right_battery_info_rw_busy.store(true);
  battery_info_api->is_left_battery_info_rw_busy.store(true);

  this->left_battery_can_config.can_id = this->battery_info_api->get_left_battery_info_can_id();
  this->left_battery_can_config.data_length =
    this->battery_info_api->get_left_battery_info_can_data_length();
  this->right_battery_can_config.can_id = this->battery_info_api->get_right_battery_info_can_id();
  this->right_battery_can_config.data_length =
    this->battery_info_api->get_right_battery_info_can_data_length();
  this->battery_history_can_config.can_id =
    this->battery_history_info_api->get_battery_history_can_id();
  this->battery_history_can_config.data_length =
    this->battery_history_info_api->get_battery_history_can_data_length();

  battery_info_api->is_right_battery_info_rw_busy.store(false);
  battery_info_api->is_left_battery_info_rw_busy.store(false);
}

void UpdateBatteryInfo::update_right_battery_info(std::shared_ptr<struct canfd_frame> recv_frame)
{
  if (!this->is_running.load()) {
    throw std::runtime_error("UpdateBatteryInfo is not running");
  }

  if (this->battery_info_api->is_right_battery_info_rw_busy.load()) {
    return;
  }

  this->battery_info_api->is_right_battery_info_rw_busy.store(true);

  //! ignore timestamp
  // auto temp_voltage = (recv_frame->data[7] << 24) | (recv_frame->data[6] <<
  // 16) |
  //                     (recv_frame->data[5] << 8) | (recv_frame->data[4]);
  // auto temp_current_cadc = (recv_frame->data[11] << 24) |
  // (recv_frame->data[10] << 16) |
  //                          (recv_frame->data[9] << 8) |
  //                          (recv_frame->data[8]);

  auto raw_voltage = (recv_frame->data[7] << 24) | (recv_frame->data[6] << 16) |
                     (recv_frame->data[5] << 8) | (recv_frame->data[4]);
  auto temp_is_conn = raw_voltage & 0x80000000 ? true : false;  // check if the battery is connected
  auto temp_voltage = raw_voltage & 0x7fffffff;
  //  auto temp_current_cadc = (recv_frame->data[9] << 8) |
  //  (recv_frame->data[8]);
  auto temp_current_cadc = (recv_frame->data[11] << 24) | (recv_frame->data[10] << 16) |
                           (recv_frame->data[9] << 8) | (recv_frame->data[8]);
  auto temp_temperature1 = (recv_frame->data[13] << 8) | (recv_frame->data[12]);
  auto temp_temperature2 = (recv_frame->data[15] << 8) | (recv_frame->data[14]);
  auto temp_temperature3 = (recv_frame->data[17] << 8) | (recv_frame->data[16]);
  auto temp_temperatureMOS = (recv_frame->data[19] << 8) | (recv_frame->data[18]);
  auto temp_max_cap = (recv_frame->data[21] << 8) | (recv_frame->data[20]);
  auto temp_remain_cap = (recv_frame->data[23] << 8) | (recv_frame->data[22]);
  auto temp_remain_soc = (recv_frame->data[25] << 8) | (recv_frame->data[24]);
  auto temp_cycle_count = (recv_frame->data[27] << 8) | (recv_frame->data[26]);
  auto temp_pack_status = (recv_frame->data[29] << 8) | (recv_frame->data[28]);
  auto temp_bat_status = (recv_frame->data[31] << 8) | (recv_frame->data[30]);
  auto temp_pack_conf = (recv_frame->data[33] << 8) | (recv_frame->data[32]);
  auto temp_app_version = (recv_frame->data[37] << 24) | (recv_frame->data[36] << 16) |
                          (recv_frame->data[35] << 8) | (recv_frame->data[34]);
  auto temp_build_timestamp = (recv_frame->data[41] << 24) | (recv_frame->data[40] << 16) |
                              (recv_frame->data[39] << 8) | (recv_frame->data[38]);

  auto temp_vcell1 = (recv_frame->data[1] << 8) | (recv_frame->data[0]);
  auto temp_vcell2 = (recv_frame->data[43] << 8) | (recv_frame->data[42]);
  auto temp_vcell3 = (recv_frame->data[45] << 8) | (recv_frame->data[44]);
  auto temp_vcell4 = (recv_frame->data[47] << 8) | (recv_frame->data[46]);
  auto temp_vcell5 = (recv_frame->data[49] << 8) | (recv_frame->data[48]);
  auto temp_vcell6 = (recv_frame->data[51] << 8) | (recv_frame->data[50]);
  auto temp_vcell7 = (recv_frame->data[53] << 8) | (recv_frame->data[52]);
  auto temp_vcell8 = (recv_frame->data[55] << 8) | (recv_frame->data[54]);
  auto temp_vcell9 = (recv_frame->data[57] << 8) | (recv_frame->data[56]);
  auto temp_vcell10 = (recv_frame->data[59] << 8) | (recv_frame->data[58]);
  auto temp_vcell11 = (recv_frame->data[61] << 8) | (recv_frame->data[60]);
  auto temp_vcell12 = (recv_frame->data[63] << 8) | (recv_frame->data[62]);

  this->battery_info_api->set_right_battery_info_voltage(temp_voltage);
  this->battery_info_api->set_right_battery_info_current_cadc(temp_current_cadc);
  this->battery_info_api->set_right_battery_info_temperature1(temp_temperature1);
  this->battery_info_api->set_right_battery_info_temperature2(temp_temperature2);
  this->battery_info_api->set_right_battery_info_temperature3(temp_temperature3);
  this->battery_info_api->set_right_battery_info_temperatureMOS(temp_temperatureMOS);
  this->battery_info_api->set_right_battery_info_max_cap(temp_max_cap);
  this->battery_info_api->set_right_battery_info_remain_cap(temp_remain_cap);
  this->battery_info_api->set_right_battery_info_remain_soc(temp_remain_soc);
  this->battery_info_api->set_right_battery_info_cycle_count(temp_cycle_count);
  this->battery_info_api->set_right_battery_info_pack_status(temp_pack_status);
  this->battery_info_api->set_right_battery_info_bat_status(temp_bat_status);
  this->battery_info_api->set_right_battery_info_pack_conf(temp_pack_conf);
  this->battery_info_api->set_right_battery_info_app_version(temp_app_version);
  this->battery_info_api->set_right_battery_info_build_timestamp(temp_build_timestamp);

  this->battery_info_api->set_right_battery_info_vcell1(temp_vcell1);
  this->battery_info_api->set_right_battery_info_vcell2(temp_vcell2);
  this->battery_info_api->set_right_battery_info_vcell3(temp_vcell3);
  this->battery_info_api->set_right_battery_info_vcell4(temp_vcell4);
  this->battery_info_api->set_right_battery_info_vcell5(temp_vcell5);
  this->battery_info_api->set_right_battery_info_vcell6(temp_vcell6);
  this->battery_info_api->set_right_battery_info_vcell7(temp_vcell7);
  this->battery_info_api->set_right_battery_info_vcell8(temp_vcell8);
  this->battery_info_api->set_right_battery_info_vcell9(temp_vcell9);
  this->battery_info_api->set_right_battery_info_vcell10(temp_vcell10);
  this->battery_info_api->set_right_battery_info_vcell11(temp_vcell11);
  this->battery_info_api->set_right_battery_info_vcell12(temp_vcell12);

  // update battery status
  battery_status_t temp_battery_status =
    this->battery_info_api->get_right_battery_info_battery_status();
  UI_BAT_STATUS union_battery_status;
  union_battery_status.val = temp_bat_status;

  temp_battery_status.high_voltage = union_battery_status.bits.bHV ? true : false;
  temp_battery_status.low_voltage = union_battery_status.bits.bLV ? true : false;
  temp_battery_status.discharge_over_current1 = union_battery_status.bits.bOCD1 ? true : false;
  temp_battery_status.discharge_over_current2 = union_battery_status.bits.bOCD2 ? true : false;
  temp_battery_status.charge_over_current = union_battery_status.bits.bOCC ? true : false;
  temp_battery_status.short_circuit = union_battery_status.bits.bSC ? true : false;
  temp_battery_status.abnormal_voltage = union_battery_status.bits.bPF ? true : false;
  temp_battery_status.charge_low_temperature = union_battery_status.bits.bUTC ? true : false;
  temp_battery_status.charge_high_temperature = union_battery_status.bits.bOTC ? true : false;
  temp_battery_status.discharge_low_temperature = union_battery_status.bits.bUTD ? true : false;
  temp_battery_status.discharge_high_temperature = union_battery_status.bits.bOTD ? true : false;

  this->battery_info_api->set_right_battery_info_battery_status(temp_battery_status);

  // update packet status
  packet_status_t temp_packet_status =
    this->battery_info_api->get_right_battery_info_packet_status();
  UI_PACK_STATUS union_packet_status;
  union_packet_status.val = temp_pack_status;

  temp_packet_status.discharge_sw = union_packet_status.bits.bDSG_FET ? true : false;
  temp_packet_status.charge_sw = union_packet_status.bits.bCHG_FET ? true : false;
  temp_packet_status.low_voltage_disable_charge = union_packet_status.bits.bPCHG_FET ? true : false;
  temp_packet_status.discharging = union_packet_status.bits.bDSGING ? true : false;
  temp_packet_status.charging = union_packet_status.bits.bCHGING ? true : false;
  temp_packet_status.full_charge = union_packet_status.bits.bFC ? true : false;
  temp_packet_status.full_discharge = union_packet_status.bits.bFD ? true : false;
  temp_packet_status.update_qmax = union_packet_status.bits.bVDQ ? true : false;
  temp_packet_status.calibrate = union_packet_status.bits.bCAL ? true : false;
  temp_packet_status.afe_error = union_packet_status.bits.bAFE_ERR ? true : false;

  this->battery_info_api->set_right_battery_info_packet_status(temp_packet_status);

  // update online status
  this->battery_info_api->is_right_battery_online_detect.store(temp_is_conn);

  this->battery_info_api->is_right_battery_info_rw_busy.store(false);
}

void UpdateBatteryInfo::update_left_battery_info(std::shared_ptr<struct canfd_frame> recv_frame)
{
  if (!this->is_running.load()) {
    throw std::runtime_error("UpdateBatteryInfo is not running");
  }

  if (this->battery_info_api->is_left_battery_info_rw_busy.load()) {
    return;
  }

  this->battery_info_api->is_left_battery_info_rw_busy.store(true);

  //! ignore timestamp
  // auto temp_voltage = (recv_frame->data[7] << 24) | (recv_frame->data[6] <<
  // 16) |
  //                    (recv_frame->data[5] << 8) | (recv_frame->data[4]);
  // auto temp_current_cadc = (recv_frame->data[11] << 24) |
  // (recv_frame->data[10] << 16) |
  //                         (recv_frame->data[9] << 8) | (recv_frame->data[8]);

  auto raw_voltage = (recv_frame->data[7] << 24) | (recv_frame->data[6] << 16) |
                     (recv_frame->data[5] << 8) | (recv_frame->data[4]);
  auto temp_is_conn = raw_voltage & 0x80000000 ? true : false;  // check if the battery is connected
  auto temp_voltage = raw_voltage & 0x7fffffff;
  //  auto temp_current_cadc = (recv_frame->data[9] << 8) |
  //  (recv_frame->data[8]);
  auto temp_current_cadc = (recv_frame->data[11] << 24) | (recv_frame->data[10] << 16) |
                           (recv_frame->data[9] << 8) | (recv_frame->data[8]);
  auto temp_temperature1 = (recv_frame->data[13] << 8) | (recv_frame->data[12]);
  auto temp_temperature2 = (recv_frame->data[15] << 8) | (recv_frame->data[14]);
  auto temp_temperature3 = (recv_frame->data[17] << 8) | (recv_frame->data[16]);
  auto temp_temperatureMOS = (recv_frame->data[19] << 8) | (recv_frame->data[18]);
  auto temp_max_cap = (recv_frame->data[21] << 8) | (recv_frame->data[20]);
  auto temp_remain_cap = (recv_frame->data[23] << 8) | (recv_frame->data[22]);
  auto temp_remain_soc = (recv_frame->data[25] << 8) | (recv_frame->data[24]);
  auto temp_cycle_count = (recv_frame->data[27] << 8) | (recv_frame->data[26]);
  auto temp_pack_status = (recv_frame->data[29] << 8) | (recv_frame->data[28]);
  auto temp_bat_status = (recv_frame->data[31] << 8) | (recv_frame->data[30]);
  auto temp_pack_conf = (recv_frame->data[33] << 8) | (recv_frame->data[32]);
  auto temp_app_version = (recv_frame->data[37] << 24) | (recv_frame->data[36] << 16) |
                          (recv_frame->data[35] << 8) | (recv_frame->data[34]);
  auto temp_build_timestamp = (recv_frame->data[41] << 24) | (recv_frame->data[40] << 16) |
                              (recv_frame->data[39] << 8) | (recv_frame->data[38]);

  auto temp_vcell1 = (recv_frame->data[1] << 8) | (recv_frame->data[0]);
  auto temp_vcell2 = (recv_frame->data[43] << 8) | (recv_frame->data[42]);
  auto temp_vcell3 = (recv_frame->data[45] << 8) | (recv_frame->data[44]);
  auto temp_vcell4 = (recv_frame->data[47] << 8) | (recv_frame->data[46]);
  auto temp_vcell5 = (recv_frame->data[49] << 8) | (recv_frame->data[48]);
  auto temp_vcell6 = (recv_frame->data[51] << 8) | (recv_frame->data[50]);
  auto temp_vcell7 = (recv_frame->data[53] << 8) | (recv_frame->data[52]);
  auto temp_vcell8 = (recv_frame->data[55] << 8) | (recv_frame->data[54]);
  auto temp_vcell9 = (recv_frame->data[57] << 8) | (recv_frame->data[56]);
  auto temp_vcell10 = (recv_frame->data[59] << 8) | (recv_frame->data[58]);
  auto temp_vcell11 = (recv_frame->data[61] << 8) | (recv_frame->data[60]);
  auto temp_vcell12 = (recv_frame->data[63] << 8) | (recv_frame->data[62]);

  this->battery_info_api->set_left_battery_info_voltage(temp_voltage);
  this->battery_info_api->set_left_battery_info_current_cadc(temp_current_cadc);
  this->battery_info_api->set_left_battery_info_temperature1(temp_temperature1);
  this->battery_info_api->set_left_battery_info_temperature2(temp_temperature2);
  this->battery_info_api->set_left_battery_info_temperature3(temp_temperature3);
  this->battery_info_api->set_left_battery_info_temperatureMOS(temp_temperatureMOS);
  this->battery_info_api->set_left_battery_info_max_cap(temp_max_cap);
  this->battery_info_api->set_left_battery_info_remain_cap(temp_remain_cap);
  this->battery_info_api->set_left_battery_info_remain_soc(temp_remain_soc);
  this->battery_info_api->set_left_battery_info_cycle_count(temp_cycle_count);
  this->battery_info_api->set_left_battery_info_pack_status(temp_pack_status);
  this->battery_info_api->set_left_battery_info_bat_status(temp_bat_status);
  this->battery_info_api->set_left_battery_info_pack_conf(temp_pack_conf);
  this->battery_info_api->set_left_battery_info_app_version(temp_app_version);
  this->battery_info_api->set_left_battery_info_build_timestamp(temp_build_timestamp);

  this->battery_info_api->set_left_battery_info_vcell1(temp_vcell1);
  this->battery_info_api->set_left_battery_info_vcell2(temp_vcell2);
  this->battery_info_api->set_left_battery_info_vcell3(temp_vcell3);
  this->battery_info_api->set_left_battery_info_vcell4(temp_vcell4);
  this->battery_info_api->set_left_battery_info_vcell5(temp_vcell5);
  this->battery_info_api->set_left_battery_info_vcell6(temp_vcell6);
  this->battery_info_api->set_left_battery_info_vcell7(temp_vcell7);
  this->battery_info_api->set_left_battery_info_vcell8(temp_vcell8);
  this->battery_info_api->set_left_battery_info_vcell9(temp_vcell9);
  this->battery_info_api->set_left_battery_info_vcell10(temp_vcell10);
  this->battery_info_api->set_left_battery_info_vcell11(temp_vcell11);
  this->battery_info_api->set_left_battery_info_vcell12(temp_vcell12);

  // update battery status
  battery_status_t temp_battery_status =
    this->battery_info_api->get_left_battery_info_battery_status();
  UI_BAT_STATUS union_battery_status;
  union_battery_status.val = temp_bat_status;

  temp_battery_status.high_voltage = union_battery_status.bits.bHV ? true : false;
  temp_battery_status.low_voltage = union_battery_status.bits.bLV ? true : false;
  temp_battery_status.discharge_over_current1 = union_battery_status.bits.bOCD1 ? true : false;
  temp_battery_status.discharge_over_current2 = union_battery_status.bits.bOCD2 ? true : false;
  temp_battery_status.charge_over_current = union_battery_status.bits.bOCC ? true : false;
  temp_battery_status.short_circuit = union_battery_status.bits.bSC ? true : false;
  temp_battery_status.abnormal_voltage = union_battery_status.bits.bPF ? true : false;
  temp_battery_status.charge_low_temperature = union_battery_status.bits.bUTC ? true : false;
  temp_battery_status.charge_high_temperature = union_battery_status.bits.bOTC ? true : false;
  temp_battery_status.discharge_low_temperature = union_battery_status.bits.bUTD ? true : false;
  temp_battery_status.discharge_high_temperature = union_battery_status.bits.bOTD ? true : false;

  this->battery_info_api->set_left_battery_info_battery_status(temp_battery_status);

  // update packet status
  packet_status_t temp_packet_status =
    this->battery_info_api->get_left_battery_info_packet_status();
  UI_PACK_STATUS union_packet_status;
  union_packet_status.val = temp_pack_status;

  temp_packet_status.discharge_sw = union_packet_status.bits.bDSG_FET ? true : false;
  temp_packet_status.charge_sw = union_packet_status.bits.bCHG_FET ? true : false;
  temp_packet_status.low_voltage_disable_charge = union_packet_status.bits.bPCHG_FET ? true : false;
  temp_packet_status.discharging = union_packet_status.bits.bDSGING ? true : false;
  temp_packet_status.charging = union_packet_status.bits.bCHGING ? true : false;
  temp_packet_status.full_charge = union_packet_status.bits.bFC ? true : false;
  temp_packet_status.full_discharge = union_packet_status.bits.bFD ? true : false;
  temp_packet_status.update_qmax = union_packet_status.bits.bVDQ ? true : false;
  temp_packet_status.calibrate = union_packet_status.bits.bCAL ? true : false;
  temp_packet_status.afe_error = union_packet_status.bits.bAFE_ERR ? true : false;

  this->battery_info_api->set_left_battery_info_packet_status(temp_packet_status);

  // update online status
  this->battery_info_api->is_left_battery_online_detect.store(temp_is_conn);

  this->battery_info_api->is_left_battery_info_rw_busy.store(false);
}

void UpdateBatteryInfo::update_battery_history_info(std::shared_ptr<struct canfd_frame> recv_frame)
{
  if (!this->is_running.load()) {
    throw std::runtime_error("UpdateBatteryInfo is not running");
  }

  if (this->battery_history_info_api->is_battery_history_info_rw_busy.load()) {
    return;
  }

  this->battery_history_info_api->is_battery_history_info_rw_busy.store(true);

  // ignore timestamp
  auto temp_voltage = (recv_frame->data[4] << 24) | (recv_frame->data[5] << 16) |
                      (recv_frame->data[6] << 8) | (recv_frame->data[7]);
  auto temp_temperature1 = (recv_frame->data[8] << 8) | (recv_frame->data[9]);
  auto temp_temperature2 = (recv_frame->data[10] << 8) | (recv_frame->data[11]);
  auto temp_temperature3 = (recv_frame->data[12] << 8) | (recv_frame->data[13]);
  auto temp_full_charge_coulomb = (recv_frame->data[14] << 8) | (recv_frame->data[15]);
  auto temp_remaining_coulomb = (recv_frame->data[16] << 8) | (recv_frame->data[17]);
  auto temp_remain_soc = (recv_frame->data[18] << 8) | (recv_frame->data[19]);
  auto temp_cycle_count = (recv_frame->data[20] << 8) | (recv_frame->data[21]);
  auto temp_charge_coulomb = (recv_frame->data[22] << 8) | (recv_frame->data[23]);
  auto temp_discharge_coulomb = (recv_frame->data[24] << 8) | (recv_frame->data[25]);
  auto temp_poweronCount = (recv_frame->data[26] << 24) | (recv_frame->data[27] << 16) |
                           (recv_frame->data[28] << 8) | (recv_frame->data[29]);
  auto temp_poweroff_ts = (recv_frame->data[30] << 24) | (recv_frame->data[31] << 16) |
                          (recv_frame->data[32] << 8) | (recv_frame->data[33]);
  auto temp_runtime = (recv_frame->data[34] << 24) | (recv_frame->data[35] << 16) |
                      (recv_frame->data[36]) << 8 | (recv_frame->data[37]);
  auto temp_cell_diff_max = (recv_frame->data[38] << 8) | (recv_frame->data[39]);
  auto temp_bat_stat = (recv_frame->data[40] << 8) | (recv_frame->data[41]);
  auto temp_coulomb_count_mAs = (recv_frame->data[42] << 24) | (recv_frame->data[43] << 16) |
                                (recv_frame->data[44] << 8) | (recv_frame->data[45]);
  auto temp_soc_delta = (recv_frame->data[46] << 24) | (recv_frame->data[47] << 16) |
                        (recv_frame->data[48] << 8) | (recv_frame->data[49]);
  auto temp_pack_stat = (recv_frame->data[50] << 8) | (recv_frame->data[51]);
  auto temp_build_timestamp = (recv_frame->data[52] << 24) | (recv_frame->data[53] << 16) |
                              (recv_frame->data[54] << 8) | (recv_frame->data[55]);

  this->battery_history_info_api->set_battery_history_voltage(temp_voltage);
  this->battery_history_info_api->set_battery_history_temperature1(temp_temperature1);
  this->battery_history_info_api->set_battery_history_temperature2(temp_temperature2);
  this->battery_history_info_api->set_battery_history_temperature3(temp_temperature3);
  this->battery_history_info_api->set_battery_history_full_charge_coulomb(temp_full_charge_coulomb);
  this->battery_history_info_api->set_battery_history_remaining_coulomb(temp_remaining_coulomb);
  this->battery_history_info_api->set_battery_history_remain_soc(temp_remain_soc);
  this->battery_history_info_api->set_battery_history_cycle_count(temp_cycle_count);
  this->battery_history_info_api->set_battery_history_charge_coulomb(temp_charge_coulomb);
  this->battery_history_info_api->set_battery_history_discharge_coulomb(temp_discharge_coulomb);
  this->battery_history_info_api->set_battery_history_poweronCount(temp_poweronCount);
  this->battery_history_info_api->set_battery_history_poweroff_ts(temp_poweroff_ts);
  this->battery_history_info_api->set_battery_history_runtime(temp_runtime);
  this->battery_history_info_api->set_battery_history_cell_diff_max(temp_cell_diff_max);
  this->battery_history_info_api->set_battery_history_bat_stat(temp_bat_stat);
  this->battery_history_info_api->set_battery_history_coulomb_count_mAs(temp_coulomb_count_mAs);
  this->battery_history_info_api->set_battery_history_soc_delta(temp_soc_delta);
  this->battery_history_info_api->set_battery_history_pack_stat(temp_pack_stat);
  this->battery_history_info_api->set_battery_history_build_timestamp(temp_build_timestamp);

  this->battery_history_info_api->is_battery_history_info_rw_busy.store(false);
}

void UpdateBatteryInfo::get_battery_can_data(std::shared_ptr<struct canfd_frame> recv_frame)
{
  if (!this->is_running.load()) {
    throw std::runtime_error("UpdateBatteryInfo is not running");
  }

  if (recv_frame->can_id == this->right_battery_can_config.can_id) {
    update_right_battery_info(recv_frame);
  } else if (recv_frame->can_id == this->left_battery_can_config.can_id) {
    update_left_battery_info(recv_frame);
  } else if (recv_frame->can_id == this->battery_history_can_config.can_id) {
    update_battery_history_info(recv_frame);
  } else {
    return;
  }
}
}  // namespace battery_device
}  // namespace tita
