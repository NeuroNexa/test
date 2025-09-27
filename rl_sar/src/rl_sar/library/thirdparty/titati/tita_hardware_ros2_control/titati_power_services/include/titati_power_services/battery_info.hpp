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

#pragma once
#ifndef TITATI_POWER_SERVICES__BATTERY_INFO_HPP_
#define TITATI_POWER_SERVICES__BATTERY_INFO_HPP_

#include <algorithm>
#include <atomic>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "Common.hpp"
namespace tita
{
namespace battery_device
{

#define CAN_ID_AS_BATTERY_HISTORY_INFO (0x91U)
#define CAN_ID_AS_LEFT_BATTERY_RECV \
  (0x81U)  // Look at the left and right position of the battery from the rear of the robot,
// It is distinguished from the left and right defined by the underlying software
#define CAN_ID_AS_RIGHT_BATTERY_RECV \
  (0x82U)  // Look at the left and right position of the battery from the rear of the robot,
// It is distinguished from the left and right defined by the underlying software

#define CAN_DLC_AS_BATTERY_HISTORY_INFO (56)
#define CAN_DLC_AS_LEFT_BATTERY_RECV (64)
#define CAN_DLC_AS_RIGHT_BATTERY_RECV (64)

#define CAN_ONLINE_DETECT_TIMEOUT (7)

typedef union {
  struct
  {
    uint16_t bDSG_FET : 1;   // 放电MOSFET开关状态位
    uint16_t bCHG_FET : 1;   // 充电MOSFET开关状态位
    uint16_t bPCHG_FET : 1;  // 预充电MOSFET开关状态位
    uint16_t bL0V : 1;       // 低电压禁止充电状态位
    // 以上都是AFE寄存器值
    // 以下是MCU自己管理
    uint16_t unused1 : 2;
    uint16_t bDSGING : 1;  // 放电状态位
    uint16_t bCHGING : 1;  // 充电状态位
    uint16_t bFC : 1;      // 满充状态标志位
    uint16_t bFD : 1;      // 放电截止状态标志位
    uint16_t bVDQ : 1;     // 满充容量更新有效位
    uint16_t bBLEOPEN : 1;
    uint16_t bCAL : 1;      // 是否校准
    uint16_t bAFE_ERR : 1;  // EEPROM写操作状态位
    uint16_t unused2 : 1;
  } bits;
  uint16_t val;
} UI_PACK_STATUS;

typedef union {
  struct
  {
    // 以下都是AFE寄存器值
    uint16_t bHV : 1;    // 过压保护标志位
    uint16_t bLV : 1;    // 欠压保护标志位
    uint16_t bOCD1 : 1;  // 放电过流1保护标志位
    uint16_t bOCD2 : 1;  // 放电过流2保护标志位
    uint16_t bOCC : 1;   // 充电过流保护标志位
    uint16_t bSC : 1;    // 短路保护标志位
    uint16_t bPF : 1;    // 二级保护标志位
    uint16_t unused1 : 1;
    uint16_t bUTC : 1;  // 充电低温保护标志位
    uint16_t bOTC : 1;  // 充电高温保护标志位
    uint16_t bUTD : 1;  // 放电低温保护标志位
    uint16_t bOTD : 1;  // 放电高温保护标志位
    uint16_t unused2 : 4;
  } bits;
  uint16_t val;
} UI_BAT_STATUS;

typedef struct
{
  uint32_t can_id;
  uint8_t data_length;
} BasicCANConfig_T;
typedef struct BatteryHistory_T
{
  BasicCANConfig_T can_config_t;
  uint32_t voltage;
  int16_t temperature1;
  int16_t temperature2;
  int16_t temperature3;
  uint16_t full_charge_coulomb;
  uint16_t remaining_coulomb;
  uint16_t remain_soc;
  uint16_t cycle_count;
  uint16_t charge_coulomb;
  uint16_t discharge_coulomb;
  uint32_t poweronCount;
  uint32_t poweroff_ts;
  uint32_t runtime;
  uint16_t cell_diff_max;
  uint16_t bat_stat;
  int32_t coulomb_count_mAs;
  float soc_delta;
  uint16_t pack_stat;
  uint32_t build_timestamp;
} battery_history_t;

typedef struct PacketStatus_T
{
  bool discharge_sw;
  bool charge_sw;
  bool precharge_sw;
  bool low_voltage_disable_charge;
  bool discharging;
  bool charging;
  bool full_charge;
  bool full_discharge;
  bool update_qmax;
  bool calibrate;
  bool afe_error;
} packet_status_t;

typedef struct BatteryStatus_T
{
  bool high_voltage;
  bool low_voltage;
  bool discharge_over_current1;
  bool discharge_over_current2;
  bool charge_over_current;
  bool short_circuit;
  bool abnormal_voltage;
  bool charge_low_temperature;
  bool charge_high_temperature;
  bool discharge_low_temperature;
  bool discharge_high_temperature;
} battery_status_t;

typedef struct BatteryInfo_T
{
  BasicCANConfig_T can_config_t;
  bool is_conn;
  uint32_t voltage;
  int32_t current_cadc;
  uint16_t temperature1;
  uint16_t temperature2;
  uint16_t temperature3;
  uint16_t temperatureMOS;
  uint16_t max_cap;
  uint16_t remain_cap;
  uint16_t remain_soc;
  uint16_t cycle_count;
  uint16_t pack_status;
  uint16_t bat_status;
  uint16_t pack_conf;
  uint32_t app_version;
  uint32_t build_timestamp;
  uint16_t vcell1;
  uint16_t vcell2;
  uint16_t vcell3;
  uint16_t vcell4;
  uint16_t vcell5;
  uint16_t vcell6;
  uint16_t vcell7;
  uint16_t vcell8;
  uint16_t vcell9;
  uint16_t vcell10;
  uint16_t vcell11;
  uint16_t vcell12;
  battery_status_t battery_status;
  packet_status_t packet_status;
} battery_info_t;

class BatteryInfo
{
public:
  std::atomic<bool> is_left_battery_info_rw_busy{false};
  std::atomic<bool> is_right_battery_info_rw_busy{false};
  std::atomic<bool> is_left_battery_online_detect{false};
  std::atomic<bool> is_right_battery_online_detect{false};

  BatteryInfo()
  {
    this->is_running.store(true);
    init_left_battery_info();
    init_right_battery_info();
  }
  ~BatteryInfo() = default;

  uint32_t get_left_battery_info_can_id() { return this->left_battery_info.can_config_t.can_id; }
  uint8_t get_left_battery_info_can_data_length()
  {
    return this->left_battery_info.can_config_t.data_length;
  }
  uint32_t get_right_battery_info_can_id() { return this->right_battery_info.can_config_t.can_id; }
  uint8_t get_right_battery_info_can_data_length()
  {
    return this->right_battery_info.can_config_t.data_length;
  }

  battery_status_t get_left_battery_info_battery_status()
  {
    return this->left_battery_info.battery_status;
  }

  void set_left_battery_info_battery_status(battery_status_t status)
  {
    this->left_battery_info.battery_status = status;
  }

  battery_status_t get_right_battery_info_battery_status()
  {
    return this->right_battery_info.battery_status;
  }

  void set_right_battery_info_battery_status(battery_status_t status)
  {
    this->right_battery_info.battery_status = status;
  }

  packet_status_t get_left_battery_info_packet_status()
  {
    return this->left_battery_info.packet_status;
  }

  void set_left_battery_info_packet_status(packet_status_t status)
  {
    this->left_battery_info.packet_status = status;
  }

  packet_status_t get_right_battery_info_packet_status()
  {
    return this->right_battery_info.packet_status;
  }

  void set_right_battery_info_packet_status(packet_status_t status)
  {
    this->right_battery_info.packet_status = status;
  }

private:
  std::atomic<bool> is_running{false};
  battery_info_t left_battery_info;
  battery_info_t right_battery_info;

  DEF_STRUCT_OPT(left_battery_info, bool, is_conn);
  DEF_STRUCT_OPT(left_battery_info, uint32_t, voltage);
  DEF_STRUCT_OPT(left_battery_info, int32_t, current_cadc);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, temperature1);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, temperature2);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, temperature3);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, temperatureMOS);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, max_cap);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, remain_cap);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, remain_soc);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, cycle_count);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, pack_status);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, bat_status);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, pack_conf);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, app_version);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, build_timestamp);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, vcell1);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, vcell2);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, vcell3);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, vcell4);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, vcell5);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, vcell6);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, vcell7);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, vcell8);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, vcell9);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, vcell10);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, vcell11);
  DEF_STRUCT_OPT(left_battery_info, uint16_t, vcell12);

  DEF_STRUCT_OPT(right_battery_info, bool, is_conn);
  DEF_STRUCT_OPT(right_battery_info, uint32_t, voltage);
  DEF_STRUCT_OPT(right_battery_info, int32_t, current_cadc);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, temperature1);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, temperature2);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, temperature3);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, temperatureMOS);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, max_cap);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, remain_cap);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, remain_soc);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, cycle_count);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, pack_status);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, bat_status);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, pack_conf);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, app_version);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, build_timestamp);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, vcell1);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, vcell2);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, vcell3);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, vcell4);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, vcell5);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, vcell6);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, vcell7);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, vcell8);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, vcell9);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, vcell10);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, vcell11);
  DEF_STRUCT_OPT(right_battery_info, uint16_t, vcell12);

  inline void init_left_battery_info()
  {
    this->left_battery_info.can_config_t.can_id = CAN_ID_AS_LEFT_BATTERY_RECV;
    this->left_battery_info.can_config_t.data_length = CAN_DLC_AS_LEFT_BATTERY_RECV;

    this->left_battery_info.is_conn = false;
    this->left_battery_info.voltage = 0;
    this->left_battery_info.current_cadc = 0;
    this->left_battery_info.temperature1 = 0;
    this->left_battery_info.temperature2 = 0;
    this->left_battery_info.temperature3 = 0;
    this->left_battery_info.temperatureMOS = 0;
    this->left_battery_info.max_cap = 0;
    this->left_battery_info.remain_cap = 0;
    this->left_battery_info.remain_soc = 0;
    this->left_battery_info.cycle_count = 0;
    this->left_battery_info.pack_status = 0;
    this->left_battery_info.bat_status = 0;
    this->left_battery_info.pack_conf = 0;
    this->left_battery_info.app_version = 0;
    this->left_battery_info.build_timestamp = 0;

    this->left_battery_info.vcell1 = 0;
    this->left_battery_info.vcell2 = 0;
    this->left_battery_info.vcell3 = 0;
    this->left_battery_info.vcell4 = 0;
    this->left_battery_info.vcell5 = 0;
    this->left_battery_info.vcell6 = 0;
    this->left_battery_info.vcell7 = 0;
    this->left_battery_info.vcell8 = 0;
    this->left_battery_info.vcell9 = 0;
    this->left_battery_info.vcell10 = 0;
    this->left_battery_info.vcell11 = 0;
    this->left_battery_info.vcell12 = 0;

    this->left_battery_info.battery_status.high_voltage = false;
    this->left_battery_info.battery_status.low_voltage = false;
    this->left_battery_info.battery_status.discharge_over_current1 = false;
    this->left_battery_info.battery_status.discharge_over_current2 = false;
    this->left_battery_info.battery_status.charge_over_current = false;
    this->left_battery_info.battery_status.short_circuit = false;
    this->left_battery_info.battery_status.abnormal_voltage = false;
    this->left_battery_info.battery_status.charge_low_temperature = false;
    this->left_battery_info.battery_status.charge_high_temperature = false;
    this->left_battery_info.battery_status.discharge_low_temperature = false;
    this->left_battery_info.battery_status.discharge_high_temperature = false;

    this->left_battery_info.packet_status.discharge_sw = false;
    this->left_battery_info.packet_status.charge_sw = false;
    this->left_battery_info.packet_status.precharge_sw = false;
    this->left_battery_info.packet_status.low_voltage_disable_charge = false;
    this->left_battery_info.packet_status.discharging = false;
    this->left_battery_info.packet_status.charging = false;
    this->left_battery_info.packet_status.full_charge = false;
    this->left_battery_info.packet_status.full_discharge = false;
    this->left_battery_info.packet_status.update_qmax = false;
    this->left_battery_info.packet_status.calibrate = false;
    this->left_battery_info.packet_status.afe_error = false;
  }

  inline void init_right_battery_info()
  {
    this->right_battery_info.can_config_t.can_id = CAN_ID_AS_RIGHT_BATTERY_RECV;
    this->right_battery_info.can_config_t.data_length = CAN_DLC_AS_RIGHT_BATTERY_RECV;

    this->right_battery_info.is_conn = false;
    this->right_battery_info.voltage = 0;
    this->right_battery_info.current_cadc = 0;
    this->right_battery_info.temperature1 = 0;
    this->right_battery_info.temperature2 = 0;
    this->right_battery_info.temperature3 = 0;
    this->right_battery_info.temperatureMOS = 0;
    this->right_battery_info.max_cap = 0;
    this->right_battery_info.remain_cap = 0;
    this->right_battery_info.remain_soc = 0;
    this->right_battery_info.cycle_count = 0;
    this->right_battery_info.pack_status = 0;
    this->right_battery_info.bat_status = 0;
    this->right_battery_info.pack_conf = 0;
    this->right_battery_info.app_version = 0;
    this->right_battery_info.build_timestamp = 0;

    this->right_battery_info.vcell1 = 0;
    this->right_battery_info.vcell2 = 0;
    this->right_battery_info.vcell3 = 0;
    this->right_battery_info.vcell4 = 0;
    this->right_battery_info.vcell5 = 0;
    this->right_battery_info.vcell6 = 0;
    this->right_battery_info.vcell7 = 0;
    this->right_battery_info.vcell8 = 0;
    this->right_battery_info.vcell9 = 0;
    this->right_battery_info.vcell10 = 0;
    this->right_battery_info.vcell11 = 0;
    this->right_battery_info.vcell12 = 0;

    this->right_battery_info.battery_status.high_voltage = false;
    this->right_battery_info.battery_status.low_voltage = false;
    this->right_battery_info.battery_status.discharge_over_current1 = false;
    this->right_battery_info.battery_status.discharge_over_current2 = false;
    this->right_battery_info.battery_status.charge_over_current = false;
    this->right_battery_info.battery_status.short_circuit = false;
    this->right_battery_info.battery_status.abnormal_voltage = false;
    this->right_battery_info.battery_status.charge_low_temperature = false;
    this->right_battery_info.battery_status.charge_high_temperature = false;
    this->right_battery_info.battery_status.discharge_low_temperature = false;
    this->right_battery_info.battery_status.discharge_high_temperature = false;

    this->right_battery_info.packet_status.discharge_sw = false;
    this->right_battery_info.packet_status.charge_sw = false;
    this->right_battery_info.packet_status.precharge_sw = false;
    this->right_battery_info.packet_status.low_voltage_disable_charge = false;
    this->right_battery_info.packet_status.discharging = false;
    this->right_battery_info.packet_status.charging = false;
    this->right_battery_info.packet_status.full_charge = false;
    this->right_battery_info.packet_status.full_discharge = false;
    this->right_battery_info.packet_status.update_qmax = false;
    this->right_battery_info.packet_status.calibrate = false;
    this->right_battery_info.packet_status.afe_error = false;
  }
};

class BatteryHistory
{
public:
  std::atomic<bool> is_battery_history_info_rw_busy{false};
  std::atomic<int> is_online_detect_cnt{CAN_ONLINE_DETECT_TIMEOUT};
  std::atomic<bool> is_online_detect{false};
  BatteryHistory()
  {
    this->is_running.store(true);
    init_battery_history();
  }
  ~BatteryHistory() = default;

  uint32_t get_battery_history_can_id() { return this->battery_history.can_config_t.can_id; }
  uint8_t get_battery_history_can_data_length()
  {
    return this->battery_history.can_config_t.data_length;
  }

private:
  std::atomic<bool> is_running{false};
  battery_history_t battery_history;
  DEF_STRUCT_OPT(battery_history, uint32_t, voltage);
  DEF_STRUCT_OPT(battery_history, int16_t, temperature1);
  DEF_STRUCT_OPT(battery_history, int16_t, temperature2);
  DEF_STRUCT_OPT(battery_history, int16_t, temperature3);
  DEF_STRUCT_OPT(battery_history, uint16_t, full_charge_coulomb);
  DEF_STRUCT_OPT(battery_history, uint16_t, remaining_coulomb);
  DEF_STRUCT_OPT(battery_history, uint16_t, remain_soc);
  DEF_STRUCT_OPT(battery_history, uint16_t, cycle_count);
  DEF_STRUCT_OPT(battery_history, uint16_t, charge_coulomb);
  DEF_STRUCT_OPT(battery_history, uint16_t, discharge_coulomb);
  DEF_STRUCT_OPT(battery_history, uint32_t, poweronCount);
  DEF_STRUCT_OPT(battery_history, uint32_t, poweroff_ts);
  DEF_STRUCT_OPT(battery_history, uint32_t, runtime);
  DEF_STRUCT_OPT(battery_history, uint16_t, cell_diff_max);
  DEF_STRUCT_OPT(battery_history, uint16_t, bat_stat);
  DEF_STRUCT_OPT(battery_history, int16_t, coulomb_count_mAs);
  DEF_STRUCT_OPT(battery_history, float, soc_delta);
  DEF_STRUCT_OPT(battery_history, uint16_t, pack_stat);
  DEF_STRUCT_OPT(battery_history, uint32_t, build_timestamp);

  inline void init_battery_history()
  {
    this->battery_history.can_config_t.can_id = CAN_ID_AS_BATTERY_HISTORY_INFO;
    this->battery_history.can_config_t.data_length = CAN_DLC_AS_BATTERY_HISTORY_INFO;

    this->battery_history.voltage = 0;
    this->battery_history.temperature1 = 0;
    this->battery_history.temperature2 = 0;
    this->battery_history.temperature3 = 0;
    this->battery_history.full_charge_coulomb = 0;
    this->battery_history.remaining_coulomb = 0;
    this->battery_history.remain_soc = 0;
    this->battery_history.cycle_count = 0;
    this->battery_history.charge_coulomb = 0;
    this->battery_history.discharge_coulomb = 0;
    this->battery_history.poweronCount = 0;
    this->battery_history.poweroff_ts = 0;
    this->battery_history.runtime = 0;
    this->battery_history.cell_diff_max = 0;
    this->battery_history.bat_stat = 0;
    this->battery_history.coulomb_count_mAs = 0;
    this->battery_history.soc_delta = 0;
    this->battery_history.pack_stat = 0;
    this->battery_history.build_timestamp = 0;
  }
};

}  // namespace battery_device
}  // namespace tita

#endif  // TITATI_POWER_SERVICES__BATTERY_INFO_HPP_
