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

#ifndef TITATI_POWER_SERVICES__POWER_MANAGEMENT_INFO_HPP_
#define TITATI_POWER_SERVICES__POWER_MANAGEMENT_INFO_HPP_

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

#define CAN_ID_AS_POWER_STATE_INFO_GET (0x85U)
#define CAN_ID_AS_POWER_STATE_INFO_SET (0x86U)
#define CAN_ID_AS_POWER_SELFTEST (0x87U)
#define CAN_ID_AS_POWER_DOMAIN_INFO (0x88U)
#define CAN_ID_AS_POWER_HEARTBEAT (0x90U)

#define CAN_DLC_AS_POWER_STATE_INFO_GET (20)
#define CAN_DLC_AS_POWER_STATE_INFO_SET (20)
#define CAN_DLC_AS_POWER_SELFTEST (32)
#define CAN_DLC_AS_POWER_DOMAIN_INFO (32)
#define CAN_DLC_AS_POWER_HEARTEAT (10)

const int POWER_CONTROL_OFF = -1;
const int POWER_CONTORL_NO_CHANGE = 0;
const int POWER_CONTORL_ON = 1;

typedef struct
{
  uint32_t can_id;
  uint8_t data_length;
} BasicPowerCANConfig_T;

typedef struct PowerStateInfo_T
{
  BasicPowerCANConfig_T can_config_t;
  int8_t power_5v;
  int8_t power_12v;
  int8_t power_24v;
  int8_t power_motor_48v;
  int8_t power_extern_48v;

  uint16_t power_5v_operation_delay_ms;
  uint16_t power_12v_operation_delay_ms;
  uint16_t power_24v_operation_delay_ms;
  uint16_t power_motor_48v_operation_delay_ms;
  uint16_t power_extern_48v_operation_delay_ms;

  uint8_t fixed_0x55;
} power_state_info_t;

typedef struct PowerSelfTest_T
{
  BasicPowerCANConfig_T can_config_t;
  uint64_t rst;
  uint32_t app_version;
  uint32_t build_timestamp;
  uint32_t uuid_0;
  uint32_t uuid_1;
  uint32_t uuid_2;
} power_selftest_t;

typedef struct PowerHeartbeat_T
{
  BasicPowerCANConfig_T can_config_t;
  uint32_t cur_mode;
  uint8_t get_right_battery_history_info;
  uint8_t get_left_battery_history_info;
} power_heart_beat_t;

typedef struct PowerDomainInfo_T
{
  BasicPowerCANConfig_T can_config_t;
  int16_t ibus_vol_max;
  int16_t ibus_vol_min;
  int16_t ibus_vol;
  int16_t xt30_vol_max;
  int16_t xt30_vol_min;
  int16_t xt30_vol;
  int16_t vbus_vol;
} power_domain_info_t;

class PowerManagementInfo
{
public:
  std::atomic<bool> is_power_state_info_get_rw_busy{false};
  std::atomic<bool> is_power_selftest_get_rw_busy{false};
  std::atomic<bool> is_power_domain_info_get_rw_busy{false};
  PowerManagementInfo()
  {
    this->is_running.store(true);
    init_power_state_info_get();
    init_power_selftest_get();
    init_power_domain_info_get();
  };
  ~PowerManagementInfo() = default;

  uint32_t get_power_state_info_get_can_id()
  {
    return this->power_state_info_get.can_config_t.can_id;
  }
  uint8_t get_power_state_info_get_can_data_length()
  {
    return this->power_state_info_get.can_config_t.data_length;
  }

  uint32_t get_power_selftest_get_can_id() { return this->power_selftest_get.can_config_t.can_id; }
  uint8_t get_power_selftest_get_can_data_length()
  {
    return this->power_selftest_get.can_config_t.data_length;
  }

  uint32_t get_power_domain_info_get_can_id()
  {
    return this->power_domain_info_get.can_config_t.can_id;
  }
  uint8_t get_power_domain_info_get_can_data_length()
  {
    return this->power_domain_info_get.can_config_t.data_length;
  }

  power_domain_info_t get_power_domain_info() { return this->power_domain_info_get; }

  void set_power_domain_info(power_domain_info_t info) { this->power_domain_info_get = info; }

private:
  std::atomic<bool> is_running{false};
  power_state_info_t power_state_info_get;
  power_selftest_t power_selftest_get;
  power_domain_info_t power_domain_info_get;

  DEF_STRUCT_OPT(power_state_info_get, int8_t, power_motor_48v);
  DEF_STRUCT_OPT(power_state_info_get, int8_t, power_extern_48v);
  DEF_STRUCT_OPT(power_state_info_get, int8_t, power_24v);
  DEF_STRUCT_OPT(power_state_info_get, int8_t, power_12v);
  DEF_STRUCT_OPT(power_state_info_get, int8_t, power_5v);
  DEF_STRUCT_OPT(power_state_info_get, uint16_t, power_5v_operation_delay_ms);
  DEF_STRUCT_OPT(power_state_info_get, uint16_t, power_12v_operation_delay_ms);
  DEF_STRUCT_OPT(power_state_info_get, uint16_t, power_24v_operation_delay_ms);
  DEF_STRUCT_OPT(power_state_info_get, uint16_t, power_motor_48v_operation_delay_ms);
  DEF_STRUCT_OPT(power_state_info_get, uint16_t, power_extern_48v_operation_delay_ms);
  DEF_STRUCT_OPT(power_state_info_get, uint8_t, fixed_0x55);

  DEF_STRUCT_OPT(power_selftest_get, uint64_t, rst);
  DEF_STRUCT_OPT(power_selftest_get, uint32_t, app_version);
  DEF_STRUCT_OPT(power_selftest_get, uint32_t, build_timestamp);
  DEF_STRUCT_OPT(power_selftest_get, uint32_t, uuid_0);
  DEF_STRUCT_OPT(power_selftest_get, uint32_t, uuid_1);
  DEF_STRUCT_OPT(power_selftest_get, uint32_t, uuid_2);

  DEF_STRUCT_OPT(power_domain_info_get, int16_t, ibus_vol_max);
  DEF_STRUCT_OPT(power_domain_info_get, int16_t, ibus_vol_min);
  DEF_STRUCT_OPT(power_domain_info_get, int16_t, ibus_vol);
  DEF_STRUCT_OPT(power_domain_info_get, int16_t, xt30_vol_max);
  DEF_STRUCT_OPT(power_domain_info_get, int16_t, xt30_vol_min);
  DEF_STRUCT_OPT(power_domain_info_get, int16_t, xt30_vol);
  DEF_STRUCT_OPT(power_domain_info_get, int16_t, vbus_vol);

  inline void init_power_state_info_get()
  {
    this->power_state_info_get.can_config_t.can_id = CAN_ID_AS_POWER_STATE_INFO_GET;
    this->power_state_info_get.can_config_t.data_length = CAN_DLC_AS_POWER_STATE_INFO_GET;

    this->power_state_info_get.power_motor_48v = tita::battery_device::POWER_CONTORL_NO_CHANGE;
    this->power_state_info_get.power_extern_48v = tita::battery_device::POWER_CONTORL_NO_CHANGE;
    this->power_state_info_get.power_24v = tita::battery_device::POWER_CONTORL_NO_CHANGE;
    this->power_state_info_get.power_12v = tita::battery_device::POWER_CONTORL_NO_CHANGE;
    this->power_state_info_get.power_5v = tita::battery_device::POWER_CONTORL_NO_CHANGE;
    this->power_state_info_get.power_5v_operation_delay_ms = 1000;
    this->power_state_info_get.power_12v_operation_delay_ms = 1000;
    this->power_state_info_get.power_24v_operation_delay_ms = 1000;
    this->power_state_info_get.power_motor_48v_operation_delay_ms = 1000;
    this->power_state_info_get.power_extern_48v_operation_delay_ms = 1000;
  }

  inline void init_power_selftest_get()
  {
    this->power_selftest_get.can_config_t.can_id = CAN_ID_AS_POWER_SELFTEST;
    this->power_selftest_get.can_config_t.data_length = CAN_DLC_AS_POWER_SELFTEST;

    this->power_selftest_get.rst = 0;
    this->power_selftest_get.app_version = 0;
    this->power_selftest_get.build_timestamp = 0;
    this->power_selftest_get.uuid_0 = 0;
    this->power_selftest_get.uuid_1 = 0;
    this->power_selftest_get.uuid_2 = 0;
  }

  inline void init_power_domain_info_get()
  {
    this->power_domain_info_get.can_config_t.can_id = CAN_ID_AS_POWER_DOMAIN_INFO;
    this->power_domain_info_get.can_config_t.data_length = CAN_DLC_AS_POWER_DOMAIN_INFO;

    this->power_domain_info_get.ibus_vol_max = 0;
    this->power_domain_info_get.ibus_vol_min = 0;
    this->power_domain_info_get.ibus_vol = 0;
    this->power_domain_info_get.xt30_vol_max = 0;
    this->power_domain_info_get.xt30_vol_min = 0;
    this->power_domain_info_get.xt30_vol = 0;
    this->power_domain_info_get.vbus_vol = 0;
  }
};

}  // namespace battery_device
}  // namespace tita

#endif  // TITATI_POWER_SERVICES__POWER_MANAGEMENT_INFO_HPP_
