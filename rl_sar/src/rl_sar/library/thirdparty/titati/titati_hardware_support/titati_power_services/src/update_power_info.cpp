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

#include "titati_power_services/update_power_info.hpp"

#include "titati_power_services/protocol/can_utils.hpp"

namespace tita
{
namespace battery_device
{

void UpdatePowerInfo::register_power_device_can_filter()
{
  if (!this->is_running.load()) {
    throw std::runtime_error("UpdatePowerInfo is not running");
  }
  struct can_filter power_device_rx_filter;
  power_device_rx_filter.can_id = CAN_ID_AS_POWER_INFO;
  power_device_rx_filter.can_mask = CAN_MASK_AS_POWER_INFO;
  this->power_can_receive_api->set_filter(&power_device_rx_filter, sizeof(struct can_filter));
}

void UpdatePowerInfo::get_power_domain_can_config()
{
  if (!this->is_running.load()) {
    throw std::runtime_error("UpdatePowerInfo is not running");
  }

  if (this->power_management_info_api->is_power_domain_info_get_rw_busy.load()) {
    return;
  }

  this->power_management_info_api->is_power_domain_info_get_rw_busy.store(true);

  this->power_domain_info_can_config.can_id =
    this->power_management_info_api->get_power_domain_info_get_can_id();
  this->power_domain_info_can_config.data_length =
    this->power_management_info_api->get_power_domain_info_get_can_data_length();

  this->power_management_info_api->is_power_domain_info_get_rw_busy.store(false);
}

void UpdatePowerInfo::update_power_domain_info(std::shared_ptr<struct canfd_frame> recv_frame)
{
  if (!this->is_running.load()) {
    throw std::runtime_error("UpdatePowerInfo is not running");
  }

  if (this->power_management_info_api->is_power_domain_info_get_rw_busy.load()) {
    return;
  }

  this->power_management_info_api->is_power_domain_info_get_rw_busy.store(true);

  auto temp_ibus_vol_max = (recv_frame->data[0] << 8) | (recv_frame->data[1]);
  auto temp_ibus_vol_min = (recv_frame->data[2] << 8) | (recv_frame->data[3]);
  auto temp_ibus_vol = (recv_frame->data[4] << 8) | (recv_frame->data[5]);
  auto temp_xt30_vol_max = (recv_frame->data[6] << 8) | (recv_frame->data[7]);
  auto temp_xt30_vol_min = (recv_frame->data[8] << 8) | (recv_frame->data[9]);
  auto temp_xt30_vol = (recv_frame->data[10] << 8) | (recv_frame->data[11]);
  auto temp_vbus_vol = (recv_frame->data[12] << 8) | (recv_frame->data[13]);

  this->power_management_info_api->set_power_domain_info_get_ibus_vol_max(temp_ibus_vol_max);
  this->power_management_info_api->set_power_domain_info_get_ibus_vol_min(temp_ibus_vol_min);
  this->power_management_info_api->set_power_domain_info_get_ibus_vol(temp_ibus_vol);
  this->power_management_info_api->set_power_domain_info_get_xt30_vol_max(temp_xt30_vol_max);
  this->power_management_info_api->set_power_domain_info_get_xt30_vol_min(temp_xt30_vol_min);
  this->power_management_info_api->set_power_domain_info_get_xt30_vol(temp_xt30_vol);
  this->power_management_info_api->set_power_domain_info_get_vbus_vol(temp_vbus_vol);

  this->power_management_info_api->is_power_domain_info_get_rw_busy.store(false);
}

void UpdatePowerInfo::get_power_can_data(std::shared_ptr<struct canfd_frame> recv_frame)
{
  if (!this->is_running.load()) {
    throw std::runtime_error("UpdatePowerInfo is not running");
  }

  if (recv_frame->can_id == this->power_domain_info_can_config.can_id) {
    update_power_domain_info(recv_frame);
  } else {
    return;
  }
}
}  // namespace battery_device
}  // namespace tita
