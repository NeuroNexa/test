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
#ifndef BATTERY_DEVICE__BATTERY_HPP_
#define BATTERY_DEVICE__BATTERY_HPP_
#include <memory>
#include <string>

#include "battery_device/battery_info.hpp"
namespace tita
{
namespace battery_device
{
// 3Mbyte: 1024bit * 1024 * 3 = 3Mbyte
#define SIZE_OF_SINGLE_CSV_FILE (3 * 1024 * 1024)

class BatteryRecord
{
public:
#define ROOT_PATH_FOR_BATTERY "/tmp/power_supply/battery_device/"
  explicit BatteryRecord(
    std::shared_ptr<tita::battery_device::BatteryHistory> battery_history_info_api_,
    bool is_task_start, int init_local_file_num = 0)
  : local_file_num_(init_local_file_num)
  {
    try {
      if (battery_history_info_api_ == nullptr) {
        throw std::runtime_error("battery_info_api_ or battery_history_info_api_ is nullptr");
      }

      this->is_running.store(true);
      battery_history_info_api = battery_history_info_api_;
      set_task_running(is_task_start);
      start_record_battery_data_thread_task(is_task_start);
    } catch (const std::runtime_error & e) {
      std::cerr << "Caught an exception: " << e.what() << std::endl;
    }
  }
  ~BatteryRecord()
  {
    if (record_T_ != nullptr) {
      record_T_->join();
    }
    record_T_ = nullptr;
    set_task_running(false);
  }
  // uuid ?
  std::string set_record_file_path();
  void write_record();
  bool start_record_battery_data_thread_task(bool is_start_task);
  void set_task_running(bool is_running) { this->is_task_running.store(is_running); }

private:
  std::atomic<bool> is_running{false};
  std::atomic<bool> is_task_running{false};
  int local_file_num_;
  std::shared_ptr<std::thread> record_T_;
  std::shared_ptr<tita::battery_device::BatteryHistory> battery_history_info_api = nullptr;
  bool checkIfExists(std::fstream & file, int poweronCount);
  void writeHeader(std::fstream & file);
  void writeData(std::fstream & file, std::shared_ptr<tita::battery_device::BatteryHistory> & info);
};
}  // namespace battery_device
}  // namespace tita

#endif  // BATTERY_DEVICE__BATTERY_HPP_
