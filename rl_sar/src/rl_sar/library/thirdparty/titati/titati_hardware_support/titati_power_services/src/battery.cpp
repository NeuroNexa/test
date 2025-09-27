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

#include "titati_power_services/battery.hpp"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "titati_power_services/debug_log.h"

namespace tita
{
namespace battery_device
{

std::string BatteryRecord::set_record_file_path()
{
  if (!this->is_running.load()) {
    throw std::runtime_error("BatteryRecord is not running");
  }

  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::stringstream record_data;
  record_data << std::put_time(std::localtime(&now_time), "%Y__%m_%d__%H_%M");

  std::string record_file_path = ROOT_PATH_FOR_BATTERY;
  record_file_path = record_file_path + record_data.str();

  // maybe use try{}catch(){}
  if (!std::filesystem::exists(ROOT_PATH_FOR_BATTERY)) {
    std::filesystem::create_directories(ROOT_PATH_FOR_BATTERY);
  }

  if (!std::filesystem::exists(record_file_path)) {
    std::filesystem::create_directory(record_file_path);
  }

  return record_file_path;
}

void BatteryRecord::write_record()
{
  if (!this->is_running.load()) {
    throw std::runtime_error("BatteryRecord is not running");
  }

  LOG_INFO("[Success] Task Thread BatteryRecord::write_record() is called\r\n");

  std::string dir_path = set_record_file_path();

  LOG_INFO("[Debug] Thread Task Start\r\n");

  while (this->is_task_running.load()) {
    std::shared_ptr<tita::battery_device::BatteryHistory> info = this->battery_history_info_api;
    if (info->is_battery_history_info_rw_busy.load()) {
      continue;
    }

    info->is_battery_history_info_rw_busy.store(true);

    std::string file_path =
      dir_path + "/battery_history_info_" + std::to_string(local_file_num_) + ".csv";
    std::fstream file(file_path, std::ios::in | std::ios::out | std::ios::app);
    if (file.is_open()) {
      bool exists = checkIfExists(file, info->get_battery_history_poweronCount());

      if (exists) {
        writeData(file, info);
        // make sure csv file will not be too large
        if (std::filesystem::file_size(file_path) >= SIZE_OF_SINGLE_CSV_FILE) {
          this->local_file_num_ += 1;
          LOG_INFO(
            "[Debug] battery.cpp:write_record() [local_file_num] is [%d]! \r\n",
            this->local_file_num_);
        }
      } else {
        LOG_INFO("[Failed] battery.cpp:write_record() if( !exists ) return false! \r\n");
      }
    } else {
      LOG_INFO("[Failed] battery.cpp:write_record() if( file.is_open() ) return false! \r\n");
    }

    info->is_battery_history_info_rw_busy.store(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100 hz
  }

  LOG_INFO("[Exit] Task Thread BatteryRecord::write_record() is exited\r\n");
}

bool BatteryRecord::checkIfExists(std::fstream & file, int poweronCount)
{
  if (!this->is_running.load()) {
    throw std::runtime_error("BatteryRecord is not running");
  }

  std::string line;
  bool is_first = false;
  while (std::getline(file, line)) {
    if (is_first == false) {
      is_first = true;
      continue;
    }
    std::stringstream ss(line);
    std::string temp;
    std::vector<std::string> row;

    while (std::getline(ss, temp, ',')) {
      row.push_back(temp);
    }

    // if file impl is empty or wrong
    if ((row.empty()) || (std::stoi(row[0]) != poweronCount)) {
      file.clear();
      is_first = false;
    }
  }
  file.clear();
  if (is_first == false) {
    writeHeader(file);
  }
  return true;
}

void BatteryRecord::writeHeader(std::fstream & file)
{
  file.seekp(0, std::ios::end);
  file
    << "poweronCount,voltage,temperature1,temperature2,temperature3,full_charge_coulomb,remain_"
       "coulomb,remain_soc,cycle_count,charge_coulomb,discharge_coulomb,poweroff_ts,runtime,cell_"
       "diff_max,bat_stat,coulomb_count_mAs,soc_delta,pack_stat,build_timestamp,reccord_date\n";
}

void BatteryRecord::writeData(
  std::fstream & file, std::shared_ptr<tita::battery_device::BatteryHistory> & info)
{
  if (!this->is_running.load()) {
    throw std::runtime_error("BatteryRecord is not running");
  }

  auto now = std::chrono::system_clock::now();
  std::time_t time1 = std::chrono::system_clock::to_time_t(now);
  std::time_t time2 = static_cast<std::time_t>(info->get_battery_history_build_timestamp());
  std::time_t time3 = static_cast<std::time_t>(info->get_battery_history_poweroff_ts());

  std::stringstream record_data, build_timestamp, poweroff_ts;
  record_data << std::put_time(std::localtime(&time1), "%Y-%m-%d %H:%M:%S");
  build_timestamp << std::put_time(std::localtime(&time2), "%Y-%m-%d %H:%M:%S");
  poweroff_ts << std::put_time(std::localtime(&time3), "%Y-%m-%d %H:%M:%S");

  file.seekp(0, std::ios::end);
  file << info->get_battery_history_poweronCount() << "," << info->get_battery_history_voltage()
       << "," << info->get_battery_history_temperature1() << ","
       << info->get_battery_history_temperature2() << ","
       << info->get_battery_history_temperature3() << ","
       << info->get_battery_history_full_charge_coulomb() << ","
       << info->get_battery_history_remaining_coulomb() << ","
       << info->get_battery_history_remain_soc() << "," << info->get_battery_history_cycle_count()
       << "," << info->get_battery_history_charge_coulomb() << ","
       << info->get_battery_history_discharge_coulomb() << "," << poweroff_ts.str() << ","
       << info->get_battery_history_runtime() << "," << info->get_battery_history_cell_diff_max()
       << "," << info->get_battery_history_bat_stat() << ","
       << info->get_battery_history_coulomb_count_mAs() << ","
       << info->get_battery_history_soc_delta() << "," << info->get_battery_history_pack_stat()
       << "," << build_timestamp.str() << "," << record_data.str() << "\n";
}

bool BatteryRecord::start_record_battery_data_thread_task(bool is_start_task)
{
  if (!this->is_running.load()) {
    throw std::runtime_error("BatteryRecord is not running");
  }

  LOG_INFO("[Debug] Task Thread try to create!\r\n");
  try {
    if (is_start_task) {
      record_T_ = std::make_shared<std::thread>(&BatteryRecord::write_record, this);
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    LOG_ERR("[Failed] Task Thread created failed!\r\n");
    return false;
  }

  return true;
}

}  // namespace battery_device
}  // namespace tita
