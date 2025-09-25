/*
 * @Author: mashicheng mashicheng@directdrivetec.com
 * @Date: 2024-03-26 19:05:34
 * @LastEditors: mashicheng mashicheng@directdrivetec.com
 * @LastEditTime: 2024-06-28 11:56:44
 * @FilePath: /tita_ros2/repos/apollo/src/libraries/locomotion/gait_control/src/task/PureDamper.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "task/PureDamper.hpp"

void PureDamper_Handle::Package_Init()
{
    count_ = 0;
    for(int i = 0; i < 4; i++)
    {
      pos_init_[3*i] = legdata_->q_abad[i];
      pos_init_[3*i + 1] = legdata_->q_hip[i];
      pos_init_[3*i + 2] = legdata_->q_knee[i];
      pos_wheel_init[i] = legdata_->q_wheel_abs[i];
    }
    enable_transition_ = false;
}

void PureDamper_Handle::Package_Run()
{
    count_++;
    if(count_ < 3500) {
      for (int i = 0; i < 12; i++)
      {
        pos_des_[i] = pos_init_[i] + static_cast<float>(count_) * (pos_final_pd_[i] - pos_init_[i]) / 3500.0;
      }
      for (int i = 0; i < 4; i++)
      {
        torque_set_[4 * i] = 65 * (pos_des_[3 * i] - legdata_->q_abad[i]) + 4 * (0 - legdata_->qd_abad[i]);
        torque_set_[4 * i + 1] = 65 * (pos_des_[3 * i + 1] - legdata_->q_hip[i]) + 4 * (0 - legdata_->qd_hip[i]);
        torque_set_[4 * i + 2] = 65 * (pos_des_[3 * i + 2] - legdata_->q_knee[i]) + 4 * (0 - legdata_->qd_knee[i]);
        torque_set_[4 * i + 3] = 5 * (pos_wheel_init[i] - legdata_->q_wheel_abs[i]) + 0.5 * (0 - legdata_->qd_wheel[i]);
      }
    } else if (count_ < 5000) {
      for (int i = 0; i < 4; i++)
      {
        torque_set_[4 * i] = 65 * (static_cast<float>(5000 - count_) / 1500.0) * (pos_final_pd_[3 * i] - legdata_->q_abad[i]) + 4 * (static_cast<float>(5000 - count_) / 1500.0) * (0 - legdata_->qd_abad[i]);
        torque_set_[4 * i + 1] = 65 * (static_cast<float>(5000 - count_) / 1500.0) * (pos_final_pd_[3 * i + 1] - legdata_->q_hip[i]) + 4 * (static_cast<float>(5000 - count_) / 1500.0) * (0 - legdata_->qd_hip[i]);
        torque_set_[4 * i + 2] = 65 * (static_cast<float>(5000 - count_) / 1500.0) * (pos_final_pd_[3 * i + 2] - legdata_->q_knee[i]) + 4 * (static_cast<float>(5000 - count_) / 1500.0) * (0 - legdata_->qd_knee[i]);
        torque_set_[4 * i + 3] = 0.0;
      }

    } else {
      enable_transition_ = true;
      for (int i = 0; i < 4; i++)
      {
        torque_set_[4 * i] = 0.0;
        torque_set_[4 * i + 1] = 0.0;;
        torque_set_[4 * i + 2] = 0.0;
        torque_set_[4 * i + 3] = 0.0;
      }
    }
}