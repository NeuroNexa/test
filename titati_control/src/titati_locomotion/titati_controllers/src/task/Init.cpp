#include "task/Init.hpp"

void Init_Handle::Package_Init()
{
    count_ = 0;
    for(int i = 0; i < 4; i++)
    {
        pos_init_[3*i] = legdata_->q_abad[i];
        pos_init_[3*i + 1] = legdata_->q_hip[i];
        pos_init_[3*i + 2] = legdata_->q_knee[i];
        pos_wheel_init[i] = legdata_->q_wheel[i];
    }

    kneel_flag_ = false;
    for(int i = 0; i < 4; i++)
    {
      if(legdata_->q_abad[i] > -1.42 || legdata_->q_abad[i] < -1.72  || fabs(legdata_->foot_pos_local[i][0]) > 0.1) {
        kneel_flag_ = true;
        break;
      }
    }
    
    enable_transition_ = false;
}

void Init_Handle::Package_Run()
{
    count_++;
    if(kneel_flag_)
      kneelBeforeStand();
    else
      standOnly();

}

void Init_Handle::standOnly()
{
  if (count_ >= 2000)
  {
    count_ = 2000;
    enable_transition_ = true;
  }
  for (int i = 0; i < 12; i++)
  {
    pos_des_[i] = pos_init_[i] + static_cast<float>(count_) * (pos_final_stand_[i] - pos_init_[i]) / 2000.0;
  }
  for (int i = 0; i < 4; i++)
  {
    torque_set_[4 * i] = 110 * (pos_des_[3 * i] - legdata_->q_abad[i]) + 2.5 * (0 - legdata_->qd_abad[i]);
    torque_set_[4 * i + 1] = 110 * (pos_des_[3 * i + 1] - legdata_->q_hip[i]) + 2.5 * (0 - legdata_->qd_hip[i]);
    torque_set_[4 * i + 2] = 110 * (pos_des_[3 * i + 2] - legdata_->q_knee[i]) + 2.5 * (0 - legdata_->qd_knee[i]);
    torque_set_[4 * i + 3] = 0.7 * (0 - legdata_->qd_wheel[i]);
  }
}

void Init_Handle::kneelBeforeStand()
{
  if (count_ >= 6000)
  {
    count_ = 6000;
    enable_transition_ = true;
  }
  if(count_ <= 2000) {
    for (int i = 0; i < 12; i++)
    {
      pos_des_[i] = pos_init_[i] + static_cast<float>(count_) * (pos_final_kneel_stage1_[i] - pos_init_[i]) / 2000.0;
    } 
    for(int i = 0; i < 4; i++)
      torque_set_[4 * i + 3] = 0.0;
  } else if(count_ <= 4000) {
    for (int i = 0; i < 12; i++)
    {
      pos_des_[i] = pos_final_kneel_stage1_[i] + static_cast<float>(count_ - 2000) * (pos_final_kneel_stage2_[i] - pos_final_kneel_stage1_[i]) / 2000.0;
    } 
    for(int i = 0; i < 4; i++)
      torque_set_[4 * i + 3] = 0.0;
  } else {
    if(count_ == 4001)
    {
      for(int i = 0; i < 4; i++)
      {
        pos_wheel_init[i] = legdata_->q_wheel[i];
      }  
    }
    for (int i = 0; i < 12; i++)
    {
      pos_des_[i] = pos_final_kneel_stage2_[i] + static_cast<float>(count_ - 4000) * (pos_final_stand_[i] - pos_final_kneel_stage2_[i]) / 2000.0;
    }
    for(int i = 0; i < 4; i++)
      torque_set_[4 * i + 3] = 0.7 * (0 - legdata_->qd_wheel[i]);
  }

  for (int i = 0; i < 4; i++)
  {
    torque_set_[4 * i] = 110 * (pos_des_[3 * i] - legdata_->q_abad[i]) + 2.5 * (0 - legdata_->qd_abad[i]);
    torque_set_[4 * i + 1] = 110 * (pos_des_[3 * i + 1] - legdata_->q_hip[i]) + 2.5 * (0 - legdata_->qd_hip[i]);
    torque_set_[4 * i + 2] = 110 * (pos_des_[3 * i + 2] - legdata_->q_knee[i]) + 2.5 * (0 - legdata_->qd_knee[i]);
    // torque_set_[4 * i + 3] = 7 * (pos_wheel_init[i] - legdata_->q_wheel[i]) + 0.4 * (0 - legdata_->qd_wheel[i]);
  }
}

