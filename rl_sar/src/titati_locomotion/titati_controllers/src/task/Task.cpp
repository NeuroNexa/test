/*
 * @Author: mashicheng mashicheng@directdrivetec.com
 * @Date: 2024-01-05 13:26:44
 * @LastEditors: mashicheng mashicheng@directdrivetec.com
 * @LastEditTime: 2024-04-10 17:49:26
 * @FilePath: /tita_ros2/repos/apollo/src/libraries/locomotion/gait_control/src/task/Task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "task/Task.hpp"

void WheelQuadruped_Task::Run_Init()
{
    init_flag_ = true;
    down_flag_ = true;
    rltensorrt_flag_ = true;
    rltensorrt_srr_flag_ = true;
    rltensorrt_ho_flag_ = true;
    enable_transition_ = true;
    current_mode_ = 0;

    ahrs = ahrs_init(500);
}

void WheelQuadruped_Task::Run_Update(float quat[4], float gyro[3], float accl[3], float joint_pos[16], float joint_vel[16])
{
    float quaternion[4] = {0.,0.,0.,0.};
    float quat_tmp[4] = {0.,0.,0.,0.};
    float accl_tmp[3] = {0.,0.,0.};
    for(int i = 0; i < 3; i++)
      accl_tmp[i] = accl[i] / 9.8;
    ahrs_update(ahrs, accl_tmp, gyro, 0, 0.002);
    ahrs_get_quaternion(ahrs, quaternion);
    // static int debug_count = 0;
    // if(debug_count++ % 200 == 0) {
    //   printf("arhs:%f,%f,%f,%f\n",quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
    //   printf("org:%f,%f,%f,%f\n",quat[0], quat[1], quat[2], quat[3]);
    // }
    quat_tmp[0] = quaternion[3];
    for(int i = 0; i < 3; i++)
      quat_tmp[i + 1] = quaternion[i];
    attidata_->Update(quat_tmp, gyro, accl);
    legdata_->Update(joint_pos, joint_vel);
}

void WheelQuadruped_Task::Run_Ctrl(int mode)
{ if(mode == 0)
    current_mode_ = 0;
  else if(mode == 3)
    current_mode_ = 3;
  else if(mode == 1 && enable_transition_)
    current_mode_ = 1;
  else if(enable_transition_ && ((mode == 4 || mode == 5 || mode ==6) && (current_mode_ == 4 || current_mode_ == 5 || current_mode_ == 6 || current_mode_ == 1)))
    current_mode_ = mode;
  else if(mode != 1 && mode != 0 && mode != 3 && mode != 5 && mode != 4 && mode != 6)
    current_mode_ = 3;
 
    if (current_mode_ != 1)
      init_flag_ = true;
    if (current_mode_ != 3)
      down_flag_ = true;
    
    if (current_mode_ != 4) {
      rltensorrt_srr_flag_ = true;
      rltensorrt_srr_handle_->Set_UpdateFlag(true);
    }
    if (current_mode_ != 5) {
      rltensorrt_flag_ = true;
      rltensorrt_handle_->Set_UpdateFlag(true);
    }
    if (current_mode_ != 6) {
      rltensorrt_ho_flag_ = true;
      rltensorrt_ho_handle_->Set_UpdateFlag(true);
    }

    if (current_mode_ == 0)
    {
      enable_transition_ = true;
      for (int i = 0; i < 16; i++)
        tau_set_[i] = 0.0;
    } else if (current_mode_ == 1)
    {
      if (init_flag_)
      {
        init_flag_ = false;
        init_handle_->Package_Init();
      }
      enable_transition_ = init_handle_->enable_transition_;
      init_handle_->Package_Run();
    } else if(current_mode_ == 3) {
      if (down_flag_)
      {
        down_flag_ = false;
        puredamper_handle_->Package_Init();
      }
      enable_transition_ = puredamper_handle_->enable_transition_;
      puredamper_handle_->Package_Run();
    } else if(current_mode_ == 4) {
      enable_transition_ = true;
      if(rltensorrt_srr_flag_)
      {
        rltensorrt_srr_flag_ = false;
        rltensorrt_srr_handle_->Package_Init(legdata_, attidata_);
        rltensorrt_srr_handle_->Set_UpdateFlag(false);
      }
      float x_vel_cmd = vel_gait_cmd_->x_vel_cmd * vel_gait_cmd_->slope_x_yaw_gain[0] + vel_gait_cmd_->slope_x_yaw_bias[0];
      float yaw_turn_rate = vel_gait_cmd_->yaw_turn_rate * vel_gait_cmd_->slope_x_yaw_gain[1] + vel_gait_cmd_->slope_x_yaw_bias[1];
      rltensorrt_srr_handle_->Set_VelPitch(x_vel_cmd, yaw_turn_rate);
      rltensorrt_srr_handle_->Package_Run();
    } else if(current_mode_ == 5) {
      enable_transition_ = true;
      if(rltensorrt_flag_)
      {
        rltensorrt_flag_ = false;
        rltensorrt_handle_->Package_Init(legdata_, attidata_);
        rltensorrt_handle_->Set_UpdateFlag(false);
      }
      float x_vel_cmd = vel_gait_cmd_->x_vel_cmd * vel_gait_cmd_->stair_x_yaw_gain[0] + vel_gait_cmd_->stair_x_yaw_bias[0];
      float yaw_turn_rate = vel_gait_cmd_->yaw_turn_rate * vel_gait_cmd_->stair_x_yaw_gain[1] + vel_gait_cmd_->stair_x_yaw_bias[1];
      rltensorrt_handle_->Set_VelPitch(x_vel_cmd, yaw_turn_rate);
      rltensorrt_handle_->Package_Run();
    } else if(current_mode_ == 6) {
      enable_transition_ = true;
      if(rltensorrt_ho_flag_)
      {
        rltensorrt_ho_flag_ = false;
        rltensorrt_ho_handle_->Package_Init(legdata_, attidata_);
        rltensorrt_ho_handle_->Set_UpdateFlag(false);
      }
      float x_vel_cmd = vel_gait_cmd_->x_vel_cmd * vel_gait_cmd_->high_x_yaw_gain[0] + vel_gait_cmd_->high_x_yaw_bias[0];
      float yaw_turn_rate = vel_gait_cmd_->yaw_turn_rate * vel_gait_cmd_->high_x_yaw_gain[1] + vel_gait_cmd_->high_x_yaw_bias[1];
      rltensorrt_ho_handle_->Set_VelPitch(x_vel_cmd, yaw_turn_rate);
      rltensorrt_ho_handle_->Package_Run();
    }
    Safety_Check();
}

void WheelQuadruped_Task::Safety_Check()
{
  if(current_mode_ == 4)
  {
    // if(fabs(attidata_->rpy[0]) > 0.15 || legdata_->foot_pos_local[0][0] < -0.12 || 
    //     legdata_->foot_pos_local[1][0] < -0.12 || legdata_->foot_pos_local[2][0] > 0.12 || legdata_->foot_pos_local[3][0] > 0.12)
    //   dangerous_flag_ = true;
    // else
    dangerous_flag_ = false;
  } 
  else if(current_mode_ == 2)
  {
    if(fabs(attidata_->rpy[0]) > 0.15)
      dangerous_flag_ = true;
    else
      dangerous_flag_ = false;
  }
  else
  {
    dangerous_flag_ = false;
  }

}
