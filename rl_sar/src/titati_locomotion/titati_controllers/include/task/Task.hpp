/*
 * @Author: mashicheng mashicheng@directdrivetec.com
 * @Date: 2024-03-26 19:05:34
 * @LastEditors: mashicheng mashicheng@directdrivetec.com
 * @LastEditTime: 2024-04-11 15:54:22
 * @FilePath: /tita_ros2/repos/apollo/src/libraries/locomotion/gait_control/include/task/Task.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef WQ_TASK_HPP
#define WQ_TASK_HPP

#include "task/Init.hpp"
#include "common/LegData.h"
#include "common/AttitudeData.h"
#include "task/PureDamper.hpp"
#include "ahrs.h"
// #include "task/RL.hpp"
#include "task/RLTensorRT.hpp"
#include "task/RLTensorRT_SlopeRecoveryRotation.hpp"
#include "task/RLTensorRT_HighObstacle.hpp"

  struct VelGaitCmd
  {
    float yaw_turn_rate;
    float x_vel_cmd;

    float slope_x_yaw_gain[2];
    float slope_x_yaw_bias[2];

    float stair_x_yaw_gain[2];
    float stair_x_yaw_bias[2];

    float high_x_yaw_gain[2];
    float high_x_yaw_bias[2];

    VelGaitCmd() : yaw_turn_rate(0.),
                   x_vel_cmd(0.){
      for(int i = 0; i < 2; i++) {
        slope_x_yaw_gain[i] = 1.0;
        slope_x_yaw_bias[i] = 0.0;
        stair_x_yaw_gain[i] = 1.0;
        stair_x_yaw_bias[i] = 0.0;
        high_x_yaw_gain[i] = 1.0;
        high_x_yaw_bias[i] = 0.0;
      }

    }
  };
class WheelQuadruped_Task {
public:
  WheelQuadruped_Task(float* tau_set, std::shared_ptr<VelGaitCmd> cmd):
  tau_set_(tau_set),vel_gait_cmd_(cmd)
  {
    legdata_ = new LegData();
    init_handle_ = new Init_Handle(tau_set_, legdata_);
    puredamper_handle_ = new PureDamper_Handle(tau_set_, legdata_);
    attidata_ = new AttitudeData();
    rltensorrt_handle_ = new RLTensorRT_Handle(tau_set_);
    rltensorrt_srr_handle_ = new RLTensorRT_SRR_Handle(tau_set_);
    rltensorrt_ho_handle_ = new RLTensorRT_HO_Handle(tau_set_);
  }
  ~WheelQuadruped_Task()
  {
    delete legdata_;
    delete init_handle_;
    delete puredamper_handle_;
    delete attidata_ ;
    delete rltensorrt_handle_;
    delete rltensorrt_srr_handle_;
    delete rltensorrt_ho_handle_;
  }

  void Run_Init();
  void Run_Update(float quat[4], float gyro[3], float accl[3], float joint_pos[16], float joint_vel[16]);
  void Run_Ctrl(int mode);

  int Get_Current_Mode() {return current_mode_;}
  bool Get_DangerousPos_Flag() {return dangerous_flag_;};
private:
  LegData* legdata_;
  float* tau_set_;
  Init_Handle* init_handle_;
  bool init_flag_ = true;
  PureDamper_Handle* puredamper_handle_;
  bool down_flag_ = true;
  RLTensorRT_Handle* rltensorrt_handle_;
  bool rltensorrt_flag_ = true;
  RLTensorRT_SRR_Handle* rltensorrt_srr_handle_;
  bool rltensorrt_srr_flag_ = true;
  RLTensorRT_HO_Handle* rltensorrt_ho_handle_;
  bool rltensorrt_ho_flag_ = true;
  AttitudeData* attidata_;

  int current_mode_ = 0;
  bool enable_transition_ = true;

  bool dangerous_flag_ = false;
  void Safety_Check();

  void* ahrs;
  std::shared_ptr<VelGaitCmd> vel_gait_cmd_;
};

#endif