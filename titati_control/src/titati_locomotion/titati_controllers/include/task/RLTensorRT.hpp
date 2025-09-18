#ifndef RL_TENSORRT_HPP
#define RL_TENSORRT_HPP

#include "common/LegData.h"
#include "common/AttitudeData.h"
#include "common/tensor_cuda_test.hpp"

#include <thread>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

struct ModelParams
{
    float damping;
    float stiffness;
    float action_scale;
    float hip_scale_reduction;
    float num_of_dofs;
    float lin_vel_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float clip_obs;
    float clip_actions;
    float torque_limits[16];
    float d_gains[16];
    float p_gains[16];
    float commands_scale[3];
    float default_dof_pos[16];
};

struct Observations
{
    float lin_vel[3];           
    float ang_vel[3];  
    float gravity_vec[3];
    float forward_vec[3];       
    float commands[3];        
    float base_quat[4];   
    float dof_pos[16];           
    float dof_vel[16];           
    float actions[16];
};

class RLTensorRT_Handle
{
public:
    RLTensorRT_Handle(float* torque_set):torque_set_(torque_set), 
    input_0(new float[57]),
    input_1(new float[570]),
    output(new float[16]),
    output_last(new float[16]),
    input_1_temp(new float[513])
    {
        cuda_test_ = std::make_shared<CudaTest>("/home/robot/model_gn_m0.engine");
        std::cout << input_1.get()[569] << std::endl;
    }
    ~RLTensorRT_Handle()
    {
    }

    void Package_Init(LegData* legdata, AttitudeData* attidata);
    void Package_Run();
    void Set_VelPitch(float x_vel_cmd, float pitch_cmd)
    {
        if(x_vel_cmd > 0.96)
          x_vel_cmd_ = 0.96;
        else if(x_vel_cmd < -0.96)
          x_vel_cmd_ = -0.96;
        else
          x_vel_cmd_ = x_vel_cmd;
        
        if(pitch_cmd > 0.6)
          pitch_cmd_ = 0.6;
        else if(pitch_cmd < -0.6)
          pitch_cmd_ = -0.6;
        else
          pitch_cmd_ = pitch_cmd;
    }
    void Set_UpdateFlag(bool update_flag) {stop_update_ = update_flag;} 
private:
    float* torque_set_;
    float wheel_init_pos_abs_[4];

    // Quadruped<float> quadruped_;
    // FloatingBaseModel<float> model_;
    // WBC_Ctrl<float> * wbc_ctrl_;
    // LocomotionCtrlData<float> * wbc_data_;

    // Vec3<float> ini_body_pos_;
    // Vec3<float> ini_body_ori_rpy_;
    // float body_weight_;

    float x_vel_cmd_;
    float pitch_cmd_;
private:
    ModelParams params_;
    Observations obs_;

    void Forward();
    void Run_Forward();

    std::shared_ptr<CudaTest> cuda_test_;

    std::thread forward_thread;
    bool threadRunning;
    float desired_pos[16] = {0, 0.75, -1.5, 0, 0, 0.75, -1.5, 0, 0, 0.75, -1.5, 0, 0, 0.75, -1.5, 0};
    AttitudeData* attitude_ = nullptr;
    LegData* legdata_ = nullptr;

    std::shared_ptr<float[]> input_0;
    std::shared_ptr<float[]> input_1;
    std::shared_ptr<float[]> output;

    std::shared_ptr<float[]> output_last;
    std::shared_ptr<float[]> input_1_temp;

    int history_length = 10;

    void GetObs();
    Vec3<float> a_l;

    float action[16];

    bool stop_update_ = false;
    bool thread_first_ = true;

};

#endif