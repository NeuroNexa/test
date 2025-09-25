#include "task/RLTensorRT_HighObstacle.hpp"
#include "common/timeMarker.h"

void RLTensorRT_HO_Handle::Forward()
{
    GetObs();
    cuda_test_->do_inference(input_0.get(), input_1.get(), output.get());

    for (int i = 0; i < 513; i++)
        input_1_temp.get()[i] = input_1.get()[i + 57];

    for (int i = 0; i < 513; i++)
        input_1.get()[i] = input_1_temp.get()[i];

    for (int i = 0; i < 57; i++)
        input_1.get()[i + 513] = input_0.get()[i];

    for (int i = 0; i < 16; i++)
        output_last.get()[i] = output.get()[i];

}

void RLTensorRT_HO_Handle::GetObs()
{
    std::vector<float> obs_tmp;
    // compute gravity
    Mat3<float> _B2G_RotMat = attitude_->rot_body.transpose();
    Mat3<float> _G2B_RotMat = attitude_->rot_body;

    Vec3<float> angvel = a_l;
    a_l = 0.96*attitude_->omega_body + 0.04*a_l;
    Vec3<float> projected_gravity = _G2B_RotMat * Vec3<float>(0.0, 0.0, -1.0);
    Vec3<float> projected_forward = _G2B_RotMat * Vec3<float>(1.0, 0.0, 0.0);
    // gravity
    // _gxFilter->addValue(angvel(0,0));
    // _gyFilter->addValue(angvel(1,0));
    // _gzFilter->addValue(angvel(2,0));
    //
    obs_tmp.push_back(angvel(0)*0.25);
    obs_tmp.push_back(angvel(1)*0.25);
    obs_tmp.push_back(angvel(2)*0.25);

    for (int i = 0; i < 3; ++i)
    {
        obs_tmp.push_back(projected_gravity(i));
    }

    // cmd
    float rx = pitch_cmd_;//rx * (1 - smooth) + (std::fabs(_lowState->userValue.rx) < dead_zone ? 0.0 : _lowState->userValue.rx) * smooth;
    float ly = x_vel_cmd_;//ly * (1 - smooth) + (std::fabs(_lowState->userValue.ly) < dead_zone ? 0.0 : _lowState->userValue.ly) * smooth;

    float max = 1.0;
    float min = -1.0;

    float rot = rx*3.14;
    float vel = ly*2;

    double heading = 0.;
    double angle = (double)rot - heading;
    angle = fmod(angle,2.0*M_PI);
    if(angle > M_PI)
    {
        angle = angle - 2.0*M_PI;
    }
    angle = angle*0.5;
    angle = std::max(std::min((float)angle, max), min);
    angle = angle * 0.25;

    obs_tmp.push_back(vel);
    obs_tmp.push_back(0.0);
    obs_tmp.push_back(angle);

    // pos
    for (int i = 0; i < 16; ++i)
    {
        float pos = (this->obs_.dof_pos[i]  - this->params_.default_dof_pos[i]) * params_.dof_pos_scale;
        obs_tmp.push_back(pos);
    }
    // vel
    for (int i = 0; i < 16; ++i)
    {
        float vel = this->obs_.dof_vel[i] * params_.dof_vel_scale;
        obs_tmp.push_back(vel);
    }

    // last action
    //float index[12] = {3,4,5,0,1,2,9,10,11,6,7,8};
    for (int i = 0; i < 16; ++i)
    {
        obs_tmp.push_back(output_last.get()[i]);
    }

    for(int i = 0; i < 57; i++)
    {
        input_0.get()[i] = obs_tmp[i];
    }

}

void RLTensorRT_HO_Handle::Run_Forward()
{
    while(threadRunning)
    {
        long long _start_time = getSystemTime();
        // Vec3<float> lin_vel_tmp = posvelest_->_stateEstimatorData.result->vWorld;
        // Vec3<float> ang_vel_tmp = attitude_->omega_world;
        // Quat<float> quat_tmp = attitude_->quat;
        // obs_.lin_vel = torch::tensor({{(double)lin_vel_tmp[0], (double)lin_vel_tmp[1], (double)lin_vel_tmp[2]}});
        // obs_.ang_vel = torch::tensor({{(double)ang_vel_tmp[0], (double)ang_vel_tmp[1], (double)ang_vel_tmp[2]}});
        // obs_.commands = torch::tensor({{0.0,0.0,0.0}});
        // obs_.base_quat = torch::tensor({{(double)quat_tmp[1],(double)quat_tmp[2],(double)quat_tmp[3], (double)quat_tmp[0]}});

        // obs_.dof_pos = torch::tensor({{(double)legdata_->q_abad[1] + 1.57, (double)legdata_->q_hip[1], (double)legdata_->q_knee[1],
        //                           (double)legdata_->q_abad[0] + 1.57, (double)legdata_->q_hip[0], (double)legdata_->q_knee[0],
        //                           (double)-(legdata_->q_abad[2] + 1.57), -(double)legdata_->q_hip[2], -(double)legdata_->q_knee[2],
        //                           (double)-(legdata_->q_abad[3] + 1.57), -(double)legdata_->q_hip[3], -(double)legdata_->q_knee[3]}});
        // obs_.dof_vel = torch::tensor({{(double)legdata_->qd_abad[1], (double)legdata_->qd_hip[1], (double)legdata_->qd_knee[1],
        //                           (double)legdata_->qd_abad[0], (double)legdata_->qd_hip[0], (double)legdata_->qd_knee[0],
        //                           (double)-legdata_->qd_abad[2], -(double)legdata_->qd_hip[2], -(double)legdata_->qd_knee[2],
        //                           (double)-legdata_->qd_abad[3], -(double)legdata_->qd_hip[3], -(double)legdata_->qd_knee[3]}});
        if(!stop_update_) {
            obs_.dof_pos[0] = legdata_->q_abad[1] + 1.57;
            obs_.dof_pos[1] = legdata_->q_hip[1];
            obs_.dof_pos[2] = legdata_->q_knee[1];
            obs_.dof_pos[3] = 0;//legdata_->q_wheel_abs[1] - wheel_init_pos_abs_[1];

            obs_.dof_pos[4] = legdata_->q_abad[0] + 1.57;
            obs_.dof_pos[5] = legdata_->q_hip[0];
            obs_.dof_pos[6] = legdata_->q_knee[0];
            obs_.dof_pos[7] = 0;//legdata_->q_wheel_abs[0] - wheel_init_pos_abs_[0];

            obs_.dof_pos[8] = -(legdata_->q_abad[2] + 1.57);
            obs_.dof_pos[9] = -legdata_->q_hip[2];
            obs_.dof_pos[10] = -legdata_->q_knee[2];
            obs_.dof_pos[11] = 0;//-(legdata_->q_wheel_abs[2] - wheel_init_pos_abs_[2]);

            obs_.dof_pos[12] = -(legdata_->q_abad[3] + 1.57);
            obs_.dof_pos[13] = -legdata_->q_hip[3];
            obs_.dof_pos[14] = -legdata_->q_knee[3];
            obs_.dof_pos[15] = 0;//-(legdata_->q_wheel_abs[3] - wheel_init_pos_abs_[3]);

            obs_.dof_vel[0] = legdata_->qd_abad[1];
            obs_.dof_vel[1] = legdata_->qd_hip[1];
            obs_.dof_vel[2] = legdata_->qd_knee[1];
            obs_.dof_vel[3] = legdata_->qd_wheel[1];

            obs_.dof_vel[4] = legdata_->qd_abad[0];
            obs_.dof_vel[5] = legdata_->qd_hip[0];
            obs_.dof_vel[6] = legdata_->qd_knee[0];
            obs_.dof_vel[7] = legdata_->qd_wheel[0];

            obs_.dof_vel[8] = -legdata_->qd_abad[2];
            obs_.dof_vel[9] = -legdata_->qd_hip[2];
            obs_.dof_vel[10] = -legdata_->qd_knee[2];
            obs_.dof_vel[11] = -legdata_->qd_wheel[2];

            obs_.dof_vel[12] = -legdata_->qd_abad[3];
            obs_.dof_vel[13] = -legdata_->qd_hip[3];
            obs_.dof_vel[14] = -legdata_->qd_knee[3];
            obs_.dof_vel[15] = -legdata_->qd_wheel[3];
            Forward();
            // torch::Tensor action_raw = Forward();
            // torch::Tensor output_dof_pos = ComputePosition(actions);

            // desired_pos[0] = output_dof_pos[0][0].item<float>() - 1.57;
            // desired_pos[3] = output_dof_pos[0][2].item<float>() - 1.57;
            // desired_pos[6] = output_dof_pos[0][3].item<float>() - 1.57;
            // desired_pos[9] = output_dof_pos[0][1].item<float>() - 1.57;

            // desired_pos[1] = output_dof_pos[0][4].item<float>();
            // desired_pos[4] = output_dof_pos[0][6].item<float>();
            // desired_pos[7] = -output_dof_pos[0][7].item<float>();
            // desired_pos[10] = -output_dof_pos[0][5].item<float>();

            // desired_pos[2] = output_dof_pos[0][8].item<float>();
            // desired_pos[5] = output_dof_pos[0][10].item<float>();
            // desired_pos[8] = -output_dof_pos[0][11].item<float>();
            // desired_pos[11] = -output_dof_pos[0][9].item<float>();

            // action_raw = action_raw.squeeze(0);
            // // move to cpu
            // action_raw = action_raw.to(torch::kCPU);
            // // assess the result

            // auto action_getter = action_raw.accessor<float,1>();

            for (int j = 0; j < 16; j++)
            {
                //            float  action_value = std::max(std::min(action_getter[j]* action_scale[j], action_delta_max), action_delta_min);
                //            action.at(j) = action_value + init_pos[j];
                //            action_temp.at(j) = action_value/action_scale[j];
                action[j] = output.get()[j] * params_.action_scale * (j % 4 == 0 ? (j > 7 ? 0.75 : 0.5) : 1.0) + params_.default_dof_pos[j];
                // output_last[j] = output.get()[j];
                // std::cout << j << "," << action[j] << std::endl;
                // std::cout << j << "," << obs_.dof_pos[0][j].item<float>() << std::endl;
            }

            desired_pos[0] = action[4] - 1.57;
            desired_pos[4] = action[0] - 1.57;
            desired_pos[8] = -(action[8]) - 1.57;
            desired_pos[12] = -(action[12]) - 1.57;

            desired_pos[1] = action[5];
            desired_pos[5] = action[1];
            desired_pos[9] = -action[9];
            desired_pos[13] = -action[13];

            desired_pos[2] = action[6];
            desired_pos[6] = action[2];
            desired_pos[10] = -action[10];
            desired_pos[14] = -action[14];

            desired_pos[3] = action[7];// + wheel_init_pos_abs_[0];
            desired_pos[7] = action[3];// + wheel_init_pos_abs_[1];
            desired_pos[11] = -(action[11]);// + wheel_init_pos_abs_[2];
            desired_pos[15] = -(action[15]);// + wheel_init_pos_abs_[3];
       }

        absoluteWait(_start_time, (long long)(0.02 * 1000000));
    }
    threadRunning = false;
}

void RLTensorRT_HO_Handle::Package_Init(LegData* legdata, AttitudeData* attidata)
{
    // actor = torch::jit::load("/home/msc/rl_pt/policy_1_new.pt");
    // actor.to(torch::kCUDA);
    // actor.eval();
    // Vec3<float> lin_vel_tmp = posvelest->_stateEstimatorData.result->vWorld;
    // Vec3<float> ang_vel_tmp = attidata->omega_world;
    // Quat<float> quat_tmp = attidata->quat;
    // obs_.lin_vel = torch::tensor({{(double)lin_vel_tmp[0], (double)lin_vel_tmp[1], (double)lin_vel_tmp[2]}});
    // obs_.ang_vel = torch::tensor({{(double)ang_vel_tmp[0], (double)ang_vel_tmp[1], (double)ang_vel_tmp[2]}});
    // obs_.gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    // obs_.commands = torch::tensor({{0.0, 0.0, 0.0}});
    // obs_.base_quat = torch::tensor({{(double)quat_tmp[1],(double)quat_tmp[2],(double)quat_tmp[3], (double)quat_tmp[0]}});
        for (int i = 0; i < 4; i++)
        {
            wheel_init_pos_abs_[i] = (double)legdata->q_wheel_abs[i];
            desired_pos[4*i+3] = 0.;
            desired_pos[4*i] = legdata->q_abad[i];
            desired_pos[4*i+1] = legdata->q_hip[i];
            desired_pos[4*i+2] = legdata->q_knee[i];
        }
        obs_.dof_pos[0] = legdata->q_abad[1] + 1.57;
        obs_.dof_pos[1] = legdata->q_hip[1];
        obs_.dof_pos[2] = legdata->q_knee[1];
        obs_.dof_pos[3] = legdata->q_wheel_abs[1] - wheel_init_pos_abs_[1];

        obs_.dof_pos[4] = legdata->q_abad[0] + 1.57;
        obs_.dof_pos[5] = legdata->q_hip[0];
        obs_.dof_pos[6] = legdata->q_knee[0];
        obs_.dof_pos[7] = legdata->q_wheel_abs[0] - wheel_init_pos_abs_[0];

        obs_.dof_pos[8] = -(legdata->q_abad[2] + 1.57);
        obs_.dof_pos[9] = -legdata->q_hip[2];
        obs_.dof_pos[10] = -legdata->q_knee[2];
        obs_.dof_pos[11] = -(legdata->q_wheel_abs[2] - wheel_init_pos_abs_[2]);

        obs_.dof_pos[12] = -(legdata->q_abad[3] + 1.57);
        obs_.dof_pos[13] = -legdata->q_hip[3];
        obs_.dof_pos[14] = -legdata->q_knee[3];
        obs_.dof_pos[15] = -(legdata->q_wheel_abs[3] - wheel_init_pos_abs_[3]);
    // obs_.dof_pos = torch::tensor({{(double)legdata->q_abad[0] + 1.57, (double)legdata->q_abad[3] + 1.57, (double)legdata->q_abad[1] + 1.57, (double)legdata->q_abad[2] + 1.57,     // hip
    //                              (double)legdata->q_hip[0], -(double)legdata->q_hip[3], (double)legdata->q_hip[1], -(double)legdata->q_hip[2],   // thigh
    //                              (double)legdata->q_knee[0], -(double)legdata->q_knee[3], (double)legdata->q_knee[1], -(double)legdata->q_knee[2]}});
    // obs_.dof_pos = torch::tensor({{(double)legdata->q_abad[0] + 1.57, (double)legdata->q_hip[0], (double)legdata->q_knee[0],
    //                               (double)legdata->q_abad[1] + 1.57, (double)legdata->q_hip[1], (double)legdata->q_knee[1],
    //                               (double)legdata->q_abad[3] + 1.57, -(double)legdata->q_hip[3], -(double)legdata->q_knee[3],
    //                               (double)legdata->q_abad[2] + 1.57, -(double)legdata->q_hip[2], -(double)legdata->q_knee[2]}});
        obs_.dof_vel[0] = legdata->qd_abad[1];
        obs_.dof_vel[1] = legdata->qd_hip[1];
        obs_.dof_vel[2] = legdata->qd_knee[1];
        obs_.dof_vel[3] = legdata->qd_wheel[1];

        obs_.dof_vel[4] = legdata->qd_abad[0];
        obs_.dof_vel[5] = legdata->qd_hip[0];
        obs_.dof_vel[6] = legdata->qd_knee[0];
        obs_.dof_vel[7] = legdata->qd_wheel[0];

        obs_.dof_vel[8] = -legdata->qd_abad[2];
        obs_.dof_vel[9] = -legdata->qd_hip[2];
        obs_.dof_vel[10] = -legdata->qd_knee[2];
        obs_.dof_vel[11] = -legdata->qd_wheel[2];

        obs_.dof_vel[12] = -legdata->qd_abad[3];
        obs_.dof_vel[13] = -legdata->qd_hip[3];
        obs_.dof_vel[14] = -legdata->qd_knee[3];
        obs_.dof_vel[15] = -legdata->qd_wheel[3];
        // obs_.dof_vel = torch::tensor({{(double)legdata->qd_abad[0], (double)legdata->qd_abad[3], (double)legdata->qd_abad[1], (double)legdata->qd_abad[2],     // hip
        //                              (double)legdata->qd_hip[0], -(double)legdata->qd_hip[3], (double)legdata->qd_hip[1], -(double)legdata->qd_hip[2],   // thigh
        //                              (double)legdata->qd_knee[0], -(double)legdata->qd_knee[3], (double)legdata->qd_knee[1], -(double)legdata->qd_knee[2]}});
        // obs_.dof_vel = torch::tensor({{(double)legdata->qd_abad[0], (double)legdata->qd_hip[0], (double)legdata->qd_knee[0],
        //                               (double)legdata->qd_abad[1], (double)legdata->qd_hip[1], (double)legdata->qd_knee[1],
        //                               (double)legdata->qd_abad[3], -(double)legdata->qd_hip[3], -(double)legdata->qd_knee[3],
        //                               (double)legdata->qd_abad[2], -(double)legdata->qd_hip[2], -(double)legdata->qd_knee[2]}});
        // obs_.actions = torch::tensor({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

        params_.clip_obs = 100.0;
        params_.clip_actions = 100.0;
        params_.damping = 0.5;
        params_.stiffness = 20;
        // params_.d_gains = torch::ones(12) * params_.damping;
        // params_.p_gains = torch::ones(12) * params_.stiffness;
        for(int i = 0; i < 16; i++) {
            params_.d_gains[i] = params_.damping;
            params_.p_gains[i] = params_.stiffness;
        }
        params_.action_scale = 0.25;
        params_.num_of_dofs = 16;
        params_.lin_vel_scale = 2.0;
        params_.ang_vel_scale = 0.25;
        params_.dof_pos_scale = 1.0;
        params_.dof_vel_scale = 0.05;
        // params_.commands_scale = torch::tensor({params_.lin_vel_scale, params_.lin_vel_scale, params_.ang_vel_scale});
        params_.commands_scale[0] = params_.lin_vel_scale;
        params_.commands_scale[1] = params_.lin_vel_scale;
        params_.commands_scale[2] = params_.ang_vel_scale;

        // params_.torque_limits = torch::tensor({{50.0, 55.0, 55.0,
        //                                        50.0, 55.0, 55.0,
        //                                        50.0, 55.0, 55.0,
        //                                        50.0, 55.0, 55.0}});
        for(int i = 0; i < 16; i++) {
            if(i % 4 == 0)
                params_.torque_limits[i] = 50.0;
            else if(i % 4 == 3)
                params_.torque_limits[i] = 10.0;
            else
                params_.torque_limits[i] = 55.0;
        }

        //                                           lf, lh,  rf, rh
        // params_.default_dof_pos = torch::tensor({{0.1, 0.1, -0.1, -0.1,     // hip
        //                                             0.8, -1.0, 0.8, -1.0,   // thigh
        //                                             -1.5, 1.5, -1.5, 1.5}});// calf
        const float default_dof_pos_tmp[16] = {-0.1, 0.9, -1.5, 0, 0.1, 0.9, -1.5, 0, -0.1, -0.9, 1.5, 0, 0.1, -0.9, 1.5, 0};
        for(int i = 0; i < 16; i++) {
          params_.default_dof_pos[i] = default_dof_pos_tmp[i];
        }

    x_vel_cmd_ = 0.;
    pitch_cmd_ = 0.;
    attitude_ = attidata;
    legdata_ = legdata;

    // initialize record
    // action_buf = torch::zeros({history_length,12},torch::kCUDA);
   //    obs_buf = torch::zeros({history_length,30},torch::kCUDA);
    // obs_buf = torch::zeros({history_length,45},torch::kCUDA);
    // last_action = torch::zeros({1,12},torch::kCUDA);
    for(int i = 0; i < 513; i++)
      input_1.get()[i] = 0;
    for(int i = 0; i < 16; i++)
      output_last.get()[i] = 0;

    obs_.forward_vec[0] = 1.0;
    obs_.forward_vec[1] = 0.0;
    obs_.forward_vec[2] = 0.0;

    for (int j = 0; j < 16; j++)
    {
        action[j] = obs_.dof_pos[j];
    }
    a_l.setZero();

    for (int i = 0; i < history_length; i++)
    {
        // torch::Tensor obs_tensor = GetObs();
        // // append obs to obs buffer
        // obs_buf = torch::cat({obs_buf.index({Slice(1,None),Slice()}),obs_tensor},0);
        GetObs();

        for (int i = 0; i < 513; i++)
            input_1_temp.get()[i] = input_1.get()[i + 57];

        for (int i = 0; i < 513; i++)
            input_1.get()[i] = input_1_temp.get()[i];

        for (int i = 0; i < 57; i++)
            input_1.get()[i + 513] = input_0.get()[i];
    }
    std::cout << "init finised predict" << std::endl;

    for (int i = 0; i < 10; i++)
    {
        Forward();
    }

    threadRunning = true;
    if(thread_first_) {
      forward_thread = std::thread(&RLTensorRT_HO_Handle::Run_Forward,this);
      thread_first_ = false;
    }
}

void RLTensorRT_HO_Handle::Package_Run()
{   
    for(int i = 0; i < 4; i++)
    {
        // wheel_init_pos_abs_[i] += (i < 2 ? x_vel_cmd_ : -x_vel_cmd_) * 0.002 * 2.0;
        torque_set_[4*i] = 30 * (desired_pos[4*i] - (double)legdata_->q_abad[i]) + 0.75 * (0 - (double)legdata_->qd_abad[i]);
        torque_set_[4*i+1] = 30 * (desired_pos[4*i+1] - (double)legdata_->q_hip[i]) + 0.75 * (0 - (double)legdata_->qd_hip[i]);
        torque_set_[4*i+2] = 30 * (desired_pos[4*i+2] - (double)legdata_->q_knee[i]) + 0.75 * (0 - (double)legdata_->qd_knee[i]);
        torque_set_[4*i+3] = 10 * (desired_pos[4*i+3]) + 0.5 * (0 - (double)legdata_->qd_wheel[i]);
        // torque_set_[4*i+3] = 6.5 * (wheel_init_pos_abs_[i] - (double)legdata_->q_wheel_abs[i]) + 0.5 * ((i < 2 ? x_vel_cmd_ : -x_vel_cmd_) * 2.0 - (double)legdata_->qd_wheel[i]);
    }
}
