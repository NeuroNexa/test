#ifndef INIT_HPP
#define INIT_HPP

#include <stdint.h>
#include "common/LegData.h"
#include "common/cppTypes.h"

class Init_Handle 
{

public:
    Init_Handle(float* torque_set, LegData* legdata):torque_set_(torque_set), legdata_(legdata){}
    void Package_Init();
    void Package_Run();
    bool enable_transition_;

private:
    int count_ = 0;
    bool kneel_flag_ = false;
    float pos_init_[12];
    float pos_wheel_init[4];
    const float pos_final_stand_[12] = {-1.57, 0.75, -1.50, -1.57, 0.75, -1.50, -1.57, 0.75, -1.50, -1.57, 0.75, -1.50};
    const float pos_final_kneel_stage1_[12] = {-1.57, 150.0/57.3, -150.0/57.3, -1.57, 150/57.3, -150/57.3, -1.57, 150/57.3, -150/57.3, -1.57, 150/57.3, -150/57.3};
    const float pos_final_kneel_stage2_[12] = {-1.57, 90.0/57.3, -150.0/57.3, -1.57, 90/57.3, -150/57.3, -1.57, 90/57.3, -150/57.3, -1.57, 90/57.3, -150/57.3};
    float pos_des_[12];
    float* torque_set_;
    LegData* legdata_;

    void standOnly();
    void kneelBeforeStand();
};

#endif

