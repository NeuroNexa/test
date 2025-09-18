/*
 * @Author: mashicheng mashicheng@directdrivetec.com
 * @Date: 2024-06-27 20:09:35
 * @LastEditors: mashicheng mashicheng@directdrivetec.com
 * @LastEditTime: 2024-06-28 13:27:05
 * @FilePath: /tita_ros2/repos/apollo/src/libraries/locomotion/gait_control/include/task/PureDamper.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef PUREDAMPER_HPP
#define PUREDAMPER_HPP

#include <stdint.h>
#include "common/LegData.h"
#include "common/cppTypes.h"

class PureDamper_Handle 
{

public:
    PureDamper_Handle(float* torque_set, LegData* legdata):torque_set_(torque_set), legdata_(legdata){}
    void Package_Init();
    void Package_Run();
    bool enable_transition_;

private:
    int count_ = 0;
    float pos_init_[12];
    float pos_wheel_init[4];
    const float pos_final_pd_[12] = {-1.57, 60.0/57.3, -150.0/57.3, -1.57, 60.0/57.3, -150.0/57.3, -1.57, 60.0/57.3, -150.0/57.3, -1.57, 60.0/57.3, -150.0/57.3};
    float pos_des_[12];
    float* torque_set_;
    LegData* legdata_;
};

#endif