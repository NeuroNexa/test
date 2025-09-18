#ifndef PROJECT_ATTITUDE_H
#define PROJECT_ATTITUDE_H

#include "common/cppTypes.h"
#include "common/orientation_tools.h"

class AttitudeData {
    public:
        void Update(float q[4], float w[3], float a[3]) {
            float sum = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
                quat[0] = q[0] / sum;
            for(int i = 0; i < 3; i++) {
                quat[i+1] = q[i+1] / sum;
                omega_body[i] = w[i];
                acc_body[i] = a[i];
            }
            rpy = ori::quatToRPY(quat);
            rot_body = ori::quaternionToRotationMatrix(quat);
            omega_world = rot_body.transpose() * omega_body;
            acc_world = rot_body.transpose() * acc_body;
        };
        Vec3<float> rpy;
        Mat3<float> rot_body;
        Vec3<float> omega_world;
        Vec3<float> acc_world;

        Quat<float> quat;
        Vec3<float> omega_body;
        Vec3<float> acc_body;

};

#endif