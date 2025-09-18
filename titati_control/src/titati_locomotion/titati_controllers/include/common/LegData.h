#ifndef PROJECT_LEGDATA_H
#define PROJECT_LEGDATA_H

#include "common/cppTypes.h"

#define BODY_LINK_WIDTH  0.0895f
#define ABAD_LINK_LEN  0.1413f
#define HIP_LINK_LEN   0.200f
#define KNEE_LINK_LEN  0.200f
#define WHEEL_LINK_LEN 0//0.0502f
#define BODY_LINK_LENGTH 0.260f

class LegData {
    public:
        void Update(float q[16], float qd[16]) {
            for(int i = 0; i < 4; i++) { 
                q_abad[i] = q[4*i] - 1.57;
                if(q[4*i+1] < -2.5)
                  q_hip[i] = q[4*i+1] + 2.0 * 3.1415926;
                else
                  q_hip[i] = q[4*i+1];
                q_knee[i] = q[4*i+2];
                if(q[4*i+3] < -1.57 && q_wheel[i] > 1.57)
                  round_num[i]++;
                else if(q[4*i+3] > 1.57 && q_wheel[i] < -1.57)
                  round_num[i]--;
                q_wheel_abs[i] = q[4*i+3] + round_num[i] * 2.0 * 3.1415926;
                q_wheel[i] = q[4*i+3];
                qd_abad[i] = qd[4*i];
                qd_hip[i] = qd[4*i+1];
                qd_knee[i] = qd[4*i+2];
                qd_wheel[i] = qd[4*i+3];
                UpdatePos(i);
                UpdateJaco(i);
                UpdateVel(i);
            }
        };
        Vec3<float> GetHipLocation(int i) {
            float y_sidesign = ((i == 0 || i == 3) ? 1.0 : -1.0);
            float x_sidesign = (i < 2 ? 1.0 : -1.0);
            return Vec3<float>(x_sidesign * (BODY_LINK_LENGTH), y_sidesign * (BODY_LINK_WIDTH), 0);
        }
        Vec3<float> foot_pos_local[4];
        Vec3<float> foot_vel_local[4];
        Mat3<float> foot_Jaco_local[4];
        float q_abad[4];
        float q_hip[4];
        float q_knee[4];
        float q_wheel[4];
        float q_wheel_abs[4];
        float qd_abad[4];
        float qd_hip[4];
        float qd_knee[4];
        float qd_wheel[4];
        int round_num[4] = {0,0,0,0};
    private:
        void UpdatePos(int i) {
            float y_sidesign = ((i == 0 || i == 2) ? 1.0 : -1.0);
            float x_sidesign = (i < 2 ? 1.0 : -1.0);
            // const float a1 = BODY_LINK_WIDTH * _sideSign;
            const float a3 = HIP_LINK_LEN;
            const float a4 = KNEE_LINK_LEN;
            const float d2 = ABAD_LINK_LEN * y_sidesign;
            const float d4 = WHEEL_LINK_LEN * y_sidesign;

            float s1 = sinf(q_abad[i]);
            float s2 = sinf(q_hip[i]);
            // float s3 = sinf(q_knee[i]);

            float c1 = cosf(q_abad[i]);
            float c2 = cosf(q_hip[i]);
            // float c3 = cosf(q_knee[i]);
            float c23 = cosf(q_hip[i] + q_knee[i]);
            float s23 = sinf(q_hip[i] + q_knee[i]);

            foot_pos_local[i][0] = x_sidesign * (-a3 * s2 - a4 * s23);
            foot_pos_local[i][1] = x_sidesign * (a3 * c1 * c2 + a4 * c1 * c23 - (d2 + d4) * s1);//  + a1 ;
            foot_pos_local[i][2] = a3 * s1 * c2 + a4 * s1 * c23 + (d2 + d4) * c1;
        };

        void UpdateJaco(int i) {
            float y_sidesign = ((i == 0 || i == 2)? 1.0 : -1.0);
            float x_sidesign = (i < 2 ? 1.0 : -1.0);
            // const float a1 = BODY_LINK_WIDTH * _sideSign;
            const float a3 = HIP_LINK_LEN;
            const float a4 = KNEE_LINK_LEN;
            const float d2 = ABAD_LINK_LEN * y_sidesign;
            const float d4 = WHEEL_LINK_LEN * y_sidesign;
            float s1 = sinf(q_abad[i]);
            float s2 = sinf(q_hip[i]);
            float s3 = sinf(q_knee[i]);

            float c1 = cosf(q_abad[i]);
            float c2 = cosf(q_hip[i]);
            float c3 = cosf(q_knee[i]);
            float c23 = cosf(q_hip[i] + q_knee[i]);
            float s23 = sinf(q_hip[i] + q_knee[i]);

            foot_Jaco_local[i](0,0) = 0;
            foot_Jaco_local[i](0,1) = x_sidesign * (-a3 * c2 - a4 * c23);
            foot_Jaco_local[i](0,2) = x_sidesign * (-a4 * c23);

            foot_Jaco_local[i](1,0) = x_sidesign * (a4 * s1 * s2 * s3 - d4 * c1 - a3 * c2 * s1 - a4 * c2 * c3 * s1 - d2 * c1);
            foot_Jaco_local[i](1,1) = x_sidesign * (-c1 * (a3 * s2 + a4 * s23));
            foot_Jaco_local[i](1,2) = x_sidesign * (-a4 * c1 * s23);

            foot_Jaco_local[i](2,0) = a3 * c1 * c2 - d4 * s1 - d2 * s1 + a4 * c1 * c2 * c3 - a4 * c1 * s2 * s3;
            foot_Jaco_local[i](2,1) = -s1 * (a3 * s2 + a4 * s23);
            foot_Jaco_local[i](2,2) = -a4 * s1 * s23;

        };

        void UpdateVel(int i) {
            foot_vel_local[i] = foot_Jaco_local[i] * Vec3<float>(qd_abad[i],qd_hip[i],qd_knee[i]);
        }
};

#endif