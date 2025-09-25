/*! @file IMUTypes.h
 *  @brief Data from IMUs
 */

#ifndef PROJECT_IMUTYPES_H
#define PROJECT_IMUTYPES_H

#include "common/cppTypes.h"

/*!
 * Mini Cheetah's IMU
 */
struct VectorNavData {
  Vec3<float> accelerometer;
  Vec3<float> gyro;
  Quat<float> quat;
  // todo is there status for the vectornav?
};

#endif  // PROJECT_IMUTYPES_H
