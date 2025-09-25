/*
 * Titati hardware interface used by rl_sar.
 */

#ifndef RL_SAR_TITATI_HARDWARE_HPP_
#define RL_SAR_TITATI_HARDWARE_HPP_

#include <array>
#include <memory>
#include <vector>

#include "titati/tita_robot.hpp"
#include "titati/canfd_router.hpp"

namespace rl_sar
{

struct TitatiImuState
{
  std::array<double, 4> quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> angular_velocity{0.0, 0.0, 0.0};
  std::array<double, 3> linear_acceleration{0.0, 0.0, 0.0};
};

struct TitatiJointState
{
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> torque;
};

class TitatiHardware
{
public:
  explicit TitatiHardware(std::size_t motor_count = 16);
  ~TitatiHardware();

  TitatiHardware(const TitatiHardware &) = delete;
  TitatiHardware & operator=(const TitatiHardware &) = delete;

  bool EnableDirectControl();
  bool DisableDirectControl();

  TitatiJointState ReadJointState() const;
  TitatiImuState ReadImuState() const;

  bool SendTorqueCommand(const std::vector<double> &torque) const;
  bool SendMITCommand(
    const std::vector<double> &position,
    const std::vector<double> &velocity,
    const std::vector<double> &kp,
    const std::vector<double> &kd,
    const std::vector<double> &torque) const;

  std::size_t MotorCount() const { return motor_count_; }

private:
  std::unique_ptr<tita_robot> robot_;
  std::shared_ptr<titati::CanfdRouter> router_;
  std::size_t motor_count_;
  bool direct_control_enabled_{false};
};

}  // namespace rl_sar

#endif  // RL_SAR_TITATI_HARDWARE_HPP_
