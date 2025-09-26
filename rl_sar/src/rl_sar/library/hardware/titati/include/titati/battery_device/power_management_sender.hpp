#pragma once

#include <memory>

#include "titati/battery_device/power_management_types.hpp"
#include "titati/protocol/can_utils.hpp"

namespace titati
{
namespace battery
{

class PowerManagementSender
{
public:
  PowerManagementSender();

  void send_state(const PowerStateCommand & command);
  void send_self_test(const PowerSelfTestReport & report);
  void send_heartbeat(const PowerHeartbeat & heartbeat);

private:
  std::shared_ptr<can_device::socket_can::CanDev> can_dev_;
};

}  // namespace battery
}  // namespace titati
