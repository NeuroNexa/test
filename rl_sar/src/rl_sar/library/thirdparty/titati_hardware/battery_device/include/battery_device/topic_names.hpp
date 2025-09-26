#pragma once

#include <string>

namespace tita
{
namespace battery_device
{
namespace topics
{
inline const std::string kLeftBattery = "system/battery/left";
inline const std::string kRightBattery = "system/battery/right";
inline const std::string kLeftBatteryDiag = "system/battery_diagnostic/left";
inline const std::string kRightBatteryDiag = "system/battery_diagnostic/right";
inline const std::string kPowerDomainDiag = "system/power_domain_info_diagnostic/power_domain_info";
inline const std::string kPowerStateSetSrv = "system/power_supply/power_state_set_srv";
inline const std::string kPowerHeartBeatSrv = "system/power_supply/power_heart_beat_srv";
inline const std::string kPowerSelfTestSrv = "system/power_supply/power_self_test_srv";
}  // namespace topics
}  // namespace battery_device
}  // namespace tita
