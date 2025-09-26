#pragma once

#include <cstdint>

namespace titati
{
namespace battery
{

constexpr uint32_t kCanIdPowerStateSet = 0x86U;
constexpr uint32_t kCanIdPowerSelfTest = 0x87U;
constexpr uint32_t kCanIdPowerHeartbeat = 0x90U;

constexpr std::size_t kDlcPowerStateSet = 20U;
constexpr std::size_t kDlcPowerSelfTest = 32U;
constexpr std::size_t kDlcPowerHeartbeat = 10U;

struct PowerStateCommand
{
  int8_t power_5v{0};
  int8_t power_12v{0};
  int8_t power_24v{0};
  int8_t power_motor_48v{0};
  int8_t power_extern_48v{0};

  uint16_t power_5v_delay_ms{0};
  uint16_t power_12v_delay_ms{0};
  uint16_t power_24v_delay_ms{0};
  uint16_t power_motor_48v_delay_ms{0};
  uint16_t power_extern_48v_delay_ms{0};

  uint8_t fixed{0x55U};
};

struct PowerSelfTestReport
{
  uint64_t rst{0};
  uint32_t app_version{0};
  uint32_t build_timestamp{0};
  uint32_t uuid_0{0};
  uint32_t uuid_1{0};
  uint32_t uuid_2{0};
};

struct PowerHeartbeat
{
  uint32_t cur_mode{0};
  uint8_t right_history{0};
  uint8_t left_history{0};
};

}  // namespace battery
}  // namespace titati
