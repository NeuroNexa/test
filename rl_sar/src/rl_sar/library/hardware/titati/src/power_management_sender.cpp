#include "titati/battery_device/power_management_sender.hpp"

#include <cstring>

namespace titati
{
namespace battery
{
namespace
{
constexpr int64_t kTimeoutUs = 3'000'000L;

inline void encode_u32(uint8_t * dst, uint32_t value)
{
  dst[0] = static_cast<uint8_t>((value >> 24) & 0xFFU);
  dst[1] = static_cast<uint8_t>((value >> 16) & 0xFFU);
  dst[2] = static_cast<uint8_t>((value >> 8) & 0xFFU);
  dst[3] = static_cast<uint8_t>(value & 0xFFU);
}

inline void encode_u64(uint8_t * dst, uint64_t value)
{
  dst[0] = static_cast<uint8_t>((value >> 56) & 0xFFU);
  dst[1] = static_cast<uint8_t>((value >> 48) & 0xFFU);
  dst[2] = static_cast<uint8_t>((value >> 40) & 0xFFU);
  dst[3] = static_cast<uint8_t>((value >> 32) & 0xFFU);
  dst[4] = static_cast<uint8_t>((value >> 24) & 0xFFU);
  dst[5] = static_cast<uint8_t>((value >> 16) & 0xFFU);
  dst[6] = static_cast<uint8_t>((value >> 8) & 0xFFU);
  dst[7] = static_cast<uint8_t>(value & 0xFFU);
}
}  // namespace

PowerManagementSender::PowerManagementSender()
{
  can_dev_ = std::make_shared<can_device::socket_can::CanDev>(
    "can0", "titati_power_sender", false, true, kTimeoutUs, 0x00U);
}

void PowerManagementSender::send_state(const PowerStateCommand & command)
{
  if (!can_dev_) {
    return;
  }

  struct canfd_frame frame
  {
  };
  frame.can_id = kCanIdPowerStateSet;
  frame.len = static_cast<uint8_t>(kDlcPowerStateSet);
  std::memset(frame.data, 0x00, sizeof(frame.data));

  frame.data[4] = static_cast<uint8_t>(command.power_5v);
  frame.data[5] = static_cast<uint8_t>(command.power_12v);
  frame.data[6] = static_cast<uint8_t>(command.power_24v);
  frame.data[7] = static_cast<uint8_t>(command.power_motor_48v);
  frame.data[8] = static_cast<uint8_t>(command.power_extern_48v);

  frame.data[9] = static_cast<uint8_t>(command.power_5v_delay_ms & 0xFFU);
  frame.data[10] = static_cast<uint8_t>((command.power_5v_delay_ms >> 8) & 0xFFU);
  frame.data[11] = static_cast<uint8_t>(command.power_12v_delay_ms & 0xFFU);
  frame.data[12] = static_cast<uint8_t>((command.power_12v_delay_ms >> 8) & 0xFFU);
  frame.data[13] = static_cast<uint8_t>(command.power_24v_delay_ms & 0xFFU);
  frame.data[14] = static_cast<uint8_t>((command.power_24v_delay_ms >> 8) & 0xFFU);
  frame.data[15] = static_cast<uint8_t>(command.power_motor_48v_delay_ms & 0xFFU);
  frame.data[16] = static_cast<uint8_t>((command.power_motor_48v_delay_ms >> 8) & 0xFFU);
  frame.data[17] = static_cast<uint8_t>(command.power_extern_48v_delay_ms & 0xFFU);
  frame.data[18] = static_cast<uint8_t>((command.power_extern_48v_delay_ms >> 8) & 0xFFU);
  frame.data[19] = command.fixed;

  can_dev_->send_can_message(frame);
}

void PowerManagementSender::send_self_test(const PowerSelfTestReport & report)
{
  if (!can_dev_) {
    return;
  }

  struct canfd_frame frame
  {
  };
  frame.can_id = kCanIdPowerSelfTest;
  frame.len = static_cast<uint8_t>(kDlcPowerSelfTest);
  std::memset(frame.data, 0x00, sizeof(frame.data));

  encode_u64(&frame.data[4], report.rst);
  encode_u32(&frame.data[12], report.app_version);
  encode_u32(&frame.data[16], report.build_timestamp);
  encode_u32(&frame.data[20], report.uuid_0);
  encode_u32(&frame.data[24], report.uuid_1);
  encode_u32(&frame.data[28], report.uuid_2);

  can_dev_->send_can_message(frame);
}

void PowerManagementSender::send_heartbeat(const PowerHeartbeat & heartbeat)
{
  if (!can_dev_) {
    return;
  }

  struct canfd_frame frame
  {
  };
  frame.can_id = kCanIdPowerHeartbeat;
  frame.len = static_cast<uint8_t>(kDlcPowerHeartbeat);
  std::memset(frame.data, 0x00, sizeof(frame.data));

  encode_u32(&frame.data[4], heartbeat.cur_mode);
  frame.data[8] = heartbeat.right_history;
  frame.data[9] = heartbeat.left_history;

  can_dev_->send_can_message(frame);
}

}  // namespace battery
}  // namespace titati
