// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <linux/can.h>

#include <cstdint>
#include <stdexcept>

namespace can_device
{
namespace socket_can
{

enum class FrameType
{
  DATA = 0U,
  ERROR = 1U,
  OVERLOAD = 2U,
  INVALID,
};

class SocketCanId
{
public:
  explicit SocketCanId(const canid_t canid)
  : m_canid(canid)
  {
  }

  canid_t get() const noexcept
  {
    return m_canid;
  }

protected:
  canid_t m_canid;
};

class StandardId : public SocketCanId
{
public:
  explicit StandardId(const canid_t canid)
  : SocketCanId(canid)
  {
    if (m_canid > 0x7FFU)
    {
      throw std::domain_error{"Invalid Standard CAN ID"};
    }
  }
};

class ExtendedId : public SocketCanId
{
public:
  explicit ExtendedId(const canid_t canid)
  : SocketCanId(canid)
  {
    if (m_canid > 0x1FFFFFFFU)
    {
      throw std::domain_error{"Invalid Extended CAN ID"};
    }
  }
};

class CanId
{
public:
  CanId() noexcept
  : m_canid{0U}, m_length{0U}
  {
  }

  explicit CanId(const SocketCanId & id)
  : CanId(id.get())
  {
  }

  explicit CanId(const canid_t canid)
  : CanId(canid, 0U)
  {
  }

  explicit CanId(const canid_t canid, const uint8_t length)
  : m_canid{canid}, m_length{length}
  {
  }

  canid_t get() const noexcept
  {
    return m_canid;
  }

  FrameType frame_type() const noexcept
  {
    if (m_canid & CAN_ERR_FLAG)
    {
      return FrameType::ERROR;
    }
    if (m_canid & CAN_RTR_FLAG)
    {
      return FrameType::OVERLOAD;
    }
    return FrameType::DATA;
  }

  bool frame_is_extended() const noexcept
  {
    return (m_canid & CAN_EFF_FLAG) != 0U;
  }

  StandardId standard() const
  {
    if (frame_is_extended())
    {
      return StandardId(m_canid & CAN_SFF_MASK);
    }
    else
    {
      return StandardId(m_canid & CAN_SFF_MASK);
    }
  }

  ExtendedId extended() const
  {
    if (frame_is_extended())
    {
      return ExtendedId(m_canid & CAN_EFF_MASK);
    }
    else
    {
      throw std::domain_error{"Not an extended frame"};
    }
  }

  uint8_t length() const noexcept
  {
    return m_length;
  }

protected:
  canid_t m_canid;
  uint8_t m_length;
};

}  // namespace socket_can
}  // namespace can_device

