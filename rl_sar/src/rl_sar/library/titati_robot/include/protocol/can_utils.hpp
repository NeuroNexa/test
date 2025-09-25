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

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "linux/can.h"
#include "linux/can/error.h"
#include "linux/can/raw.h"
#include "socket_can_receiver.hpp"
#include "socket_can_sender.hpp"

#define C_END "\033[m"
#define C_RED "\033[0;31m"
#define C_YELLOW "\033[1;33m"

#ifdef GCC_OPTIMIZE_03
#pragma GCC optimize("O3")
#else
#pragma GCC optimize("O0")
#endif

namespace can_device
{
namespace socket_can
{
using can_std_callback = std::function<void(std::shared_ptr<struct can_frame> recv_frame)>;
using can_fd_callback = std::function<void(std::shared_ptr<struct canfd_frame> recv_frame)>;
class CanRxDev
{
public:
  explicit CanRxDev(
    const std::string & interface, const std::string & name, can_std_callback recv_callback,
    bool is_block, int64_t nano_timeout = -1, uint8_t can_id_offset = 0)
  : can_id_offset_(can_id_offset)
  {
    init(interface, name, false, recv_callback, nullptr, is_block, nano_timeout);
  }
  explicit CanRxDev(
    const std::string & interface, const std::string & name, can_fd_callback recv_callback,
    bool is_block, int64_t nano_timeout = -1, uint8_t can_id_offset = 0)
  : can_id_offset_(can_id_offset)
  {
    init(interface, name, true, nullptr, recv_callback, is_block, nano_timeout);
  }

  ~CanRxDev()
  {
    ready_ = false;
    isthreadrunning_ = false;
    if (main_T_ != nullptr)
    {
      main_T_->join();
    }
    main_T_ = nullptr;
    receiver_ = nullptr;
  }

  bool is_ready() { return ready_; }
  bool is_timeout() { return is_timeout_; }
  void set_filter(const struct can_filter filter[], size_t s)
  {
    if (receiver_ != nullptr)
    {
      receiver_->set_filter(filter, s);
    }
  }

  std::shared_ptr<canfd_frame> wait_for_can_data_block()
  {
    can_device::socket_can::CanId receive_id{};
    std::string can_type;
    if (canfd_)
    {
      can_type = "FD";
      rx_fd_frame_ = std::make_shared<struct canfd_frame>();
    }
    else
    {
      can_type = "STD";
      rx_std_frame_ = std::make_shared<struct can_frame>();
    }
    if (receiver_ != nullptr)
    {
      try
      {
        bool result =
          canfd_ ? receiver_->receive(rx_fd_frame_, std::chrono::nanoseconds(nano_timeout_))
                 : receiver_->receive(rx_std_frame_, std::chrono::nanoseconds(nano_timeout_));
        if (result)
        {
          is_timeout_ = false;
          return rx_fd_frame_;
        }
        else
        {
          return nullptr;
        }
      }
      catch (const std::exception & ex)
      {
        if (ex.what()[0] == '$')
        {
          is_timeout_ = true;
          return nullptr;
        }
      }
    }
    else
    {
      isthreadrunning_ = false;
      printf(
        C_RED "[CAN_RX %s][ERROR][%s] Error receiving CAN %s message: %s, "
              "no receiver init\r\n" C_END,
        can_type.c_str(), name_.c_str(), can_type.c_str(), interface_.c_str());
    }
    return nullptr;
  }

#ifdef COMMON_PROTOCOL_TEST
  bool testing_setcandata(can_frame frame)
  {
    if (can_std_callback_ != nullptr)
    {
      can_std_callback_(std::make_shared<struct can_frame>(frame));
      return true;
    }
    return false;
  }
  bool testing_setcandata(canfd_frame frame)
  {
    if (can_fd_callback_ != nullptr)
    {
      can_fd_callback_(std::make_shared<struct canfd_frame>(frame));
      return true;
    }
    return false;
  }
#endif  // COMMON_PROTOCOL_TEST

private:
  bool ready_;
  bool canfd_;
  bool is_timeout_;
  bool extended_frame_;
  bool isthreadrunning_;
  int64_t nano_timeout_;
  std::string name_;
  std::string interface_;
  can_std_callback can_std_callback_;
  can_fd_callback can_fd_callback_;
  std::unique_ptr<std::thread> main_T_;
  std::shared_ptr<struct can_frame> rx_std_frame_;
  std::shared_ptr<struct canfd_frame> rx_fd_frame_;
  std::unique_ptr<can_device::socket_can::SocketCanReceiver> receiver_;
  uint8_t can_id_offset_ = 0;
  void init(
    const std::string & interface, const std::string & name, bool canfd_on,
    can_std_callback std_callback, can_fd_callback fd_callback, bool is_block, int64_t nano_timeout)
  {
    name_ = name;
    interface_ = interface;
    canfd_ = canfd_on;
    nano_timeout_ = nano_timeout;
    is_timeout_ = false;
    interface_ = interface;
    can_std_callback_ = std_callback;
    can_fd_callback_ = fd_callback;
    try
    {
      receiver_ = std::make_unique<can_device::socket_can::SocketCanReceiver>(interface_, canfd_);
      if (!is_block)
      {
        isthreadrunning_ = true;
        ready_ = true;
        main_T_ = std::make_unique<std::thread>(std::bind(&CanRxDev::main_recv_func, this));
      }
    }
    catch (const std::exception & ex)
    {
      printf(
        C_RED "[CAN_RX][ERROR][%s] %s receiver creat error! %s\r\n" C_END, name_.c_str(),
        interface_.c_str(), ex.what());
      return;
    }
  }

  bool wait_for_can_data()
  {
    can_device::socket_can::CanId receive_id{};
    std::string can_type;
    if (canfd_)
    {
      can_type = "FD";
      rx_fd_frame_ = std::make_shared<struct canfd_frame>();
    }
    else
    {
      can_type = "STD";
      rx_std_frame_ = std::make_shared<struct can_frame>();
    }

    if (receiver_ != nullptr)
    {
      try
      {
        bool result =
          canfd_ ? receiver_->receive(rx_fd_frame_, std::chrono::nanoseconds(nano_timeout_))
                 : receiver_->receive(rx_std_frame_, std::chrono::nanoseconds(nano_timeout_));
        if (result)
        {
          if (canfd_)
          {
            receive_id = can_device::socket_can::CanId{
              rx_fd_frame_->can_id + (can_id_offset_ << 16U), rx_fd_frame_->len};
            if (receive_id.frame_type() == can_device::socket_can::FrameType::DATA)
            {
              can_fd_callback_(rx_fd_frame_);
            }
          }
          else
          {
            receive_id = can_device::socket_can::CanId{
              rx_std_frame_->can_id + (can_id_offset_ << 16U), rx_std_frame_->can_dlc};
            if (receive_id.frame_type() == can_device::socket_can::FrameType::DATA)
            {
              can_std_callback_(rx_std_frame_);
            }
          }
          is_timeout_ = false;
        }
        return result;
      }
      catch (const std::exception & ex)
      {
        if (ex.what()[0] == '$')
        {
          is_timeout_ = true;
        }
        else
        {
          printf(
            C_RED "[CAN_RX][ERROR][%s] Error receiving CAN %s message: %s, %s" C_END "\r\n",
            can_type.c_str(), can_type.c_str(), interface_.c_str(), ex.what());
        }
        return false;
      }
    }
    else
    {
      isthreadrunning_ = false;
      printf(
        C_RED "[CAN_RX %s][ERROR][%s] Error receiving CAN %s message: %s, "
              "no receiver init\r\n" C_END,
        can_type.c_str(), name_.c_str(), can_type.c_str(), interface_.c_str());
      return false;
    }
  }

  void main_recv_func()
  {
    while (isthreadrunning_)
    {
      if (wait_for_can_data())
      {
        is_timeout_ = false;
      }
    }
  }
};

class CanTxDev
{
public:
  explicit CanTxDev(
    const std::string & interface, const std::string & name, bool canfd_on,
    bool extend_frame, bool wait_timeout = false, bool check_timeout = false, uint8_t can_id_offset = 0)
  : extend_frame_(extend_frame), check_timeout_(check_timeout), wait_timeout_(wait_timeout),
    can_id_offset_(can_id_offset)
  {
    init(interface, name, canfd_on);
  }
  ~CanTxDev()
  {
    if (sender_ != nullptr)
    {
      delete sender_;
    }
    sender_ = nullptr;
  }

  bool is_ready() { return ready_; }
  void config_default_id(socket_can::CanId can_id)
  {
    if (sender_ != nullptr)
    {
      delete sender_;
      sender_ = nullptr;
    }
    sender_ = new can_device::socket_can::SocketCanSender(interface_, canfd_, can_id);
  }
  bool send_can_message(const struct canfd_frame & frame)
  {
    try
    {
      bool canfd = true;
      if (sender_ == nullptr)
      {
        canfd = frame.flags & CANFD_BRS;
        sender_ = new can_device::socket_can::SocketCanSender(interface_, canfd);
      }
      uint32_t id = frame.can_id;
      if ((id == 0 || id > 2047) && canfd)
      {
        id = id >> 11;
        id &= 0x1FFU;
        id |= CAN_EFF_FLAG;
      }
      if (extend_frame_)
      {
        id &= CAN_EFF_MASK;
        id |= CAN_EFF_FLAG;
      }
      id += (can_id_offset_ << 16U);
      if (wait_timeout_)
      {
        sender_->wait(std::chrono::milliseconds(10));
      }

      sender_->send_fd(frame.data, frame.len, can_device::socket_can::CanId{id});
      return true;
    }
    catch (const std::exception & ex)
    {
      if (ex.what()[0] == '$')
      {
        return false;
      }
      printf(
        C_RED "[CAN_TX][ERROR][%s] Error send CAN message: %s" C_END "\r\n", interface_.c_str(),
        ex.what());
      return false;
    }
  }
  bool send_can_message(const struct can_frame & frame)
  {
    try
    {
      bool canfd = false;
      if (sender_ == nullptr)
      {
        canfd = frame.can_dlc > CAN_MAX_DLEN;
        sender_ = new can_device::socket_can::SocketCanSender(interface_, canfd);
      }
      uint32_t id = frame.can_id;
      if ((id == 0 || id > 2047) && !canfd)
      {
        id = id >> 18;
        id &= 0x7FFU;
      }
      if (extend_frame_)
      {
        id &= CAN_EFF_MASK;
        id |= CAN_EFF_FLAG;
      }
      id += (can_id_offset_ << 16U);
      if (wait_timeout_)
      {
        sender_->wait(std::chrono::milliseconds(10));
      }
      sender_->send(frame.data, frame.can_dlc, can_device::socket_can::CanId{id});
      return true;
    }
    catch (const std::exception & ex)
    {
      if (ex.what()[0] == '$')
      {
        return false;
      }
      printf(
        C_RED "[CAN_TX][ERROR][%s] Error send CAN message: %s" C_END "\r\n", interface_.c_str(),
        ex.what());
      return false;
    }
  }

private:
  bool ready_;
  bool canfd_;
  bool extend_frame_;
  bool check_timeout_;
  bool wait_timeout_;
  std::string name_;
  std::string interface_;
  can_device::socket_can::SocketCanSender * sender_;
  std::unique_ptr<std::thread> main_T_;
  std::shared_ptr<struct can_frame> rx_std_frame_;
  std::shared_ptr<struct canfd_frame> rx_fd_frame_;
  uint8_t can_id_offset_ = 0;
  void init(const std::string & interface, const std::string & name, bool canfd)
  {
    interface_ = interface;
    name_ = name;
    canfd_ = canfd;
    ready_ = false;
    try
    {
      sender_ = new can_device::socket_can::SocketCanSender(interface_, canfd_);
    }
    catch (const std::exception & ex)
    {
      printf(
        C_RED "[CAN_TX][ERROR][%s] %s sender creat error! %s\r\n" C_END, name_.c_str(),
        interface_.c_str(), ex.what());
      return;
    }
    ready_ = true;
  }
};

}  // namespace socket_can
}  // namespace can_device

