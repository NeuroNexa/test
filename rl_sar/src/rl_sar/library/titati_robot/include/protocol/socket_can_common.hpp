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
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#include "socket_can_id.hpp"
#include "visibility_control.hpp"

namespace can_device
{
namespace socket_can
{

inline SOCKETCAN_LOCAL bool bind_can_socket(int32_t & fd, const std::string & interface, bool canfd_on)
{
  struct ifreq ifr;
  fd = socket(PF_CAN, SOCK_RAW, canfd_on ? CAN_RAW_FD_FRAMES : CAN_RAW);
  if (fd < 0)
  {
    printf("[CAN][ERROR][%s] socket_creat()\r\n", interface.c_str());
    return false;
  }
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
  {
    printf("[CAN][ERROR][%s] ioctl() get index error\r\n", interface.c_str());
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    printf("[CAN][ERROR][%s] bind() failed!\r\n", interface.c_str());
    return false;
  }

  if (canfd_on)
  {
    int enable_canfd = 1;
    setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));
  }

  return true;
}

inline SOCKETCAN_LOCAL fd_set single_set(const int32_t fd)
{
  fd_set set;
  FD_ZERO(&set);
  FD_SET(fd, &set);
  return set;
}

inline SOCKETCAN_LOCAL timeval to_timeval(const std::chrono::nanoseconds timeout)
{
  timeval tv;
  tv.tv_sec = static_cast<decltype(tv.tv_sec)>(
    std::chrono::duration_cast<std::chrono::seconds>(timeout).count());
  tv.tv_usec = static_cast<decltype(tv.tv_usec)>(
    std::chrono::duration_cast<std::chrono::microseconds>(
      timeout - std::chrono::duration_cast<std::chrono::seconds>(timeout)).count());
  return tv;
}

class SOCKETCAN_PUBLIC SocketCanTimeout : public std::runtime_error
{
public:
  explicit SocketCanTimeout(const std::string & what_arg)
  : std::runtime_error(what_arg)
  {
  }
};

}  // namespace socket_can
}  // namespace can_device

