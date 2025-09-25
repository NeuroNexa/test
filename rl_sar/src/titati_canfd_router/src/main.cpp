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

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

#include "titati_canfd_router/canfd_router_can_receive_api.hpp"

namespace
{
std::atomic_bool g_running{true};

void signal_handler(int)
{
  g_running.store(false);
}
}  // namespace

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  auto router = std::make_shared<can_device::CanfdRouterCanReceiveApi>();
  router->set_forcedirect_mode(true);

  std::cout << "[titati_can_router] Waiting for CANFD router heartbeat..." << std::endl;
  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  std::cout << "[titati_can_router] Shutdown requested. Exiting." << std::endl;
  return 0;
}
