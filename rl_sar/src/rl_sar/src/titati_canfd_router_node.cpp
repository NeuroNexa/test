/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "titati_canfd_router/canfd_router_can_receive_api.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

namespace
{
std::atomic<bool> keep_running{true};

void signal_handler(int)
{
    keep_running.store(false);
}

void PrintUsage(const char *argv0)
{
    std::cout << "Usage: " << argv0 << " [--stay-alive|--once]" << std::endl;
    std::cout << "  --stay-alive  Keep the program running after the forced-direct command is sent (default)." << std::endl;
    std::cout << "  --once        Exit after switching to forced-direct mode." << std::endl;
}
} // namespace

int main(int argc, char **argv)
{
    bool stay_alive = true;

    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0)
        {
            PrintUsage(argv[0]);
            return 0;
        }
        if (std::strcmp(argv[i], "--stay-alive") == 0)
        {
            stay_alive = true;
        }
        else if (std::strcmp(argv[i], "--once") == 0)
        {
            stay_alive = false;
        }
        else
        {
            std::cerr << "Unknown argument: " << argv[i] << std::endl;
            PrintUsage(argv[0]);
            return 1;
        }
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "[titati_canfd_router_node] Initialising CAN-FD router on can0..." << std::endl;
    auto router = std::make_shared<can_device::CanfdRouterCanReceiveApi>();

    std::cout << "[titati_canfd_router_node] Waiting for power-on self-test to finish (mode=1 or 2)." << std::endl;
    router->set_forcedirect_mode(true);

    bool announced_switch = false;
    bool announced_alive = false;
    while (keep_running.load())
    {
        if (router->forcedirect_complete())
        {
            if (!announced_switch)
            {
                std::cout << "[titati_canfd_router_node] Router switched to forced-direct relay mode." << std::endl;
                announced_switch = true;
            }
            if (stay_alive && !announced_alive)
            {
                std::cout << "[titati_canfd_router_node] Staying alive - press Ctrl+C to exit." << std::endl;
                announced_alive = true;
            }
            if (!stay_alive)
            {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if (!router->forcedirect_complete())
    {
        std::cerr << "[titati_canfd_router_node] Forced-direct command not acknowledged."
                  << " Check wiring and rerun after the self-test completes." << std::endl;
        return 2;
    }

    std::cout << "[titati_canfd_router_node] Done." << std::endl;
    return 0;
}
