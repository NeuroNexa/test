/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "titati_canfd_router/canfd_router_can_receive_api.hpp"

#include <atomic>
#include <csignal>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

namespace
{
std::atomic<bool> g_running{true};

void SignalHandler(int)
{
    g_running.store(false);
}

void PrintUsage(const char *binary)
{
    std::cout << "Usage: " << binary << " [--interface <can_if>] [--rpc-interface <can_if>] [--no-force]" << std::endl;
    std::cout << "\n";
    std::cout << "Options:" << std::endl;
    std::cout << "  --interface <can_if>        CAN interface used to receive router status (default: can0)" << std::endl;
    std::cout << "  --rpc-interface <can_if>    CAN interface used to send RPC commands (default: same as --interface)" << std::endl;
    std::cout << "  --no-force                  Do not automatically request FORCE_DIRECT mode" << std::endl;
    std::cout << "  -h, --help                  Show this help message" << std::endl;
}
} // namespace

int main(int argc, char **argv)
{
    std::string receive_interface = "can0";
    std::string rpc_interface;
    bool force_direct = true;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--interface" && i + 1 < argc)
        {
            receive_interface = argv[++i];
        }
        else if (arg == "--rpc-interface" && i + 1 < argc)
        {
            rpc_interface = argv[++i];
        }
        else if (arg == "--no-force")
        {
            force_direct = false;
        }
        else if (arg == "-h" || arg == "--help")
        {
            PrintUsage(argv[0]);
            return 0;
        }
        else
        {
            std::cerr << "Unknown argument: " << arg << std::endl;
            PrintUsage(argv[0]);
            return 1;
        }
    }

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    try
    {
        can_device::CanfdRouterCanReceiveApi router(receive_interface, rpc_interface);
        if (force_direct)
        {
            router.set_forcedirect_mode(true);
            std::cout << "[titati_can_router] Waiting for Titati CAN router heartbeat on "
                      << receive_interface << "..." << std::endl;
        }
        else
        {
            std::cout << "[titati_can_router] Monitoring Titati CAN router on " << receive_interface
                      << " without sending FORCE_DIRECT request." << std::endl;
        }

        while (g_running.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << "[titati_can_router] Failed to initialise CAN router: " << ex.what() << std::endl;
        return 1;
    }

    std::cout << "[titati_can_router] Shutdown requested." << std::endl;
    return 0;
}
