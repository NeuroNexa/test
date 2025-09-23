#include "tita_hardware/canfd_router.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

namespace
{
std::atomic_bool keep_running{true};

void SignalHandler(int)
{
    keep_running.store(false);
}
}

int main()
{
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    can_device::CanfdRouterCanReceiveApi router;
    router.set_forcedirect_mode(true);

    std::cout << "titati_canfd_router running. Press Ctrl+C to exit." << std::endl;
    while (keep_running.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cout << "titati_canfd_router stopped." << std::endl;
    return 0;
}

