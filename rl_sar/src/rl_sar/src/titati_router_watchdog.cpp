#include <csignal>
#include <iostream>
#include <thread>

#include "titati/force_direct_manager.hpp"

namespace
{
volatile std::sig_atomic_t g_running = 1;

void signal_handler(int)
{
    g_running = 0;
}
}

int main()
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    titati::ForceDirectManager manager;
    manager.start();

    std::cout << "Titati CAN-FD router watchdog started. Press Ctrl+C to exit." << std::endl;

    while (g_running)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    manager.stop();
    std::cout << "Watchdog stopped." << std::endl;
    return 0;
}
