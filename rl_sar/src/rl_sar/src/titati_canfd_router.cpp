#include "titati/canfd_router.hpp"

#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    titati::CanfdRouter router;
    router.RequestForceDirectMode();
    std::cout << "[INFO] Titati CAN-FD router is running. Force-direct request sent." << std::endl;
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
