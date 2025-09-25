#include "tita_robot/canfd_router_bridge.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

namespace
{
std::atomic<bool> g_running{true};

void handle_signal(int)
{
  g_running.store(false);
}

void print_usage(const char *program)
{
  std::cout << "Usage: " << program
            << " [--interface canX] [--offset <value>] [--no-auto-retry]" << std::endl;
  std::cout << "  --interface, -i    CAN interface name (default: can0)" << std::endl;
  std::cout << "  --offset, -o       Optional CAN ID offset in decimal or hex (e.g. 0x10)" << std::endl;
  std::cout << "  --no-auto-retry    Disable automatic re-handshake when the router drops out" << std::endl;
}
}

int main(int argc, char **argv)
{
  std::string interface = "can0";
  uint8_t can_id_offset = 0x00U;
  bool auto_retry = true;

  for (int idx = 1; idx < argc; ++idx)
  {
    const std::string arg = argv[idx];
    if ((arg == "-i" || arg == "--interface") && idx + 1 < argc)
    {
      interface = argv[++idx];
    }
    else if ((arg == "-o" || arg == "--offset") && idx + 1 < argc)
    {
      const std::string value = argv[++idx];
      try
      {
        size_t processed = 0U;
        const unsigned long parsed = std::stoul(value, &processed, 0);
        if (processed != value.size() || parsed > 0xFFUL)
        {
          throw std::out_of_range("offset");
        }
        can_id_offset = static_cast<uint8_t>(parsed);
      }
      catch (const std::exception &ex)
      {
        std::cerr << "[rl_titati_router] Invalid CAN ID offset '" << value << "': "
                  << ex.what() << std::endl;
        print_usage(argv[0]);
        return 1;
      }
    }
    else if (arg == "--no-auto-retry")
    {
      auto_retry = false;
    }
    else if (arg == "-h" || arg == "--help")
    {
      print_usage(argv[0]);
      return 0;
    }
    else
    {
      std::cerr << "[rl_titati_router] Unknown argument '" << arg << "'." << std::endl;
      print_usage(argv[0]);
      return 1;
    }
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  try
  {
    can_device::CanfdRouterBridge router(interface, can_id_offset, auto_retry);
    router.set_status_callback([](const std::string &message) {
      std::cout << message << std::endl;
    });

    router.request_force_direct();

    std::cout << "[rl_titati_router] Listening on interface '" << interface
              << "' (offset 0x" << std::hex << static_cast<int>(can_id_offset) << std::dec
              << ") with " << (auto_retry ? "auto" : "manual")
              << " retry. Press Ctrl+C to exit." << std::endl;

    while (g_running.load())
    {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[rl_titati_router] Failed to initialise CAN-FD router bridge: "
              << ex.what() << std::endl;
    return 1;
  }

  std::cout << "[rl_titati_router] Shutdown complete." << std::endl;
  return 0;
}
