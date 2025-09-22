/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <chrono>
#include <memory>

#if defined(USE_ROS2)
#include "rclcpp/rclcpp.hpp"
#include "titati_canfd_router/canfd_router_can_receive_api.hpp"

namespace titati
{
class TitatiCanfdRouterNode : public rclcpp::Node
{
public:
  TitatiCanfdRouterNode()
  : rclcpp::Node("titati_canfd_router_node")
  {
    RCLCPP_INFO(this->get_logger(), "Starting CAN-FD router bridge (forcedirect mode)");
    router_ = std::make_shared<can_device::CanfdRouterCanReceiveApi>();
    router_->set_forcedirect_mode(true);
    heartbeat_log_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      [this]()
      {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), std::chrono::seconds(5).count() * 1000,
          "Waiting for mode feedback on CAN bus to trigger forcedirect transition...");
      });
  }

private:
  std::shared_ptr<can_device::CanfdRouterCanReceiveApi> router_;
  rclcpp::TimerBase::SharedPtr heartbeat_log_timer_;
};
}  // namespace titati

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<titati::TitatiCanfdRouterNode>());
  rclcpp::shutdown();
  return 0;
}

#else

#include <atomic>
#include <csignal>
#include <iostream>
#include <thread>

#include "titati_canfd_router/canfd_router_can_receive_api.hpp"

namespace
{
std::atomic_bool g_should_run{true};

void handle_signal(int /*signal*/)
{
  g_should_run.store(false);
}
}  // namespace

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  std::cerr << "[titati_canfd_router_node] Starting standalone CAN-FD router (forcedirect mode)." << std::endl;

  auto router = std::make_shared<can_device::CanfdRouterCanReceiveApi>();
  router->set_forcedirect_mode(true);

  while (g_should_run.load()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  std::cerr << "[titati_canfd_router_node] Shutting down." << std::endl;
  return 0;
}

#endif
