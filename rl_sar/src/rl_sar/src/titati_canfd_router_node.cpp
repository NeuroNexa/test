/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <chrono>
#include <memory>

#if defined(USE_ROS2) && defined(USE_ROS)
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

#include <iostream>

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;
  std::cerr << "titati_canfd_router_node requires ROS 2 to build and run." << std::endl;
  return 1;
}

#endif
