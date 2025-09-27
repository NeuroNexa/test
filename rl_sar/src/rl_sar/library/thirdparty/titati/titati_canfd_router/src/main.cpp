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
#include <time.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "titati_canfd_router/canfd_router_can_receive_api.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tita_system_interfaces/srv/power_self_test_srv.hpp"
#include "tita_utils/topic_names.hpp"

namespace tita
{

class CanfdRouterCanNode : public rclcpp::Node
{
public:
  explicit CanfdRouterCanNode(const rclcpp::NodeOptions & option) : Node("titati_slave_controller_node", option) {
    rmw_qos_profile_t client_qos = rmw_qos_profile_services_default;
    power_self_test_client_ = this->create_client<tita_system_interfaces::srv::PowerSelfTestSrv>(
    tita_topic::kpower_self_test_service, client_qos);
    auto request = std::make_shared<tita_system_interfaces::srv::PowerSelfTestSrv::Request>();
    request->power_self_test.status.resize(1);
    request->power_self_test.status[0].values.resize(1);
    request->power_self_test.status[0].values[0].key = "test_string";

    power_self_test_client_->wait_for_service();
    auto result = power_self_test_client_->async_send_request(request, std::bind(&CanfdRouterCanNode::service_response_callback, this, std::placeholders::_1));
}
  ~CanfdRouterCanNode() = default;

private:
  std::shared_ptr<can_device::CanfdRouterCanReceiveApi> reveive_canfd_router_ =
    std::make_shared<can_device::CanfdRouterCanReceiveApi>();
  rclcpp::Client<tita_system_interfaces::srv::PowerSelfTestSrv>::SharedPtr power_self_test_client_ =
    nullptr;
  void service_response_callback(rclcpp::Client<tita_system_interfaces::srv::PowerSelfTestSrv>::SharedFuture response) {
      (void)response;
      reveive_canfd_router_->set_forcedirect_mode(true);
      RCLCPP_INFO(this->get_logger(), "set_forcedirect_mode is started");
  }
};
}  // namespace tita

int main(int argc, char * argv[])
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<tita::CanfdRouterCanNode>(options);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();

  return 0;
}
