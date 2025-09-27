# Copyright (c) 2024
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    role_arg = DeclareLaunchArgument(
        "role",
        default_value="master",
        description="Logical role of the Jetson running this launch file (master or slave)."
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Optional ROS namespace for Titati infrastructure nodes."
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging verbosity for Titati infrastructure nodes."
    )

    namespace_cfg = LaunchConfiguration("namespace")
    log_level_cfg = LaunchConfiguration("log_level")

    router_node = Node(
        package="rl_sar",
        executable="titati_canfd_router_node",
        name="titati_canfd_router_node",
        namespace=namespace_cfg,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level_cfg],
    )

    battery_node = Node(
        package="rl_sar",
        executable="titati_battery_device_node",
        name="titati_battery_device_node",
        namespace=namespace_cfg,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level_cfg],
        parameters=[{"role": LaunchConfiguration("role")}],
    )

    return LaunchDescription([
        role_arg,
        namespace_arg,
        log_level_arg,
        router_node,
        battery_node,
    ])
