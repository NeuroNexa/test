# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
import os
from launch import LaunchDescription, LaunchContext
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


import sys
sys.path.insert(0, os.path.join(get_package_share_directory('tita_bringup'), 'launch'))
from launch_utils import tita_namespace

def generate_launch_description():
    
    canfd_router_node = Node(
        package='titati_canfd_router',
        executable='titati_canfd_router_node',
        name='titati_canfd_router_node',
        namespace=tita_namespace,
        output='screen',
    )

    battery_device_node = Node(
        package='battery_device',
        executable='battery_device_node',
        name='battery_device_node',
        namespace=tita_namespace,
        output='screen',
    )

    nodes = [
        battery_device_node,
        canfd_router_node,
    ]

    return LaunchDescription(nodes)
