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
from launch_utils import *

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_webots",
            default_value="false",
            description="Decide use webots or not",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ctrl_mode",
            default_value="wbc",
            choices=["wbc", "sdk", "mcu"],
            description="Select wheel-legged robot control methods, mcu means on mcu-board control",
        )
    )
    use_webots = LaunchConfiguration("use_webots")
    ctrl_mode = LaunchConfiguration("ctrl_mode")

    prefix=tita_namespace

    # Initialize Arguments
    # save urdf
    robot_description_content_dir = PathJoinSubstitution(
        [FindPackageShare("titati_bringup"), "urdf", "titati.urdf.xacro"]
    )
    xacro_executable = FindExecutable(name="xacro")
    urdf_output_path = '/tmp/titati_controller/titati_robot_description.urdf'
    urdf_output_dir = os.path.dirname(urdf_output_path)
    os.makedirs(urdf_output_dir, exist_ok=True)
    save_urdf_process = ExecuteProcess(
        cmd=[
            xacro_executable, robot_description_content_dir,
            '>', urdf_output_path
        ],
        shell=True
    )  
    robot_description_content = Command(
        [
            PathJoinSubstitution([xacro_executable]),
            " ",
            robot_description_content_dir,
            " ",
            "use_webots:=",
            use_webots,
            " ",
            "ctrl_mode:=",
            ctrl_mode,
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    robot_controllers = os.path.join(
        get_package_share_directory("titati_controller"),
        "config",
        "titati_ros2_controllers.yaml",
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            robot_controllers,
            {"robot_description_path": urdf_output_path},
            {"frame_prefix": prefix+"/"},
        ],
        namespace=prefix, 
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        # output="both",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"frame_prefix": prefix+"/"},
        ],
        namespace=prefix, 
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", prefix+"/controller_manager"],
    )
    
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", prefix+"/controller_manager"],
    )
    tita_controller_spawner = Node(
        package='controller_manager',
        # output='screen',
        executable='spawner',
        arguments=["titati_controller", "--controller-manager", prefix+"/controller_manager"], #TODO: change: cheater_tita_control
        )   
    
    # start_controller = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=imu_sensor_broadcaster_spawner,
    #         on_exit=[tita_controller_spawner],
    #     )
    # )
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

    command_launch = return_launch_file('joy_controller','launch','joy_controller.launch.py')
    
    nodes = [
        save_urdf_process,
        # command_node,
        control_node,
        robot_state_pub_node,
        # joint_state_broadcaster_spawner,
        # imu_sensor_broadcaster_spawner,
        battery_device_node,
        canfd_router_node,
        # start_controller,
        tita_controller_spawner,
        command_launch,
    ]

    return LaunchDescription(declared_arguments + nodes)
