from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="titati_motor_tools",
            executable="motor_test_node",
            output="screen",
            parameters=[{
                "joint_index": 0,
                "mode": "torque",
                "torque": 0.0,
                "kp": 40.0,
                "kd": 2.0,
                "position": 0.0,
                "velocity": 0.0,
                "duration": 5.0,
            }],
        )
    ])
