from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="titati_motor_test",
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
                "command_rate": 500.0,
                "status_rate": 10.0,
                "command_delay": 5.0,
                "num_motors": 16,
                "monitor_only": False,
            }],
        )
    ])
