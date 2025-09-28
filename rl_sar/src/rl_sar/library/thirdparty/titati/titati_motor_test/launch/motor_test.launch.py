from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    parameter_defaults = {
        "joint_index": ("0", int),
        "mode": ("torque", str),
        "torque": ("0.0", float),
        "kp": ("40.0", float),
        "kd": ("2.0", float),
        "position": ("0.0", float),
        "velocity": ("0.0", float),
        "duration": ("5.0", float),
        "command_rate": ("500.0", float),
        "status_rate": ("10.0", float),
        "command_delay": ("5.0", float),
        "num_motors": ("16", int),
        "monitor_only": ("false", bool),
    }

    declare_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=f"Parameter '{name}'")
        for name, (default, _value_type) in parameter_defaults.items()
    ]

    parameter_overrides = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, (_default, value_type) in parameter_defaults.items()
    }

    return LaunchDescription(
        declare_arguments
        + [
            Node(
                package="titati_motor_test",
                executable="motor_test_node",
                output="screen",
                parameters=[parameter_overrides],
            )
        ]
    )
