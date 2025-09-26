import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_namespace = os.environ.get("TITATI_NAMESPACE", "titati")
    config = os.path.join(
        get_package_share_directory('battery_device'),
        'config',
        'param.yaml'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=default_namespace,
        description='Namespace for Titati topics'
    )

    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        namespace_arg,
        Node(
            package='battery_device',
            executable='battery_device_node',
            name='battery_device_node',
            namespace=namespace,
            output='screen',
            parameters=[config]
        )
    ])
