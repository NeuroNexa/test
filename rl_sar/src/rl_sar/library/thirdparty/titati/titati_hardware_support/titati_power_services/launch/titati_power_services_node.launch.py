import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def resolve_tita_namespace() -> str:
    """Derive the Titati namespace from the device tree serial number."""
    namespace = "tita"
    serial_path_candidates = (
        "/proc/device-tree/serial-number",
        "/proc/device-tree/chosen/serial-number",
    )

    for serial_path in serial_path_candidates:
        try:
            with open(serial_path, "r", encoding="utf-8") as file:
                unique_id = file.read().strip().replace("\x00", "")
                # The vendor utilities drop the first six characters when deriving
                # the namespace, preserving that behaviour keeps topic names stable.
                if len(unique_id) > 6:
                    namespace = f"tita{unique_id[6:]}"
                    break
        except (FileNotFoundError, OSError):
            continue
        except Exception:
            break

    return namespace


TITA_NAMESPACE = resolve_tita_namespace()


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('titati_power_services'),
        'config',
        'param.yaml'
    )

    return LaunchDescription([
        Node(
            package='titati_power_services',
            executable='titati_power_services_node',
            name='titati_power_services_node',
            namespace=TITA_NAMESPACE,
            output='screen',
            parameters=[config]
        )
    ])
