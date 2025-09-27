#!/usr/bin/env python3
"""Launch titati_power_services and titati_canfd_gateway together.

This keeps the Titati power handshake helpers in sync on a single command.
"""
import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _resolve_tita_namespace() -> str:
    """Derive the Titati namespace from the device tree serial number."""
    namespace = "tita"
    serial_path_candidates: List[str] = [
        "/proc/device-tree/serial-number",
        "/proc/device-tree/chosen/serial-number",
    ]

    for serial_path in serial_path_candidates:
        try:
            with open(serial_path, "r", encoding="utf-8") as file:
                unique_id = file.read().strip().replace("\x00", "")
                if len(unique_id) > 6:
                    namespace = f"tita{unique_id[6:]}"
                    break
        except FileNotFoundError:
            continue
        except OSError:
            continue
        except Exception:
            break

    return namespace


TITA_NAMESPACE = _resolve_tita_namespace()


def generate_launch_description() -> LaunchDescription:
    battery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("titati_power_services"),
                "launch",
                "titati_power_services_node.launch.py",
            )
        )
    )

    canfd_router = Node(
        package="titati_canfd_gateway",
        executable="titati_canfd_gateway_node",
        namespace=TITA_NAMESPACE,
        output="screen",
    )

    return LaunchDescription([
        battery_launch,
        canfd_router,
    ])
