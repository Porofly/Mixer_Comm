"""Bring up one mixer_serial_node per dongle listed in dongles.yaml.

Usage:
    ros2 launch mixer_comm mixer_bridge.launch.py
    ros2 launch mixer_comm mixer_bridge.launch.py config:=/path/to/custom.yaml

Each dongle entry produces a node under namespace /mixer/node<id> that opens
the by-id device path for the matching USB serial. A missing dongle (serial
not present in /dev/serial/by-id/) fails fast with a clear error.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BY_ID_PREFIX = "/dev/serial/by-id/usb-NES_Lab___Mixer_Mixer_Console__PCA10059__"
BY_ID_SUFFIX = "-if00"


def _spawn(context, *_args, **_kwargs):
    config_path = Path(LaunchConfiguration("config").perform(context))
    if not config_path.is_file():
        raise FileNotFoundError(f"dongles config not found: {config_path}")

    with config_path.open() as f:
        cfg = yaml.safe_load(f) or {}

    entries = cfg.get("dongles", [])
    if not entries:
        raise ValueError(f"{config_path}: no `dongles` entries")

    nodes = []
    seen_ids = set()
    for entry in entries:
        node_id = int(entry["node_id"])
        serial = str(entry["serial"]).strip()
        if node_id in seen_ids:
            raise ValueError(f"duplicate node_id {node_id} in {config_path}")
        seen_ids.add(node_id)

        device = f"{BY_ID_PREFIX}{serial}{BY_ID_SUFFIX}"
        if not Path(device).exists():
            raise FileNotFoundError(
                f"dongle for node {node_id} not found at {device} -- "
                f"is the dongle with serial {serial} plugged in?"
            )

        nodes.append(
            Node(
                package="mixer_comm",
                executable="mixer_serial_node",
                namespace=f"/mixer/node{node_id}",
                name="mixer_serial_node",
                output="screen",
                parameters=[
                    {"device": device},
                    {"baud": 115200},
                    {"frame_id": f"mixer_node{node_id}"},
                ],
            )
        )

    return nodes


def generate_launch_description():
    default_config = (
        Path(get_package_share_directory("mixer_comm")) / "config" / "dongles.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=str(default_config)),
            OpaqueFunction(function=_spawn),
        ]
    )
