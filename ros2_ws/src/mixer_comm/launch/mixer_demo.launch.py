"""Bring up the Mixer bridge and a demo publisher/subscriber pair per dongle.

For every dongle listed in dongles.yaml this launch file spawns:
- mixer_demo_pub under /mixer/node<id>, publishing 1 Hz to /mixer/node<id>/mixer/tx
- mixer_demo_sub under /mixer/node<id>, listening to *every other* node's
  /mixer/node<other>/mixer/rx topic so each sub reports what its dongle sees
  of the other dongles' traffic.

The publisher's slot defaults to (node_id - 1) which matches the cyclic
payload_distribution used by the firmware (slot 0 -> node 1, slot 1 -> node 2,
...). Override via `pub_slot_offset`/`pub_rate_hz`/`report_period_s` launch args.

Usage:
    ros2 launch mixer_comm mixer_demo.launch.py
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_demos(context, *_args, **_kwargs):
    config_path = Path(LaunchConfiguration("config").perform(context))
    pub_rate_hz = float(LaunchConfiguration("pub_rate_hz").perform(context))
    report_period_s = float(LaunchConfiguration("report_period_s").perform(context))

    with config_path.open() as f:
        cfg = yaml.safe_load(f) or {}
    entries = cfg.get("dongles", [])
    if not entries:
        raise ValueError(f"{config_path}: no dongles to demo")

    node_ids = [int(e["node_id"]) for e in entries]
    actions = []
    for nid in node_ids:
        slot = nid - 1  # matches cyclic payload_distribution slot 0->1, 1->2, ...
        actions.append(
            Node(
                package="mixer_comm",
                executable="mixer_demo_pub",
                namespace=f"/mixer/node{nid}",
                name="mixer_demo_pub",
                output="screen",
                parameters=[
                    {"node_id": nid},
                    {"slot": slot},
                    {"rate_hz": pub_rate_hz},
                ],
            )
        )
        # Each demo_sub listens to every other dongle's RX topic. Listening to
        # its own RX too would also work (each dongle decodes its own slot) but
        # would clutter the report with self-loops.
        peer_topics = [
            f"/mixer/node{other}/mixer/rx" for other in node_ids if other != nid
        ]
        actions.append(
            Node(
                package="mixer_comm",
                executable="mixer_demo_sub",
                namespace=f"/mixer/node{nid}",
                name="mixer_demo_sub",
                output="screen",
                parameters=[
                    {"listen_topics": peer_topics},
                    {"report_period_s": report_period_s},
                ],
            )
        )
    return actions


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("mixer_comm"))
    default_config = pkg_share / "config" / "dongles.yaml"
    bridge_launch = pkg_share / "launch" / "mixer_bridge.launch.py"

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=str(default_config)),
            DeclareLaunchArgument("pub_rate_hz", default_value="1.0"),
            DeclareLaunchArgument("report_period_s", default_value="2.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bridge_launch)),
                launch_arguments={"config": LaunchConfiguration("config")}.items(),
            ),
            OpaqueFunction(function=_spawn_demos),
        ]
    )
