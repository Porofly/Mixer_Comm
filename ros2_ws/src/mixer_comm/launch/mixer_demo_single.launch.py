"""Single-dongle bring-up for one Jetson (or any host with one dongle).

Spawns the serial bridge + demo_pub + demo_sub for exactly one dongle. Used
when each Jetson runs its own Docker container with its own dongle, and ROS 2
DDS is intentionally NOT shared between hosts -- all peer traffic arrives over
the Mixer RF link, decoded locally by the dongle.

Required args:
    node_id   integer Mixer node id this dongle was flashed for (matches
              firmware's TOS_NODE_ID and the cyclic payload_distribution)
    serial    USB serial-number string of this dongle (the suffix of
              /dev/serial/by-id/usb-...PCA10059__<SERIAL>-if00)

Optional:
    slot              defaults to node_id - 1 (cyclic)
    pub_rate_hz       1.0
    report_period_s   2.0
    baud              115200

Usage:
    ros2 launch mixer_comm mixer_demo_single.launch.py \\
        node_id:=1 serial:=DONGLE_USB_SERIAL_HERE
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BY_ID_PREFIX = "/dev/serial/by-id/usb-NES_Lab___Mixer_Mixer_Console__PCA10059__"
BY_ID_SUFFIX = "-if00"


def _spawn(context, *_args, **_kwargs):
    node_id = int(LaunchConfiguration("node_id").perform(context))
    serial = LaunchConfiguration("serial").perform(context).strip()
    slot_str = LaunchConfiguration("slot").perform(context).strip()
    slot = int(slot_str) if slot_str else node_id - 1
    pub_rate_hz = float(LaunchConfiguration("pub_rate_hz").perform(context))
    report_period_s = float(LaunchConfiguration("report_period_s").perform(context))
    baud = int(LaunchConfiguration("baud").perform(context))

    if node_id < 1 or node_id > 255:
        raise ValueError(f"node_id must be in [1,255], got {node_id}")
    if not serial:
        raise ValueError("serial is required (USB serial of the dongle)")

    device = f"{BY_ID_PREFIX}{serial}{BY_ID_SUFFIX}"
    if not Path(device).exists():
        raise FileNotFoundError(
            f"dongle for node {node_id} not found at {device} -- "
            f"is the dongle with serial {serial} plugged in?"
        )

    ns = f"/mixer/node{node_id}"
    return [
        Node(
            package="mixer_comm",
            executable="mixer_serial_node",
            namespace=ns,
            name="mixer_serial_node",
            output="screen",
            parameters=[
                {"device": device},
                {"baud": baud},
                {"frame_id": f"mixer_node{node_id}"},
            ],
        ),
        Node(
            package="mixer_comm",
            executable="mixer_demo_pub",
            namespace=ns,
            name="mixer_demo_pub",
            output="screen",
            parameters=[
                {"node_id": node_id},
                {"slot": slot},
                {"rate_hz": pub_rate_hz},
            ],
        ),
        Node(
            package="mixer_comm",
            executable="mixer_demo_sub",
            namespace=ns,
            name="mixer_demo_sub",
            output="screen",
            parameters=[
                {"node_id": node_id},
                {"report_period_s": report_period_s},
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("node_id"),
            DeclareLaunchArgument("serial"),
            DeclareLaunchArgument("slot", default_value=""),
            DeclareLaunchArgument("pub_rate_hz", default_value="1.0"),
            DeclareLaunchArgument("report_period_s", default_value="2.0"),
            DeclareLaunchArgument("baud", default_value="115200"),
            OpaqueFunction(function=_spawn),
        ]
    )
