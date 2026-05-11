from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration("device")
    baud = LaunchConfiguration("baud")
    publish_raw = LaunchConfiguration("publish_raw")

    return LaunchDescription(
        [
            DeclareLaunchArgument("device", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("publish_raw", default_value="true"),
            Node(
                package="mixer_comm",
                executable="mixer_serial_node",
                name="mixer_serial_node",
                output="screen",
                parameters=[
                    {"device": device},
                    {"baud": baud},
                    {"publish_raw": publish_raw},
                ],
            ),
        ]
    )
