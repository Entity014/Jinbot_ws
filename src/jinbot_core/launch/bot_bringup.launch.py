import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("jinbot_core"), "config", "params.yaml"
    )

    node_microros = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB0"],
    )
    node_drive = Node(
        package="jinbot_core", executable="drive_node", parameters=[config]
    )
    node_flag = Node(package="jinbot_core", executable="flag_node", parameters=[config])
    node_joy = Node(package="joy", executable="joy_node")
    node_joyd = Node(package="jinbot_core", executable="joy_node")

    ld.add_action(node_microros)
    ld.add_action(node_joy)
    ld.add_action(node_joyd)
    ld.add_action(node_drive)
    ld.add_action(node_flag)

    return ld
