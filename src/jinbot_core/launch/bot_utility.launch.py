import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    function_config_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "config", "params.yaml"]
    )
    node_flag = Node(
        package="jinbot_core", executable="flag_node", parameters=[function_config_path]
    )
    node_state = Node(
        package="jinbot_core",
        executable="state_node",
        parameters=[function_config_path],
    )
    node_model_flag = Node(
        package="jinbot_core",
        executable="model_flag_node",
        parameters=[function_config_path],
    )
    node_color_flag = Node(package="jinbot_core", executable="color_flag_node")
    node_joy = Node(package="joy", executable="joy_node")
    node_joyd = Node(package="jinbot_core", executable="joy_node")
    node_drive = Node(
        package="jinbot_core",
        executable="drive_node",
        parameters=[function_config_path],
    )
    node_slope = Node(package="jinbot_core", executable="slope_node")
    node_nav2 = Node(package="jinbot_core", executable="navigation_node")

    # ld.add_action(node_joy)
    # ld.add_action(node_joyd)
    # ld.add_action(node_drive)
    ld.add_action(node_slope)
    ld.add_action(node_state)
    ld.add_action(node_nav2)
    # ld.add_action(node_flag)
    # ld.add_action(node_model_flag)
    # ld.add_action(node_color_flag)

    return ld
