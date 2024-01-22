import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

MAP_NAME = "jinpao_blue2"


def generate_launch_description():
    ld = LaunchDescription()

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "config", "ekf.yaml"]
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "rviz", "jinbot_navigation.rviz"]
    )
    default_map_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "maps", f"{MAP_NAME}.yaml"]
    )
    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "config", "navigation.yaml"]
    )

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
    )

    launch_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            "map": default_map_path,
            "use_sim_time": "false",
            "params_file": nav2_config_path,
        }.items(),
    )

    node_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path],
        remappings=[("odometry/filtered", "odom")],
    )
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    ld.add_action(node_localization)
    ld.add_action(launch_nav)
    ld.add_action(node_rviz)

    return ld
