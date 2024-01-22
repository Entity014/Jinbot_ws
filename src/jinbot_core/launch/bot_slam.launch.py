import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "config", "ekf.yaml"]
    )
    slam_config_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "config", "slam.yaml"]
    )
    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "config", "navigation.yaml"]
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "rviz", "jinbot_slam.rviz"]
    )
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "launch", "bot_description.launch.py"]
    )

    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
    )

    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
    )

    launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path)
    )
    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            "slam_params_file": slam_config_path,
        }.items(),
    )
    launch_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
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
    ld.add_action(launch_description)
    ld.add_action(node_localization)
    ld.add_action(launch_nav)
    ld.add_action(launch_slam)
    ld.add_action(node_rviz)

    return ld
