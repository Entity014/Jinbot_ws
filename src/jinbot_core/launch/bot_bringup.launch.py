import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    node_config_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "config", "params.yaml"]
    )

    lidar_config_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "config", "lidar.yaml"]
    )

    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare("sllidar_ros2"), "launch", "sllidar_a3_launch.py"]
    )

    node_microros1 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB0"],
    )
    node_microros2 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB1"],
    )
    node_microros3 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB2"],
    )
    node_microros4 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB3"],
    )
    node_microros5 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB4"],
    )
    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        launch_arguments={
            "params_file": lidar_config_path,
        }.items(),
    )
    node_drive = Node(
        package="jinbot_core", executable="drive_node", parameters=[node_config_path]
    )
    node_flag = Node(
        package="jinbot_core", executable="flag_node", parameters=[node_config_path]
    )
    node_state = Node(
        package="jinbot_core", executable="state_node", parameters=[node_config_path]
    )
    node_joy = Node(package="joy", executable="joy_node")
    node_joyd = Node(package="jinbot_core", executable="joy_node")

    ld.add_action(node_microros1)
    # ld.add_action(node_microros2)
    # ld.add_action(node_microros3)
    # ld.add_action(node_microros4)
    # ld.add_action(node_microros5)
    ld.add_action(node_joy)
    ld.add_action(node_joyd)
    ld.add_action(node_drive)
    ld.add_action(node_flag)
    ld.add_action(node_state)
    # ld.add_action(launch_lidar)

    return ld
