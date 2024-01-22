import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    config_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "config", "config.yaml"]
    )

    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare("sllidar_ros2"), "launch", "sllidar_a3_launch.py"]
    )

    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        launch_arguments={"params_file": config_path}.items(),
    )

    node_microros1 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/esp32_0"],
    )
    node_microros2 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/esp32_1"],
    )
    node_microros3 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/esp32_2"],
    )
    node_microros4 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/esp32_3"],
    )
    node_microros5 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/esp32_4"],
    )
    node_microros6 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/esp32_5"],
    )
    node_lidar_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[config_path],
    )

    node_imu = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=[{"use_mag": False}],
    )

    ld.add_action(node_microros1)
    ld.add_action(node_microros2)
    ld.add_action(node_microros3)
    ld.add_action(node_microros4)
    ld.add_action(node_microros5)
    # ld.add_action(node_microros6)
    ld.add_action(node_imu)
    ld.add_action(launch_lidar)
    ld.add_action(node_lidar_filter)

    return ld
