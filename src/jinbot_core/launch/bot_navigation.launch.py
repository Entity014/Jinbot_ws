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

    config_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "config", "params.yaml"]
    )
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "config", "ekf.yaml"]
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "rviz", "mec_bot_navigation.rviz"]
    )
    default_map_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "maps", f"{MAP_NAME}.yaml"]
    )
    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare("jinbot_core"), "config", "navigation.yaml"]
    )

    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare("sllidar_ros2"), "launch", "sllidar_a3_launch.py"]
    )
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
    )

    launch_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("sim"),
            "params_file": nav2_config_path,
        }.items(),
    )
    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        launch_arguments={
            "params_file": config_path,
        }.items(),
    )

    node_microros1 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/esp32_1"],
    )
    node_microros2 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/esp32_2"],
    )
    node_microros3 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/esp32_3"],
    )
    node_microros4 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/esp32_4"],
    )
    node_microros5 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/esp32_5"],
    )
    node_drive = Node(
        package="jinbot_core", executable="drive_node", parameters=[config_path]
    )
    node_flag = Node(
        package="jinbot_core", executable="flag_node", parameters=[config_path]
    )
    node_state = Node(
        package="jinbot_core", executable="state_node", parameters=[config_path]
    )
    node_model_flag = Node(
        package="jinbot_core", executable="model_flag_node", parameters=[config_path]
    )
    node_joy = Node(package="joy", executable="joy_node")
    node_joyd = Node(package="jinbot_core", executable="joy_node")

    node_lidar_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[config_path],
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

    ld.add_action(node_microros1)
    ld.add_action(node_microros2)
    ld.add_action(node_microros3)
    # ld.add_action(node_microros4)
    # ld.add_action(node_microros5)
    ld.add_action(node_joy)
    ld.add_action(node_joyd)
    ld.add_action(node_drive)
    ld.add_action(node_flag)
    ld.add_action(node_state)
    ld.add_action(node_model_flag)
    ld.add_action(launch_lidar)
    ld.add_action(node_lidar_filter)
    ld.add_action(node_localization)
    ld.add_action(launch_nav)
    ld.add_action(node_rviz)

    return ld
