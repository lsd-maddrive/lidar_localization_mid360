#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    driver_config = LaunchConfiguration("driver_config")
    glim_config = LaunchConfiguration("glim_config")
    map_path = LaunchConfiguration("map_path")

    driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_mid360",
        output="screen",
        parameters=[{
            "xfer_format": 0,
            "multi_topic": 0,
            "data_src": 0,
            "publish_freq": 10.0,
            "output_data_type": 0,
            "frame_id": "livox_frame",
            "user_config_path": driver_config,
            "cmdline_input_bd_code": "livox0000000001",
        }],
    )

    glim = Node(
        package="glim_ros",
        executable="glim_rosnode",
        name="glim_ros",
        output="screen",
        parameters=[{
            "config_path": glim_config,
            "localization": True,
            "map_path": map_path,
            "use_sim_time": False,
        }],
    )

    # The upstream fork exposes map loading as a Trigger service rather than
    # loading map_path automatically.  Call it after the node is initialized.
    load_map = ExecuteProcess(
        cmd=[
            "ros2", "service", "call", "/glim_ros/load_map",
            "std_srvs/srv/Trigger", "{}",
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "driver_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("lidar_localization_mid360"), "config", "MID360_config.json"
            ]),
        ),
        DeclareLaunchArgument(
            "glim_config",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("FORKLIFT_WS", default_value="."), "runtime", "glim_localization_mid360"
            ]),
        ),
        DeclareLaunchArgument(
            "map_path",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("FORKLIFT_WS", default_value="."), "maps", "test7"
            ]),
        ),
        driver,
        TimerAction(period=5.0, actions=[glim]),
        TimerAction(period=10.0, actions=[load_map]),
    ])
