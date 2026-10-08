from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")
    use_rviz = LaunchConfiguration("use_rviz")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=PathJoinSubstitution([
                FindPackageShare("lidar_localization_mid360"), "config", "MID360_config.json"
            ]),
            description="Mid-360 JSON network configuration"),
        DeclareLaunchArgument("use_rviz", default_value="False"),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[{
                "xfer_format": 0,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "user_config_path": config,
                "cmdline_input_bd_code": "livox0000000001",
            }]),
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="mid360_rviz",
            output="screen"),
    ])
