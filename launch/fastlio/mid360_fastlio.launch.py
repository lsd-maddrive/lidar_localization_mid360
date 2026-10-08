from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start a Livox MID-360 in CustomMsg mode together with FAST-LIO.

    FAST-LIO's ``mid360.yaml`` uses ``preprocess.lidar_type: 1`` and therefore
    requires the Livox CustomMsg output, not the PointCloud2 output used by
    the MOLA-only driver launch.
    """
    driver_config = LaunchConfiguration("driver_config")
    fastlio_config = LaunchConfiguration("fastlio_config")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "driver_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("lidar_localization_mid360"), "config", "MID360_config.json"
            ]),
            description="MID-360 network JSON configuration",
        ),
        DeclareLaunchArgument(
            "fastlio_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("fast_lio"), "config", "mid360.yaml"
            ]),
            description="FAST-LIO parameter YAML",
        ),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("fast_lio"), "rviz", "fastlio.rviz"
            ]),
            description="RViz configuration for FAST-LIO",
        ),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[{
                # FAST-LIO's Livox preprocessing path consumes CustomMsg.
                "xfer_format": 1,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "frame_id": "livox_frame",
                "user_config_path": driver_config,
                "cmdline_input_bd_code": "livox0000000001",
            }],
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping",
            output="screen",
            parameters=[fastlio_config],
        ),
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="fastlio_rviz",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
