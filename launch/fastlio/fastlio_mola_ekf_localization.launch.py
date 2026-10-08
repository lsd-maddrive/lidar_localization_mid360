"""Experimental MOLA localization with robot_localization owning map->odom."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("lidar_localization_mid360")
    localization_launch = PathJoinSubstitution([
        bringup_share, "launch", "fastlio", "fastlio_mola_localization.launch.py"
    ])
    ekf_config = PathJoinSubstitution([bringup_share, "config", "map_odom_ekf.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_file",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("FORKLIFT_WS", default_value="."), "maps", "1122map4_lc.mm"
            ]),
            description="Absolute path to the MOLA metric map",
        ),
        DeclareLaunchArgument("initial_pose", default_value="[0, 0, 0, 0, 0, 0]"),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        DeclareLaunchArgument("diagnostics", default_value="False"),

        # MOLA still publishes its pose topic, but only the EKF may publish
        # map->odom. The Fast-LIO adapter continues publishing odom->livox_frame.
        SetEnvironmentVariable(name="MOLA_LOCALIZATION_PUBLISH_TF", value="False"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "map_file": LaunchConfiguration("map_file"),
                "initial_pose": LaunchConfiguration("initial_pose"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "diagnostics": LaunchConfiguration("diagnostics"),
                "use_state_estimator": "False",
            }.items(),
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="map_odom_ekf",
            output="screen",
            parameters=[ekf_config],
        ),
    ])
