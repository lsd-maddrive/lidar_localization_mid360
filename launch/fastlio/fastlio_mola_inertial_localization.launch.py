"""Experimental localization with a slow, windowed map->odom correction."""

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
        DeclareLaunchArgument(
            "window_seconds", default_value="3.0",
            description="Trailing time window used to average map->odom",
        ),

        # MOLA still provides /lidar_odometry/pose. This node is the sole
        # publisher of map->odom; Fast-LIO remains the odom->lidar publisher.
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
            package="lidar_localization_mid360",
            executable="map_odom_smoother.py",
            name="map_odom_smoother",
            output="screen",
            parameters=[{"window_seconds": LaunchConfiguration("window_seconds")}],
        ),
    ])
