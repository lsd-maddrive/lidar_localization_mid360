from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    root = LaunchConfiguration("workspace_root")
    name = LaunchConfiguration("map_name")
    output = PathJoinSubstitution([root, name])
    mola_launch = PathJoinSubstitution([
        FindPackageShare("mola_lidar_odometry"), "ros2-launchs", "ros2-lidar-odometry.launch.py"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("workspace_root", default_value=PathJoinSubstitution([
            EnvironmentVariable("FORKLIFT_WS", default_value="."), "maps"
        ])),
        DeclareLaunchArgument("map_name", default_value="mid360_map"),
        SetEnvironmentVariable(name="MOLA_SIMPLEMAP_OUTPUT", value=output),
        SetEnvironmentVariable(name="MOLA_SIMPLEMAP_GENERATE_LAZY_LOAD", value="True"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mola_launch),
            launch_arguments={
                "lidar_topic_name": "/livox/lidar",
                "imu_topic_name": "/livox/imu",
                "mola_tf_base_link": "livox_frame",
                "ignore_lidar_pose_from_tf": "True",
                "ignore_imu_pose_from_tf": "True",
                "use_imu_for_lio": "True",
                "initial_localization_method": "InitLocalization::PitchAndRollFromIMU",
                "publish_localization_following_rep105": "False",
                "enforce_planar_motion": "False",
                "start_mapping_enabled": "True",
                "start_active": "True",
                "generate_simplemap": "True",
                "use_rviz": "True",
                "use_mola_gui": "False",
            }.items())
    ])
