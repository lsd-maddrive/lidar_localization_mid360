from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mola_launch = PathJoinSubstitution([
        FindPackageShare("mola_lidar_odometry"), "ros2-launchs", "ros2-lidar-odometry.launch.py"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("map_file", description="Absolute path to the .mm map"),
        DeclareLaunchArgument("odom_topic", default_value="/wheel/odometry"),
        DeclareLaunchArgument("odom_sensor_label", default_value="odom_external"),
        DeclareLaunchArgument("initial_pose", default_value="[0, 0, 0, 0, 0, 0]"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mola_launch),
            launch_arguments={
                "lidar_topic_name": "/livox/lidar",
                "imu_topic_name": "/livox/imu",
                "mola_tf_base_link": "livox_frame",
                "ignore_lidar_pose_from_tf": "True",
                "ignore_imu_pose_from_tf": "True",
                "use_imu_for_lio": "True",
                "use_state_estimator": "True",
                "odom_topic_name": LaunchConfiguration("odom_topic"),
                "odom_sensor_label": LaunchConfiguration("odom_sensor_label"),
                "forward_ros_tf_odom_to_mola": "False",
                "initial_localization_method": "InitLocalization::FixedPose",
                "initial_pose": LaunchConfiguration("initial_pose"),
                "mola_initial_map_mm_file": LaunchConfiguration("map_file"),
                "publish_localization_following_rep105": "False",
                "start_mapping_enabled": "False",
                "start_active": "True",
                "generate_simplemap": "False",
                "use_rviz": "True",
                "use_mola_gui": "False",
            }.items())
    ])
