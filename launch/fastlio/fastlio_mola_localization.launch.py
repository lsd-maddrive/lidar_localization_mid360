from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Run one MID-360 driver, FAST-LIO odometry, and MOLA localization."""
    driver_config = LaunchConfiguration("driver_config")
    fastlio_config = LaunchConfiguration("fastlio_config")
    map_file = LaunchConfiguration("map_file")
    initial_pose = LaunchConfiguration("initial_pose")
    use_rviz = LaunchConfiguration("use_rviz")
    diagnostics = LaunchConfiguration("diagnostics")
    use_state_estimator = LaunchConfiguration("use_state_estimator")

    mola_launch = PathJoinSubstitution([
        FindPackageShare("lidar_localization_mid360"), "launch", "mola", "mola_localization_diagnostics.launch.py"
    ])

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
            description="FAST-LIO parameter YAML; also supplies IMU-to-LiDAR extrinsics to the adapter",
        ),
        DeclareLaunchArgument("map_file", description="Absolute path to the MOLA .mm metric map"),
        DeclareLaunchArgument("initial_pose", default_value="[0, 0, 0, 0, 0, 0]"),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        DeclareLaunchArgument(
            "diagnostics", default_value="False",
            description="Publish both raw and smoothed MOLA poses without changing TF",
        ),
        DeclareLaunchArgument(
            "use_state_estimator", default_value="False",
            description="Enable optional GTSAM state smoother (disabled by default)",
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
            parameters=[fastlio_config, {"publish.tf_en": False}],
        ),
        Node(
            package="livox_to_pointcloud2",
            executable="livox_to_pointcloud2_node",
            name="livox_to_pointcloud2",
            output="screen",
        ),
        Node(
            package="lidar_localization_mid360",
            executable="fastlio_odometry_to_livox.py",
            name="fastlio_odometry_to_livox",
            output="screen",
            parameters=[fastlio_config, {
                "odom_frame": "odom",
                "lidar_frame": "livox_frame",
                "publish_tf": True,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mola_launch),
            launch_arguments={
                "diagnostics": diagnostics,
                "lidar_topic_name": "/livox/points",
                "imu_topic_name": "/livox/imu",
                "mola_tf_base_link": "livox_frame",
                "ignore_lidar_pose_from_tf": "True",
                "ignore_imu_pose_from_tf": "True",
                "use_imu_for_lio": "True",
                "use_state_estimator": use_state_estimator,
                "odom_topic_name": "/fastlio/odometry_livox",
                "odom_sensor_label": "fastlio",
                "forward_ros_tf_odom_to_mola": "False",
                "initial_localization_method": "InitLocalization::FixedPose",
                "initial_pose": initial_pose,
                "mola_initial_map_mm_file": map_file,
                "mola_bridge_odometry_frame": "odom",
                "mola_state_estimator_reference_frame": "map",
                "publish_localization_following_rep105": "True",
                "start_mapping_enabled": "False",
                "start_active": "True",
                "generate_simplemap": "False",
                "use_rviz": use_rviz,
                "use_mola_gui": "False",
            }.items(),
        ),
    ])
