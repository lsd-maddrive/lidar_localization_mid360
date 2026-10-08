from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Record a MOLA SimpleMap using MID-360 data and FAST-LIO odometry."""
    driver_config = LaunchConfiguration("driver_config")
    fastlio_config = LaunchConfiguration("fastlio_config")
    output = PathJoinSubstitution([
        LaunchConfiguration("workspace_root"), LaunchConfiguration("map_name")
    ])
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
            description="FAST-LIO parameter YAML",
        ),
        DeclareLaunchArgument("workspace_root", default_value=PathJoinSubstitution([
            EnvironmentVariable("FORKLIFT_WS", default_value="."), "maps"
        ])),
        DeclareLaunchArgument("map_name", default_value="fastlio_map"),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        SetEnvironmentVariable(name="MOLA_SIMPLEMAP_OUTPUT", value=output),
        SetEnvironmentVariable(name="MOLA_SIMPLEMAP_GENERATE_LAZY_LOAD", value="True"),
        SetEnvironmentVariable(name="MOLA_SIMPLEMAP_ALSO_NON_KEYFRAMES", value="True"),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[{
                # FAST-LIO's Livox preprocessing path requires CustomMsg.
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
                "lidar_topic_name": "/livox/points",
                "imu_topic_name": "/livox/imu",
                "mola_tf_base_link": "livox_frame",
                "ignore_lidar_pose_from_tf": "True",
                "ignore_imu_pose_from_tf": "True",
                "use_imu_for_lio": "True",
                "odom_topic_name": "/fastlio/odometry_livox",
                "odom_sensor_label": "fastlio",
                "forward_ros_tf_odom_to_mola": "False",
                "initial_localization_method": "InitLocalization::PitchAndRollFromIMU",
                "publish_localization_following_rep105": "False",
                "start_mapping_enabled": "True",
                "start_active": "True",
                "generate_simplemap": "True",
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_mola_gui": "False",
            }.items(),
        ),
    ])
