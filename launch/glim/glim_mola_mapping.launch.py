from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    driver_config = LaunchConfiguration("driver_config")
    glim_config = LaunchConfiguration("glim_config")
    workspace_root = LaunchConfiguration("workspace_root")
    map_name = LaunchConfiguration("map_name")
    output = PathJoinSubstitution([workspace_root, map_name])
    mola_launch = PathJoinSubstitution([
        FindPackageShare("lidar_localization_mid360"), "launch", "mola", "mola_localization_diagnostics.launch.py"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("driver_config", default_value=PathJoinSubstitution([
        FindPackageShare("lidar_localization_mid360"), "config", "MID360_config.json"
        ])),
        DeclareLaunchArgument("glim_config", default_value=PathJoinSubstitution([
            EnvironmentVariable("FORKLIFT_WS", default_value="."), "runtime", "glim_odom_mid360"
        ])),
        DeclareLaunchArgument("workspace_root", default_value=PathJoinSubstitution([
            EnvironmentVariable("FORKLIFT_WS", default_value="."), "maps"
        ])),
        DeclareLaunchArgument("map_name", default_value="glim_mola_map"),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        SetEnvironmentVariable(name="MOLA_SIMPLEMAP_OUTPUT", value=output),
        SetEnvironmentVariable(name="MOLA_SIMPLEMAP_GENERATE_LAZY_LOAD", value="True"),
        SetEnvironmentVariable(name="MOLA_SIMPLEMAP_ALSO_NON_KEYFRAMES", value="True"),
        Node(
            package="livox_ros_driver2", executable="livox_ros_driver2_node",
            name="livox_lidar_publisher", output="screen",
            parameters=[{
                "xfer_format": 1, "multi_topic": 0, "data_src": 0,
                "publish_freq": 10.0, "frame_id": "livox_frame",
                "user_config_path": driver_config,
                "cmdline_input_bd_code": "livox0000000001",
            }],
        ),
        Node(
            package="glim_ros", executable="glim_rosnode", name="glim_ros", output="screen",
            parameters=[{"config_path": glim_config, "localization": False, "use_sim_time": False}],
        ),
        Node(
            package="livox_to_pointcloud2", executable="livox_to_pointcloud2_node",
            name="livox_to_pointcloud2", output="screen",
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
                "odom_topic_name": "/glim_ros/odom",
                "odom_sensor_label": "glim",
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
