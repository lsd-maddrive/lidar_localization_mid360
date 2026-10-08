from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    driver_config = LaunchConfiguration("driver_config")
    glim_config = LaunchConfiguration("glim_config")
    map_file = LaunchConfiguration("map_file")
    initial_pose = LaunchConfiguration("initial_pose")
    rviz_config = LaunchConfiguration("rviz_config")
    window_seconds = LaunchConfiguration("window_seconds")
    max_candidate_step_m = LaunchConfiguration("max_candidate_step_m")
    max_candidate_step_deg = LaunchConfiguration("max_candidate_step_deg")
    outlier_confirmations = LaunchConfiguration("outlier_confirmations")
    mola_launch = PathJoinSubstitution([
        FindPackageShare("lidar_localization_mid360"), "launch", "mola", "mola_localization_diagnostics.launch.py"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("driver_config", default_value=PathJoinSubstitution([
        FindPackageShare("lidar_localization_mid360"), "config", "MID360_config.json"
        ])),
        DeclareLaunchArgument("glim_config", default_value=PathJoinSubstitution([
            EnvironmentVariable("FORKLIFT_WS", default_value="."), "runtime", "glim_localization_mid360"
        ])),
        DeclareLaunchArgument("map_file", description="Absolute path to MOLA .mm metric map"),
        DeclareLaunchArgument("initial_pose", default_value="[0, 0, 0, 0, 0, 0]"),
        # Keep this argument distinct from the nested MOLA launch's
        # ``use_rviz`` argument, which is intentionally set to False.
        DeclareLaunchArgument("use_rviz_viewer", default_value="True"),
        DeclareLaunchArgument("rviz_config", default_value=PathJoinSubstitution([
        FindPackageShare("lidar_localization_mid360"), "config", "lidar_odometry.rviz"
        ])),
        DeclareLaunchArgument("window_seconds", default_value="3.0"),
        DeclareLaunchArgument("max_candidate_step_m", default_value="0.75"),
        DeclareLaunchArgument("max_candidate_step_deg", default_value="20.0"),
        DeclareLaunchArgument("outlier_confirmations", default_value="4"),
        # MOLA publishes the raw map pose topic, while the dedicated smoother
        # below is the only publisher of map->odom.
        SetEnvironmentVariable(name="MOLA_LOCALIZATION_PUBLISH_TF", value="False"),
        # Keep MolaViz disabled: RViz2 is the only GUI for this launch.
        SetEnvironmentVariable(name="MOLA_WITH_GUI", value="False"),
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
            remappings=[
                ("/os_cloud_node/points", "/livox/points"),
                ("/os_cloud_node/imu", "/livox/imu"),
                ("/livox/lidar", "/livox/points"),
            ],
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
                "initial_localization_method": "InitLocalization::FixedPose",
                "initial_pose": initial_pose,
                "mola_initial_map_mm_file": map_file,
                "mola_bridge_odometry_frame": "odom",
                "mola_state_estimator_reference_frame": "map",
                # map_odom_smoother.py is the single map->odom TF publisher.
                "publish_localization_following_rep105": "False",
                "start_mapping_enabled": "False",
                "start_active": "True",
                "generate_simplemap": "False",
                # RViz is started below with the package-local configuration.
                "use_rviz": "False",
                "use_mola_gui": "False",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("use_rviz_viewer")),
        ),
        Node(
            package="lidar_localization_mid360",
            executable="map_odom_smoother.py",
            name="map_odom_smoother",
            output="screen",
            parameters=[{
                "pose_topic": "/lidar_odometry/pose",
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "livox_frame",
                "window_seconds": window_seconds,
                "max_candidate_step_m": max_candidate_step_m,
                "max_candidate_step_deg": max_candidate_step_deg,
                "outlier_confirmations": outlier_confirmations,
            }],
        ),
        Node(
            package="lidar_localization_mid360",
            executable="lidar_map_path_publisher.py",
            name="lidar_map_path_publisher",
            output="screen",
            parameters=[{
                "map_frame": "map",
                "lidar_frame": "livox_frame",
                "path_topic": "/lidar_path",
                "publish_period": 0.1,
                "min_distance": 0.03,
                "max_points": 20000,
            }],
        ),
        Node(
            package="lidar_localization_mid360",
            executable="lidar_map_path_publisher.py",
            name="lidar_odom_path_publisher",
            output="screen",
            parameters=[{
                "map_frame": "odom",
                "lidar_frame": "livox_frame",
                "path_topic": "/lidar_odom_path",
                "publish_period": 0.1,
                "min_distance": 0.03,
                "max_points": 20000,
            }],
        ),
    ])
