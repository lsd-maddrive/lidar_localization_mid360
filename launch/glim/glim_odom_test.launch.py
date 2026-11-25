"""Raw GLIM odometry for manually driven out-and-back tests."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('glim_config', default_value='/home/rei/forklift_ws/runtime/glim_localization_mid360'),
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_lidar_publisher', output='screen', parameters=[{
                 'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
                 'publish_freq': 10.0, 'frame_id': 'livox_frame',
                 'cmdline_input_bd_code': 'livox0000000001',
                 'user_config_path': PathJoinSubstitution([
                     FindPackageShare('lidar_localization_mid360'), 'config', 'MID360_config.json']),
             }]),
        Node(package='livox_to_pointcloud2', executable='livox_to_pointcloud2_node', output='screen'),
        Node(package='glim_ros', executable='glim_rosnode', name='glim_ros', output='screen',
             parameters=[{'config_path': LaunchConfiguration('glim_config'),
                          'localization': False, 'use_sim_time': False}],
             remappings=[('/os_cloud_node/points', '/livox/points'),
                         ('/os_cloud_node/imu', '/livox/imu'),
                         ('/livox/lidar', '/livox/points')]),
    ])
