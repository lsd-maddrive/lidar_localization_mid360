"""Wrap the installed MOLA launch, changing only its odometry output filter."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions


ODOMETRY_OUTPUT_FILTER = "MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS_SOURCE"
# This is a YAML scalar fragment, not the final filter string. The installed
# template substitutes it into an unquoted value. A zero-length environment
# value becomes YAML null and breaks MRPT's node serialization/reparse.
EMPTY_YAML_STRING = '""'


def with_diagnostics(upstream):
    """Insert the override immediately before upstream's deferred node group.

    Setting the variable before including MOLA is insufficient: its launch
    overwrites it. Fail explicitly if a future upstream changes this layout.
    No TF filter, subscriptions, or estimator parameters are modified here.
    """
    actions = list(upstream.entities)
    context = LaunchContext()
    if (len(actions) < 2 or not isinstance(actions[-1], OpaqueFunction)
            or not isinstance(actions[-2], SetEnvironmentVariable)
            or perform_substitutions(context, actions[-2].name) != ODOMETRY_OUTPUT_FILTER):
        raise RuntimeError(
            "MOLA launch layout changed: expected its odometry output filter "
            "immediately before the deferred node group. Update the local "
            "diagnostic wrapper; system ROS files have not been modified."
        )
    return LaunchDescription([
        DeclareLaunchArgument("diagnostics", default_value="False"),
        *actions[:-1],
        SetEnvironmentVariable(
            name=ODOMETRY_OUTPUT_FILTER, value=EMPTY_YAML_STRING,
            condition=IfCondition(LaunchConfiguration("diagnostics")),
        ),
        actions[-1],
    ])


def generate_launch_description():
    path = Path(get_package_share_directory("mola_lidar_odometry")) / (
        "ros2-launchs/ros2-lidar-odometry.launch.py"
    )
    spec = importlib.util.spec_from_file_location("leti_upstream_mola_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return with_diagnostics(module.generate_launch_description())
