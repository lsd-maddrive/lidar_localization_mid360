"""Hardware-free regression tests for diagnostic launch and pose alignment."""

import importlib.util
import inspect
import math
import os
import time
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, SetEnvironmentVariable
from launch.utilities import perform_substitutions


ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


audit = load("leti_audit_test", ROOT / "scripts/fastlio_mola_tf_audit.py")
wrapper = load("leti_wrapper_test", ROOT / "launch/mola_localization_diagnostics.launch.py")
IDENTITY = [0., 0., 0., 0., 0., 0., 1.]


def row(stamp, pose=IDENTITY, frame="map", child="livox_frame"):
    return {"stamp": stamp, "pose": list(pose), "frame": frame, "child": child,
            "received_ros": stamp + .02}


def test_rotation_and_pose_composition():
    q = [0., 0., math.sin(math.pi/4), math.cos(math.pi/4)]
    p = [2., 3., 1., *q]
    assert audit.compose(p, [1., 0., 0., 0., 0., 0., 1.])[:3] == pytest.approx([2., 4., 1.])
    assert audit.compose(p, audit.inverse(p)) == pytest.approx(IDENTITY, abs=1e-12)


def test_short_arc_slerp_and_antipodal_quaternions():
    end = [2., 0., 0., 0., 0., 1., 0.]
    middle = audit.interpolate_pose(IDENTITY, end, .5)
    assert middle == pytest.approx([1., 0., 0., 0., 0., math.sqrt(.5), math.sqrt(.5)])
    antipodal = [0., 0., 0., 0., 0., 0., -1.]
    assert audit.interpolate_pose(IDENTITY, antipodal, .5) == pytest.approx(IDENTITY)
    assert audit.difference(IDENTITY, antipodal) == pytest.approx((0., 0.))


def test_timeline_no_extrapolation_gap_or_frame_crossing():
    timeline = audit.Timeline([row(1), row(0)])
    assert timeline.sample(-.01, 2) is None
    assert timeline.sample(1.01, 2) is None
    assert timeline.sample(.5, .3) is None
    assert timeline.sample(.5, 2)["pose"] == IDENTITY
    changed = audit.Timeline([row(0), row(1, frame="odom")])
    assert changed.sample(.5, 2) is None


def test_regressions_preserved_but_alignment_sorted():
    rows = [row(1), row(.9), row(1.1), row(1.1, [1., 0., 0., 0., 0., 0., 1.])]
    summary = audit.stream_summary(rows, .3, 12)
    assert summary["stamp_regressions"] == 1
    assert summary["duplicate_stamps"] == 1
    assert summary["position_jump_count"] == 1
    timeline = audit.Timeline(rows)
    assert timeline.stamps == [.9, 1, 1.1]
    assert timeline.sample(1.1, .3)["pose"][0] == 1


def test_invalid_quaternion_rejected():
    with pytest.raises(ValueError):
        audit.normalize([0., 0., 0., 0.])


def test_alignment_and_missing_streams():
    a = [row(.1), row(.2)]
    b = [row(0), row(.2)]
    comparison = audit.aligned_comparison(a, b, .3)
    assert comparison["matched"] == 2
    assert comparison["samples"][0]["position_difference_m"] == 0
    assert comparison["relative_motion_differences"][0]["rotation_difference_deg"] == 0
    mismatch = audit.aligned_comparison(a, [row(0, frame="odom"), row(.2, frame="odom")], .3)
    assert mismatch["matched"] == 0
    result = audit.analyze({"streams": {"/state_estimation/pose": a}})
    assert "/lidar_odometry/pose" in result["missing_pose_topics"]
    assert result["raw_vs_smoothed"]["matched"] == 0


def test_reconstruction_removes_robot_motion_not_map_correction():
    offset = [3., -1., .5, 0., 0., math.sin(.3), math.cos(.3)]
    odom = [row(i*.1, [i*.01, 0., 0., 0., 0., 0., 1.], frame="odom") for i in range(6)]
    source = [row(r["stamp"], audit.compose(offset, r["pose"])) for r in odom]
    result = audit.reconstructed_map_odom(source, odom, .3)
    assert result["skipped"] == 0
    for r in result["samples"]:
        assert r["pose"] == pytest.approx(offset)
    source[3]["pose"][0] += 1
    result = audit.reconstructed_map_odom(source, odom, .3)
    assert audit.stream_summary(result["samples"], .3, 12)["position_jump_count"] == 2


@pytest.mark.parametrize("diagnostics, expected", [("False", "state_estimation"), ("True", '""')])
def test_override_order_and_false_preserves_default(diagnostics, expected):
    before = SetEnvironmentVariable(name=wrapper.ODOMETRY_OUTPUT_FILTER, value="state_estimation")
    node_group = OpaqueFunction(function=lambda context: [])
    upstream = LaunchDescription([before, node_group])
    result = wrapper.with_diagnostics(upstream)
    assert result.entities[-1] is node_group
    context = LaunchContext()
    context.launch_configurations["diagnostics"] = diagnostics
    before.visit(context)
    result.entities[-2].visit(context)
    assert context.environment[wrapper.ODOMETRY_OUTPUT_FILTER] == expected


@pytest.mark.parametrize("diagnostics, expected", [("False", "state_estimation"), ("True", "")])
def test_installed_yaml_keeps_subscriptions_and_tf_filter(diagnostics, expected):
    """Validate substitutions with the same native parser used by MOLA."""
    executable = Path(os.environ.get("LETI_MOLA_YAML_PROBE", str(
        ROOT.parents[1] / "build/lidar_localization_mid360/mola_yaml_diagnostic_probe")))
    assert executable.is_file(), "Build lidar_localization_mid360 before running native YAML tests"
    context = LaunchContext()
    context.launch_configurations["diagnostics"] = diagnostics
    context.environment[wrapper.ODOMETRY_OUTPUT_FILTER] = "state_estimation"
    context.environment["MOLA_LOCALIZATION_PUBLISH_TF_SOURCE"] = "state_estimation"
    context.environment["MOLA_LIDAR_TOPIC"] = "/livox/points"
    context.environment["MOLA_IMU_TOPIC"] = "/livox/imu"
    context.environment["MOLA_ODOM_TOPIC"] = "/fastlio/odometry_livox"
    wrapper.generate_launch_description().entities[-2].visit(context)
    path = Path(get_package_share_directory("mola_lidar_odometry")) / (
        "mola-cli-launchs/lidar_odometry_ros2.yaml")
    system = yaml.safe_load(path.read_text())
    bridge = next(m for m in system["modules"] if m["name"] == "ros2_bridge")
    result = subprocess.run([str(executable)], input=yaml.safe_dump(bridge["params"]),
                            text=True, capture_output=True, env=dict(context.environment), timeout=10)
    assert result.returncode == 0, result.stderr
    assert "could not find expected" not in result.stderr
    parsed = yaml.safe_load(result.stdout)
    assert parsed["publish_odometry_msgs_from_slam_source"] == expected
    assert parsed["publish_tf_from_slam_source"] == "state_estimation"
    assert parsed["publish_tf_from_robot_pose_observations"] is False
    subscriptions = {s["topic"]: s["msg_type"] for s in parsed["subscribe"]}
    assert subscriptions["/livox/points"] == "PointCloud2"
    assert subscriptions["/livox/imu"] == "Imu"
    assert subscriptions["/fastlio/odometry_livox"] == "Odometry"


def test_upstream_change_is_not_silently_ignored():
    with pytest.raises(RuntimeError, match="layout changed"):
        wrapper.with_diagnostics(LaunchDescription([OpaqueFunction(function=lambda c: [])]))


def test_installed_mola_launch_is_compatible_without_starting_nodes():
    description = wrapper.generate_launch_description()
    assert isinstance(description.entities[-1], OpaqueFunction)
    context = LaunchContext()
    original_filter = description.entities[-3]
    assert perform_substitutions(context, original_filter.name) == wrapper.ODOMETRY_OUTPUT_FILTER
    # The original TF source assignment remains present, unmodified.
    tf_actions = [a for a in description.entities if isinstance(a, SetEnvironmentVariable)
                  and perform_substitutions(context, a.name) == "MOLA_LOCALIZATION_PUBLISH_TF_SOURCE"]
    assert len(tf_actions) == 1
    context.launch_configurations.update(use_state_estimator="True", localization_publish_tf_source="")
    tf_actions[0].visit(context)
    assert context.environment["MOLA_LOCALIZATION_PUBLISH_TF_SOURCE"] == "state_estimation"


def test_combined_launch_disables_optional_state_estimator_by_default():
    launch = load("leti_combined_launch_test", ROOT / "launch/fastlio_mola_localization.launch.py")
    description = launch.generate_launch_description()
    estimator_arg = next(
        action for action in description.entities
        if getattr(action, "name", None) == "use_state_estimator"
    )
    assert perform_substitutions(LaunchContext(), estimator_arg.default_value) == "False"
    include = next(action for action in description.entities
                   if action.__class__.__name__ == "IncludeLaunchDescription")
    arguments = dict(include.launch_arguments)
    assert isinstance(arguments["use_state_estimator"], launch.LaunchConfiguration)


def test_subscription_callbacks_are_jazzy_compatible():
    # Test callback shape without initializing ROS or touching any live graph.
    node = audit.Audit.__new__(audit.Audit)
    node.subscriptions_kept = []
    callbacks, received = [], []
    def fake_subscribe(msg_type, topic, callback, qos):
        callbacks.append(callback)
        return object()
    node.create_subscription = fake_subscribe
    node.subscribe(object, "/test", lambda t, m: received.append((t, m)), None)
    assert len(inspect.signature(callbacks[0]).parameters) == 1
    message = SimpleNamespace(data=1.)
    callbacks[0](message)
    assert received == [("/test", message)]


def test_ros_recording_in_isolated_domain():
    """Exercise real Jazzy subscriptions without the user's ROS domain/hardware."""
    import rclpy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import Imu, PointCloud2
    from std_msgs.msg import Float32, String
    from tf2_msgs.msg import TFMessage

    rclpy.init(domain_id=207)
    node = audit.Audit()
    source = Node("leti_synthetic_diagnostic_inputs")
    try:
        publishers = {t: source.create_publisher(Odometry, t, 10) for t in audit.POSE_TOPICS}
        tf_pub = source.create_publisher(TFMessage, "/tf", 10)
        static_pub = source.create_publisher(TFMessage, "/tf_static", QoSProfile(
            depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        quality_pub = source.create_publisher(Float32, "/lidar_odometry/pose_quality", 10)
        status_pub = source.create_publisher(String, "/mola_diagnostics/lidar_odom/status", 10)
        imu_pub = source.create_publisher(Imu, "/livox/imu", 10)
        points_pub = source.create_publisher(PointCloud2, "/livox/points", 10)
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if all(p.get_subscription_count() > 0 for p in publishers.values()):
                break
        assert all(p.get_subscription_count() > 0 for p in publishers.values())
        for index in range(6):
            message = Odometry()
            message.header.stamp = source.get_clock().now().to_msg()
            message.child_frame_id = "livox_frame"
            message.pose.pose.orientation.w = 1.
            message.pose.pose.position.x = index*.01
            for topic, publisher in publishers.items():
                message.header.frame_id = "odom" if topic in audit.POSE_TOPICS[:2] else "map"
                publisher.publish(message)
            transform = TransformStamped()
            transform.header = message.header
            transform.header.frame_id = "odom"
            transform.child_frame_id = "livox_frame"
            transform.transform.rotation.w = 1.
            tf_pub.publish(TFMessage(transforms=[transform]))
            transform.header.frame_id = "livox_frame"
            transform.child_frame_id = "base_footprint"
            static_pub.publish(TFMessage(transforms=[transform]))
            quality_pub.publish(Float32(data=1.))
            status_pub.publish(String(data="synthetic status"))
            imu = Imu()
            imu.header = message.header
            imu_pub.publish(imu)
            points = PointCloud2()
            points.header = message.header
            points_pub.publish(points)
            until = time.monotonic()+.08
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.005)
        analysis = audit.analyze(node.data)
        assert not analysis["missing_pose_topics"]
        assert analysis["raw_vs_smoothed"]["matched"] >= 4
        assert node.data["sensor_headers"]["/livox/imu"]
        assert node.data["sensor_headers"]["/livox/points"]
        assert node.data["status"]
        assert "livox_frame->base_footprint" in node.data["static_tf"]
        assert node.data["streams"]["odom->livox_frame"]
        endpoints = node.publishers()["/lidar_odometry/pose"]
        assert any(p["node"] == source.get_name() for p in endpoints)
        for topic in audit.POSE_TOPICS:
            assert not any(p["node"] == node.get_name() for p in node.publishers()[topic])
        assert not any(p["node"] == node.get_name() for p in node.publishers()["/tf"])
    finally:
        source.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
