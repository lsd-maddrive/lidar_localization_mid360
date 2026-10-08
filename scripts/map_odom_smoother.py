#!/usr/bin/env python3
"""Robust time-window smoother for the MOLA map->odom correction TF."""

from collections import deque
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(n) or n < 1e-12:
        raise ValueError("invalid zero-length quaternion")
    return [v / n for v in q]


def qmul(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def qrotate(q, v):
    return qmul(qmul(q, [v[0], v[1], v[2], 0.0]), [-q[0], -q[1], -q[2], q[3]])[:3]


def compose(a, b):
    aq, bq = normalize(a[3:]), normalize(b[3:])
    rotated = qrotate(aq, b[:3])
    return [a[i] + rotated[i] for i in range(3)] + normalize(qmul(aq, bq))


def inverse(pose):
    q = normalize(pose[3:])
    qi = [-q[0], -q[1], -q[2], q[3]]
    t = qrotate(qi, [-v for v in pose[:3]])
    return t + qi


def pose_distance(a, b):
    dp = math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))
    qa, qb = normalize(a[3:]), normalize(b[3:])
    dot = min(1.0, abs(sum(x * y for x, y in zip(qa, qb))))
    return dp, math.degrees(2.0 * math.acos(dot))


def mean_pose(poses):
    if len(poses) == 1:
        return list(poses[0])
    count = float(len(poses))
    xyz = [sum(p[i] for p in poses) / count for i in range(3)]
    reference = normalize(poses[0][3:])
    qsum = [0.0] * 4
    for pose in poses:
        q = normalize(pose[3:])
        if sum(x * y for x, y in zip(reference, q)) < 0:
            q = [-v for v in q]
        for i in range(4):
            qsum[i] += q[i]
    return xyz + normalize(qsum)


def message_pose(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    return [p.x, p.y, p.z, *normalize([q.x, q.y, q.z, q.w])]


def transform_pose(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    return [t.x, t.y, t.z, *normalize([q.x, q.y, q.z, q.w])]


class MapOdomSmoother(Node):
    def __init__(self):
        super().__init__("map_odom_smoother")
        self.declare_parameter("pose_topic", "/lidar_odometry/pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "livox_frame")
        self.declare_parameter("window_seconds", 3.0)
        self.declare_parameter("max_sync_gap_seconds", 0.30)
        self.declare_parameter("max_candidate_step_m", 0.75)
        self.declare_parameter("max_candidate_step_deg", 20.0)
        self.declare_parameter("outlier_confirmations", 4)
        self.declare_parameter("stale_timeout_seconds", 0.75)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.window_s = float(self.get_parameter("window_seconds").value)
        self.max_gap_s = float(self.get_parameter("max_sync_gap_seconds").value)
        self.max_step_m = float(self.get_parameter("max_candidate_step_m").value)
        self.max_step_deg = float(self.get_parameter("max_candidate_step_deg").value)
        self.confirmations = int(self.get_parameter("outlier_confirmations").value)
        self.stale_timeout_s = float(self.get_parameter("stale_timeout_seconds").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        if min(self.window_s, self.max_gap_s, self.stale_timeout_s, rate) <= 0:
            raise ValueError("window, sync gap, stale timeout and publish rate must be positive")
        if self.max_step_m <= 0 or self.max_step_deg <= 0 or self.confirmations < 2:
            raise ValueError("outlier thresholds must be positive and confirmations >= 2")

        self.buffer = Buffer()
        # The main executor owns this node, including its TF subscriptions.
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.pending = deque(maxlen=100)
        self.broadcaster = TransformBroadcaster(self)
        self.samples = deque()
        self.suspect_samples = []
        self.latest_mean = None
        self.latest_input_time = None
        self.last_stamp = None
        self.accepted = 0
        self.rejected = 0
        self.tf_lookup_failures = 0

        topic = str(self.get_parameter("pose_topic").value)
        self.subscription = self.create_subscription(Odometry, topic, self.on_pose, 100)
        self.timer = self.create_timer(1.0 / rate, self.publish_tf)
        self.sync_timer = self.create_timer(0.01, self.process_pending)
        self.get_logger().info(
            f"Smoothing {self.map_frame}->{self.odom_frame} over {self.window_s:.1f}s; "
            f"input={topic}, odometry TF={self.odom_frame}->{self.base_frame}"
        )

    def on_pose(self, message):
        if message.header.frame_id.lstrip("/") != self.map_frame:
            self.get_logger().warning(
                f"Ignoring pose in unexpected frame {message.header.frame_id!r}"
            )
            return
        if message.child_frame_id.lstrip("/") != self.base_frame:
            self.get_logger().warning(
                f"Ignoring pose for unexpected child frame {message.child_frame_id!r}"
            )
            return

        self.pending.append((time.monotonic(), message))

    def process_pending(self):
        while self.pending:
            arrived, message = self.pending[0]
            if time.monotonic() - arrived > self.max_gap_s:
                self.pending.popleft()
                self.rejected += 1
                continue
            if not self.buffer.can_transform(
                self.odom_frame, self.base_frame,
                Time.from_msg(message.header.stamp),
            ):
                # Return immediately so the same executor can receive more TF.
                return
            self.pending.popleft()
            self.process_pose(message)

    def process_pose(self, message):
        stamp = message.header.stamp
        try:
            odom_tf = self.buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time.from_msg(stamp),
                timeout=Duration(seconds=0.0),
            )
            map_base = message_pose(message)
            odom_base = transform_pose(odom_tf)
        except (TransformException, ValueError) as error:
            self.tf_lookup_failures += 1
            if self.tf_lookup_failures == 1 or self.tf_lookup_failures % 20 == 0:
                self.get_logger().warning(
                    f"Cannot align MOLA pose with odometry TF yet: {error}"
                )
            return

        tf_stamp = stamp.sec + stamp.nanosec * 1e-9
        tf_dt = abs((odom_tf.header.stamp.sec + odom_tf.header.stamp.nanosec * 1e-9) - tf_stamp)
        if tf_dt > self.max_gap_s:
            self.rejected += 1
            return
        if self.last_stamp is not None and tf_stamp <= self.last_stamp:
            self.rejected += 1
            return

        candidate = compose(map_base, inverse(odom_base))
        self.last_stamp = tf_stamp
        self.latest_input_time = self.get_clock().now()
        self.accept_candidate(tf_stamp, candidate)

    def accept_candidate(self, stamp, candidate):
        if self.latest_mean is not None:
            distance, angle = pose_distance(candidate, self.latest_mean)
            if distance > self.max_step_m or angle > self.max_step_deg:
                if self.suspect_samples:
                    sd, sa = pose_distance(candidate, self.suspect_samples[-1][1])
                else:
                    sd, sa = float("inf"), float("inf")
                if sd <= self.max_step_m and sa <= self.max_step_deg:
                    self.suspect_samples.append((stamp, candidate))
                else:
                    self.suspect_samples = [(stamp, candidate)]
                self.rejected += 1
                if len(self.suspect_samples) < self.confirmations:
                    return
                # A sustained, self-consistent correction is treated as a real
                # relocalization and enters the same slow windowed average.
                for sample_stamp, sample_pose in self.suspect_samples:
                    self.samples.append((sample_stamp, sample_pose))
                    self.accepted += 1
                self.suspect_samples.clear()
                self.get_logger().warning(
                    "Accepted a sustained map correction after repeated consistent samples"
                )
            else:
                self.suspect_samples.clear()
                self.samples.append((stamp, candidate))
                self.accepted += 1
        else:
            self.samples.append((stamp, candidate))
            self.accepted += 1

        newest_stamp = self.samples[-1][0]
        while self.samples and self.samples[0][0] < newest_stamp - self.window_s:
            self.samples.popleft()
        self.latest_mean = mean_pose([pose for _, pose in self.samples])

    def publish_tf(self):
        if self.latest_mean is None or self.latest_input_time is None:
            return
        age = (self.get_clock().now() - self.latest_input_time).nanoseconds * 1e-9
        if age > self.stale_timeout_s:
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = self.latest_mean[0]
        transform.transform.translation.y = self.latest_mean[1]
        transform.transform.translation.z = self.latest_mean[2]
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = self.latest_mean[3:]
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = MapOdomSmoother()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
