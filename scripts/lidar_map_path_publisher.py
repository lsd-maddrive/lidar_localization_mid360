#!/usr/bin/env python3
"""Publish the lidar trajectory in a fixed map frame as nav_msgs/Path."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class LidarMapPathPublisher(Node):
    def __init__(self):
        super().__init__("lidar_map_path_publisher")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.lidar_frame = self.declare_parameter("lidar_frame", "livox_frame").value
        self.path_topic = self.declare_parameter("path_topic", "/lidar_path").value
        self.publish_period = float(self.declare_parameter("publish_period", 0.1).value)
        self.min_distance = float(self.declare_parameter("min_distance", 0.03).value)
        self.max_points = int(self.declare_parameter("max_points", 20000).value)
        self.lookup_timeout = float(self.declare_parameter("lookup_timeout", 0.02).value)

        if self.publish_period <= 0.0 or self.min_distance < 0.0 or self.max_points < 1:
            raise ValueError("publish_period must be > 0, min_distance >= 0, max_points >= 1")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, self.path_topic, qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.path = Path()
        self.path.header.frame_id = self.map_frame
        self.last_position = None
        self.timer = self.create_timer(self.publish_period, self.update)

        self.get_logger().info(
            f"Publishing {self.map_frame}->{self.lidar_frame} trajectory on {self.path_topic}; "
            f"period={self.publish_period:.3f}s, min_distance={self.min_distance:.3f}m"
        )

    def update(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.lidar_frame,
                Time(),
                timeout=Duration(seconds=self.lookup_timeout),
            )
        except TransformException:
            return

        t = transform.transform.translation
        position = (t.x, t.y, t.z)
        if self.last_position is not None:
            distance = math.sqrt(sum((position[i] - self.last_position[i]) ** 2 for i in range(3)))
            if distance < self.min_distance:
                return

        pose = PoseStamped()
        pose.header = transform.header
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = t.x
        pose.pose.position.y = t.y
        pose.pose.position.z = t.z
        pose.pose.orientation.x = transform.transform.rotation.x
        pose.pose.orientation.y = transform.transform.rotation.y
        pose.pose.orientation.z = transform.transform.rotation.z
        pose.pose.orientation.w = transform.transform.rotation.w

        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(pose)
        self.last_position = position

        if len(self.path.poses) > self.max_points:
            self.path.poses = self.path.poses[-self.max_points:]

        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = LidarMapPathPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
