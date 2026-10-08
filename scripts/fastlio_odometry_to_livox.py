#!/usr/bin/env python3
"""Express FAST-LIO body odometry at the Livox LiDAR frame.

FAST-LIO publishes ``camera_init -> body`` while the MOLA profile in this
workspace localizes the LiDAR itself (``livox_frame``).  The static transform
is read from the same FAST-LIO YAML file passed to this node.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_to_rotation(x, y, z, w):
    """Return the 3x3 rotation matrix for an XYZW quaternion."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("odometry contains a zero-norm orientation quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def rotation_to_quaternion(rotation):
    """Return an XYZW quaternion for a 3x3 rotation matrix."""
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        return (
            (rotation[2][1] - rotation[1][2]) / scale,
            (rotation[0][2] - rotation[2][0]) / scale,
            (rotation[1][0] - rotation[0][1]) / scale,
            0.25 * scale,
        )
    diagonal = [rotation[0][0], rotation[1][1], rotation[2][2]]
    index = diagonal.index(max(diagonal))
    if index == 0:
        scale = 2.0 * math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2])
        return (
            0.25 * scale,
            (rotation[0][1] + rotation[1][0]) / scale,
            (rotation[0][2] + rotation[2][0]) / scale,
            (rotation[2][1] - rotation[1][2]) / scale,
        )
    if index == 1:
        scale = 2.0 * math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2])
        return (
            (rotation[0][1] + rotation[1][0]) / scale,
            0.25 * scale,
            (rotation[1][2] + rotation[2][1]) / scale,
            (rotation[0][2] - rotation[2][0]) / scale,
        )
    scale = 2.0 * math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1])
    return (
        (rotation[0][2] + rotation[2][0]) / scale,
        (rotation[1][2] + rotation[2][1]) / scale,
        0.25 * scale,
        (rotation[1][0] - rotation[0][1]) / scale,
    )


def matrix_multiply(left, right):
    return [
        [sum(left[row][k] * right[k][column] for k in range(len(right)))
         for column in range(len(right[0]))]
        for row in range(len(left))
    ]


def matrix_transpose(matrix):
    return [list(row) for row in zip(*matrix)]


def skew(vector):
    x, y, z = vector
    return [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]]


class FastLioOdometryToLivox(Node):
    def __init__(self):
        super().__init__("fastlio_odometry_to_livox")

        self.declare_parameter("input_topic", "/Odometry")
        self.declare_parameter("output_topic", "/fastlio/odometry_livox")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("lidar_frame", "livox_frame")
        self.declare_parameter("publish_tf", True)
        # These parameter names match FAST-LIO's mid360.yaml exactly.
        self.declare_parameter("mapping.extrinsic_T", [0.0, 0.0, 0.0])
        self.declare_parameter(
            "mapping.extrinsic_R", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )

        translation = list(self.get_parameter("mapping.extrinsic_T").value)
        rotation_values = list(self.get_parameter("mapping.extrinsic_R").value)
        if len(translation) != 3 or len(rotation_values) != 9:
            raise ValueError("mapping.extrinsic_T must have 3 values and mapping.extrinsic_R 9 values")
        self.body_to_lidar_translation = translation
        self.body_to_lidar_rotation = [rotation_values[i:i + 3] for i in range(0, 9, 3)]

        self.lidar_frame = self.get_parameter("lidar_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.publish_tf = self.get_parameter("publish_tf").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.publisher = self.create_publisher(Odometry, output_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.subscription = self.create_subscription(Odometry, input_topic, self.on_odometry, 20)
        self.get_logger().info(
            f"Converting {input_topic} body poses to {output_topic} at frame {self.lidar_frame}"
        )

    def transformed_covariance(self, covariance):
        """Propagate a [translation, rotation] covariance through T_body_lidar."""
        rotation = self.body_to_lidar_rotation
        translation_rotation = matrix_multiply(skew(self.body_to_lidar_translation), rotation)
        adjoint = [[0.0] * 6 for _ in range(6)]
        for row in range(3):
            for column in range(3):
                adjoint[row][column] = rotation[row][column]
                adjoint[row][column + 3] = translation_rotation[row][column]
                adjoint[row + 3][column + 3] = rotation[row][column]
        source = [list(covariance[index:index + 6]) for index in range(0, 36, 6)]
        result = matrix_multiply(matrix_multiply(adjoint, source), matrix_transpose(adjoint))
        return [value for row in result for value in row]

    def on_odometry(self, message):
        try:
            body_rotation = quaternion_to_rotation(
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
            )
        except ValueError as error:
            self.get_logger().warning(str(error))
            return

        lidar_rotation = matrix_multiply(body_rotation, self.body_to_lidar_rotation)
        offset_in_world = [
            sum(body_rotation[row][column] * self.body_to_lidar_translation[column]
                for column in range(3))
            for row in range(3)
        ]

        output = Odometry()
        output.header = message.header
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.lidar_frame
        output.pose.pose.position.x = message.pose.pose.position.x + offset_in_world[0]
        output.pose.pose.position.y = message.pose.pose.position.y + offset_in_world[1]
        output.pose.pose.position.z = message.pose.pose.position.z + offset_in_world[2]
        quaternion = rotation_to_quaternion(lidar_rotation)
        output.pose.pose.orientation.x = quaternion[0]
        output.pose.pose.orientation.y = quaternion[1]
        output.pose.pose.orientation.z = quaternion[2]
        output.pose.pose.orientation.w = quaternion[3]
        output.pose.covariance = self.transformed_covariance(message.pose.covariance)
        output.twist = message.twist
        self.publisher.publish(output)

        if self.tf_broadcaster:
            transform = TransformStamped()
            transform.header = output.header
            transform.child_frame_id = self.lidar_frame
            transform.transform.translation.x = output.pose.pose.position.x
            transform.transform.translation.y = output.pose.pose.position.y
            transform.transform.translation.z = output.pose.pose.position.z
            transform.transform.rotation = output.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = FastLioOdometryToLivox()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
