#!/usr/bin/env python3
"""Extract local cone candidates from the forward FSDS lidar."""

import math
import struct

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


class FsdsConeDetector(Node):
    def __init__(self):
        super().__init__('fsds_cone_detector')
        self.declare_parameter('lidar_topic', '/fsds/lidar/Lidar1')
        self.declare_parameter('base_frame', 'fsds/FSCar')
        self.declare_parameter('sensor_x', 0.45)
        self.declare_parameter('sensor_y', 0.0)
        self.declare_parameter('sensor_yaw', 0.0)
        self.declare_parameter('cluster_radius', 0.20)
        self.declare_parameter('min_cluster_points', 5)
        self.declare_parameter('max_cluster_points', 180)
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('max_range', 15.0)
        self.declare_parameter('min_z', -0.30)
        self.declare_parameter('max_z', 0.30)

        self.create_subscription(
            PointCloud2,
            self.get_parameter('lidar_topic').value,
            self.lidar_callback,
            qos_profile_sensor_data,
        )
        self.left_pose_publisher = self.create_publisher(
            PoseArray, '/autonomy/cones/left', 10)
        self.right_pose_publisher = self.create_publisher(
            PoseArray, '/autonomy/cones/right', 10)
        self.left_marker_publisher = self.create_publisher(
            MarkerArray, '/left_cones', 10)
        self.right_marker_publisher = self.create_publisher(
            MarkerArray, '/right_cones', 10)
        self.get_logger().info(
            'Cone detector listening on %s' %
            self.get_parameter('lidar_topic').value)

    def lidar_callback(self, message):
        points = self.pointcloud2_to_xyz(message)
        if points.size == 0:
            self.publish([], [], message.header.stamp)
            return

        radius = np.linalg.norm(points[:, :2], axis=1)
        mask = (
            (points[:, 2] > self.get_parameter('min_z').value) &
            (points[:, 2] < self.get_parameter('max_z').value) &
            (radius > self.get_parameter('min_range').value) &
            (radius < self.get_parameter('max_range').value) &
            (points[:, 0] > 0.0)
        )
        points = points[mask]
        clusters = self.euclidean_clustering(points)

        left = []
        right = []
        for cluster in clusters:
            if not self.is_cone_sized(cluster):
                continue
            centroid = np.mean(cluster, axis=0)
            x, y = self.to_base_frame(float(centroid[0]), float(centroid[1]))
            (left if y > 0.0 else right).append((x, y))

        self.publish(left, right, message.header.stamp)

    def pointcloud2_to_xyz(self, message):
        if message.point_step < 12:
            return np.empty((0, 3), dtype=np.float32)
        byte_order = '>' if message.is_bigendian else '<'
        unpacker = struct.Struct(byte_order + 'fff')
        points = []
        for offset in range(0, len(message.data), message.point_step):
            x, y, z = unpacker.unpack_from(message.data, offset)
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append((x, y, z))
        return np.asarray(points, dtype=np.float32)

    def euclidean_clustering(self, points):
        """Cluster using a spatial hash instead of an O(N^2) scan."""
        if len(points) == 0:
            return []
        radius = self.get_parameter('cluster_radius').value
        minimum = self.get_parameter('min_cluster_points').value
        maximum = self.get_parameter('max_cluster_points').value
        cells = {}
        for index, point in enumerate(points):
            key = tuple(np.floor(point / radius).astype(int))
            cells.setdefault(key, []).append(index)

        used = np.zeros(len(points), dtype=bool)
        clusters = []
        for start in range(len(points)):
            if used[start]:
                continue
            used[start] = True
            pending = [start]
            indices = []
            while pending and len(indices) <= maximum:
                index = pending.pop()
                indices.append(index)
                cell = tuple(np.floor(points[index] / radius).astype(int))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for dz in (-1, 0, 1):
                            neighbour_cell = (
                                cell[0] + dx, cell[1] + dy, cell[2] + dz)
                            for neighbour in cells.get(neighbour_cell, []):
                                if used[neighbour]:
                                    continue
                                separation = np.linalg.norm(
                                    points[neighbour] - points[index])
                                if separation < radius:
                                    used[neighbour] = True
                                    pending.append(neighbour)
            if minimum <= len(indices) <= maximum:
                clusters.append(points[indices])
        return clusters

    @staticmethod
    def is_cone_sized(cluster):
        extent = np.ptp(cluster, axis=0)
        return extent[0] < 0.8 and extent[1] < 0.8 and extent[2] < 0.8

    def to_base_frame(self, x, y):
        yaw = self.get_parameter('sensor_yaw').value
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return (
            self.get_parameter('sensor_x').value + cosine * x - sine * y,
            self.get_parameter('sensor_y').value + sine * x + cosine * y,
        )

    def publish(self, left, right, stamp):
        frame = self.get_parameter('base_frame').value
        self.left_pose_publisher.publish(
            self.make_pose_array(left, frame, stamp))
        self.right_pose_publisher.publish(
            self.make_pose_array(right, frame, stamp))
        self.left_marker_publisher.publish(
            self.make_markers(left, frame, stamp, 'left', (0.0, 0.2, 1.0)))
        self.right_marker_publisher.publish(
            self.make_markers(right, frame, stamp, 'right', (1.0, 0.9, 0.0)))

    @staticmethod
    def make_pose_array(points, frame, stamp):
        message = PoseArray()
        message.header.frame_id = frame
        message.header.stamp = stamp
        for x, y in points:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.15
            pose.orientation.w = 1.0
            message.poses.append(pose)
        return message

    @staticmethod
    def make_markers(points, frame, stamp, namespace, color):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for index, (x, y) in enumerate(points):
            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = stamp
            marker.ns = namespace
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.23
            marker.scale.y = 0.23
            marker.scale.z = 0.40
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            marker.lifetime.nanosec = 250000000
            array.markers.append(marker)
        return array


def main(args=None):
    rclpy.init(args=args)
    node = FsdsConeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
