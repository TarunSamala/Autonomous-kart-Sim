#!/usr/bin/env python3
"""Build a short vehicle-frame centerline from left and right cone poses."""

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

from .path_geometry import pair_cones, smooth_path


class FsdsLocalPlanner(Node):
    def __init__(self):
        super().__init__('fsds_local_planner')
        self.declare_parameter('base_frame', 'fsds/FSCar')
        self.declare_parameter('min_track_width', 1.5)
        self.declare_parameter('max_track_width', 6.0)
        self.declare_parameter('max_longitudinal_offset', 2.0)
        self.declare_parameter('input_timeout', 0.5)

        self.left = None
        self.right = None
        self.left_time = None
        self.right_time = None
        self.create_subscription(
            PoseArray, '/autonomy/cones/left', self.left_callback, 10)
        self.create_subscription(
            PoseArray, '/autonomy/cones/right', self.right_callback, 10)
        self.path_publisher = self.create_publisher(
            Path, '/autonomy/local_path', 10)
        self.create_timer(0.05, self.publish_path)
        self.get_logger().info('Local planner ready')

    def left_callback(self, message):
        self.left = [(pose.position.x, pose.position.y)
                     for pose in message.poses]
        self.left_time = self.get_clock().now()

    def right_callback(self, message):
        self.right = [(pose.position.x, pose.position.y)
                      for pose in message.poses]
        self.right_time = self.get_clock().now()

    def inputs_are_fresh(self):
        if self.left_time is None or self.right_time is None:
            return False
        timeout = self.get_parameter('input_timeout').value
        now = self.get_clock().now()
        return ((now - self.left_time).nanoseconds * 1e-9 <= timeout and
                (now - self.right_time).nanoseconds * 1e-9 <= timeout)

    def publish_path(self):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter('base_frame').value
        if not self.inputs_are_fresh():
            self.path_publisher.publish(message)
            return

        points = pair_cones(
            self.left,
            self.right,
            self.get_parameter('min_track_width').value,
            self.get_parameter('max_track_width').value,
            self.get_parameter('max_longitudinal_offset').value,
        )
        points = smooth_path(points)
        for x, y in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = FsdsLocalPlanner()
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
