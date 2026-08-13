#!/usr/bin/env python3
"""Fail-safe pure-pursuit control for an FSDS vehicle-frame local path."""

import math

import rclpy
from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_srvs.srv import SetBool

from .path_geometry import pure_pursuit_curvature, select_lookahead


class FsdsPurePursuitController(Node):
    def __init__(self):
        super().__init__('fsds_pure_pursuit_controller')
        self.declare_parameter('enabled', False)
        self.declare_parameter('lookahead_distance', 2.5)
        self.declare_parameter('wheelbase', 1.55)
        self.declare_parameter('max_steering_angle', 0.55)
        self.declare_parameter('steering_sign', -1.0)
        self.declare_parameter('straight_throttle', 0.20)
        self.declare_parameter('corner_throttle', 0.11)
        self.declare_parameter('path_timeout', 0.5)
        self.declare_parameter('odom_timeout', 0.5)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.path = []
        self.path_time = None
        self.odom_time = None
        self.speed = 0.0

        self.create_subscription(
            Path, '/autonomy/local_path', self.path_callback, 10)
        self.create_subscription(
            Odometry, '/fsds/testing_only/odom', self.odom_callback, 10)
        self.command_publisher = self.create_publisher(
            ControlCommand, '/fsds/control_command', 1)
        self.create_service(SetBool, '/autonomy/enable', self.enable_callback)
        self.create_timer(0.05, self.control_loop)
        state = 'ENABLED' if self.enabled else 'DISABLED'
        self.get_logger().info(
            'Pure pursuit ready (%s)' % state)

    def path_callback(self, message):
        self.path = [(pose.pose.position.x, pose.pose.position.y)
                     for pose in message.poses]
        self.path_time = self.get_clock().now()

    def odom_callback(self, message):
        velocity = message.twist.twist.linear
        self.speed = math.hypot(velocity.x, velocity.y)
        self.odom_time = self.get_clock().now()

    def enable_callback(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = (
            'autonomy enabled' if self.enabled else 'autonomy disabled')
        self.get_logger().warn(response.message.upper())
        return response

    def is_fresh(self, stamp, parameter):
        if stamp is None:
            return False
        timeout = self.get_parameter(parameter).value
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9 <= timeout

    def stop_command(self):
        command = ControlCommand()
        command.throttle = 0.0
        command.steering = 0.0
        command.brake = 1.0
        return command

    def control_loop(self):
        if (not self.enabled or len(self.path) < 2 or
                not self.is_fresh(self.path_time, 'path_timeout') or
                not self.is_fresh(self.odom_time, 'odom_timeout')):
            self.command_publisher.publish(self.stop_command())
            return

        target = select_lookahead(
            self.path, self.get_parameter('lookahead_distance').value)
        if target is None:
            self.command_publisher.publish(self.stop_command())
            return

        curvature = pure_pursuit_curvature(target)
        wheelbase = self.get_parameter('wheelbase').value
        steering_angle = math.atan(wheelbase * curvature)
        max_angle = self.get_parameter('max_steering_angle').value
        normalized_steering = steering_angle / max_angle
        normalized_steering *= self.get_parameter('steering_sign').value

        corner_amount = min(1.0, abs(curvature) / 0.45)
        straight = self.get_parameter('straight_throttle').value
        corner = self.get_parameter('corner_throttle').value

        command = ControlCommand()
        command.throttle = straight + (corner - straight) * corner_amount
        command.steering = max(-1.0, min(1.0, normalized_steering))
        command.brake = 0.0
        self.command_publisher.publish(command)

    def destroy_node(self):
        if rclpy.ok():
            for _ in range(3):
                self.command_publisher.publish(self.stop_command())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FsdsPurePursuitController()
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
