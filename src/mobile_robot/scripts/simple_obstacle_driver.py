#!/usr/bin/python3
"""Very simple obstacle driver: forward until blocked, then turn once and continue."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SimpleObstacleDriver(Node):
    def __init__(self):
        super().__init__('simple_obstacle_driver')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('forward_speed', 0.18)
        self.declare_parameter('turn_speed', 0.55)
        self.declare_parameter('front_threshold', 0.95)
        self.declare_parameter('clear_threshold', 1.30)
        self.declare_parameter('turn_duration', 2.2)
        self.declare_parameter('control_period', 0.10)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.front_threshold = float(self.get_parameter('front_threshold').value)
        self.clear_threshold = float(self.get_parameter('clear_threshold').value)
        self.turn_duration = float(self.get_parameter('turn_duration').value)
        self.control_period = float(self.get_parameter('control_period').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.last_scan = None
        self.state = 'forward'
        self.turn_direction = -1.0
        self.turn_until = None
        self.last_state = None

        self.get_logger().info(
            f'Simple obstacle driver active on {self.cmd_vel_topic} using {self.scan_topic}'
        )

    def scan_callback(self, msg):
        self.last_scan = msg

    def sector_min(self, scan, start_ratio, end_ratio):
        if not scan.ranges:
            return float('inf')

        count = len(scan.ranges)
        start = max(0, min(count - 1, int(count * start_ratio)))
        end = max(start + 1, min(count, int(count * end_ratio)))
        finite_values = [value for value in scan.ranges[start:end] if math.isfinite(value)]
        return min(finite_values) if finite_values else float('inf')

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def log_state(self, front, left, right):
        if self.last_state != self.state:
            self.get_logger().info(
                f'state={self.state} front={front:.2f} left={left:.2f} right={right:.2f}'
            )
            self.last_state = self.state

    def control_loop(self):
        now = self.get_clock().now()

        if self.last_scan is None:
            self.publish_cmd(0.0, 0.0)
            if self.last_state != 'waiting_for_scan':
                self.get_logger().info('Waiting for /scan...')
                self.last_state = 'waiting_for_scan'
            return

        scan = self.last_scan
        front = self.sector_min(scan, 0.45, 0.55)
        left = self.sector_min(scan, 0.60, 0.85)
        right = self.sector_min(scan, 0.15, 0.40)

        if self.state == 'forward':
            if front <= self.front_threshold:
                # Turn away from the closer side so we don't keep steering into the obstacle.
                self.turn_direction = -1.0 if left >= right else 1.0
                self.turn_until = now + Duration(seconds=self.turn_duration)
                self.state = 'turn'
                self.publish_cmd(0.0, self.turn_direction * self.turn_speed)
            else:
                self.publish_cmd(self.forward_speed, 0.0)

        else:
            if front >= self.clear_threshold and self.turn_until is not None and now >= self.turn_until:
                self.state = 'forward'
                self.turn_until = None
                self.publish_cmd(self.forward_speed, 0.0)
            else:
                self.publish_cmd(0.0, self.turn_direction * self.turn_speed)

        self.log_state(front, left, right)


def main():
    rclpy.init()
    node = SimpleObstacleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
