#!/usr/bin/python3
"""Run a tiny startup sequence, then drive in a small circle."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node


class CircleDriver(Node):
    def __init__(self):
        super().__init__('circle_driver')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('reverse_speed', -0.08)
        self.declare_parameter('reverse_duration', 1.0)
        self.declare_parameter('forward_speed', 0.10)
        self.declare_parameter('forward_duration', 1.2)
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_speed', 0.50)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.reverse_duration = float(self.get_parameter('reverse_duration').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.forward_duration = float(self.get_parameter('forward_duration').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_cmd)
        self.start_time = self.get_clock().now()
        self.last_phase = None

        self.get_logger().info(
            f'Circle driver publishing to {self.cmd_vel_topic} '
            f'with startup reverse={self.reverse_speed:.2f}, startup forward={self.forward_speed:.2f}, '
            f'circle linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}'
        )

    def publish_cmd(self):
        msg = Twist()
        elapsed = self.get_clock().now() - self.start_time
        reverse_limit = Duration(seconds=self.reverse_duration)
        forward_limit = Duration(seconds=self.reverse_duration + self.forward_duration)

        if elapsed < reverse_limit:
            phase = 'reverse'
            msg.linear.x = self.reverse_speed
            msg.angular.z = 0.0
        elif elapsed < forward_limit:
            phase = 'forward'
            msg.linear.x = self.forward_speed
            msg.angular.z = 0.0
        else:
            phase = 'circle'
            msg.linear.x = self.linear_speed
            msg.angular.z = self.angular_speed

        if phase != self.last_phase:
            self.get_logger().info(f'phase={phase}')
            self.last_phase = phase

        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = CircleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
