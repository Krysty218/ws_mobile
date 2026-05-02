#!/usr/bin/python3
"""Simple forward-first wall follower using a 2D lidar scan."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class RandomWalkDriver(Node):
    def __init__(self):
        super().__init__('random_walk_driver')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('forward_speed', 0.18)
        self.declare_parameter('turn_speed', 0.55)
        self.declare_parameter('front_stop_distance', 0.80)
        self.declare_parameter('front_slow_distance', 1.10)
        self.declare_parameter('wall_target_distance', 0.90)
        self.declare_parameter('wall_far_distance', 1.30)
        self.declare_parameter('control_period', 0.10)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.front_stop_distance = float(self.get_parameter('front_stop_distance').value)
        self.front_slow_distance = float(self.get_parameter('front_slow_distance').value)
        self.wall_target_distance = float(self.get_parameter('wall_target_distance').value)
        self.wall_far_distance = float(self.get_parameter('wall_far_distance').value)
        self.control_period = float(self.get_parameter('control_period').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.last_scan = None
        self.last_mode = None

        self.get_logger().info(
            f'Wall follower active on {self.cmd_vel_topic} using {self.scan_topic}'
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

    def log_mode(self, mode, front, front_left, left):
        if self.last_mode != mode:
            self.get_logger().info(
                f'mode={mode} front={front:.2f} front_left={front_left:.2f} left={left:.2f}'
            )
            self.last_mode = mode

    def control_loop(self):
        if self.last_scan is None:
            self.publish_cmd(0.0, 0.0)
            if self.last_mode != 'waiting_for_scan':
                self.get_logger().info('Waiting for /scan...')
                self.last_mode = 'waiting_for_scan'
            return

        scan = self.last_scan

        # Sectors are chosen so the robot prefers moving forward and tracks a wall on its left.
        front = self.sector_min(scan, 0.47, 0.53)
        front_left = self.sector_min(scan, 0.53, 0.68)
        left = self.sector_min(scan, 0.68, 0.88)

        linear_x = self.forward_speed
        angular_z = 0.0
        mode = 'forward'

        if front < self.front_stop_distance:
            # Front blocked: rotate right until the path opens.
            linear_x = 0.0
            angular_z = -self.turn_speed
            mode = 'turn_right'
        else:
            if front < self.front_slow_distance:
                linear_x = 0.08
                angular_z = -0.35 * self.turn_speed
                mode = 'slow_turn_right'
            elif left > self.wall_far_distance and front_left > self.wall_far_distance:
                # No wall detected on the left: search for one while still moving.
                linear_x = 0.12
                angular_z = 0.30 * self.turn_speed
                mode = 'search_left_wall'
            else:
                wall_error = left - self.wall_target_distance
                angular_z = max(min(1.3 * wall_error, 0.35), -0.35)

                # Positive angular z turns left; if we're too close to the wall,
                # wall_error becomes negative and the robot steers right.
                if abs(wall_error) < 0.12:
                    mode = 'follow_wall'
                elif wall_error > 0.0:
                    mode = 'drift_left'
                else:
                    mode = 'drift_right'

        self.publish_cmd(linear_x, angular_z)
        self.log_mode(mode, front, front_left, left)


def main():
    rclpy.init()
    node = RandomWalkDriver()
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
