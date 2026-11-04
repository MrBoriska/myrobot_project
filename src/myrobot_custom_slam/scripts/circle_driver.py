#!/usr/bin/env python3
# Copyright 2024 myrobot_project. Apache-2.0.
"""Repeatable /cmd_vel source for simulator runs (Gazebo or fake_sensors).

A scripted motion makes SLAM experiments reproducible: the same circle every
run, no joystick, no operator.

    ros2 run myrobot_custom_slam circle_driver.py --speed 0.3 --radius 1.0
"""

import argparse
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class CircleDriver(Node):

    def __init__(self, args):
        super().__init__('circle_driver')
        self.args = args
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.start = None
        self.ready = False
        # Drive only once the estimator has produced a map: its IMU
        # initialisation needs a static window, and starting to move before
        # that would make the start look like motion and never initialise.
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_timer(1.0 / 20.0, self.tick)
        self.get_logger().info(
            f'circle_driver: speed={args.speed} m/s, radius={args.radius} m, '
            f'duration={args.duration} s, waiting for /map before moving')

    def on_map(self, msg):
        if not self.ready:
            self.ready = True
            self.get_logger().info('SLAM is up: starting the run')

    def tick(self):
        if not self.ready:
            self.pub.publish(Twist())
            return
        if self.start is None:
            self.start = self.get_clock().now()
        elapsed = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        msg = Twist()
        # Do not move while the estimator is collecting its static IMU window.
        elapsed -= self.args.delay
        if elapsed < 0.0:
            self.pub.publish(msg)
            return
        if self.args.duration > 0.0 and elapsed > self.args.duration:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            omega = self.args.speed / max(0.05, self.args.radius)
            ramp = min(1.0, elapsed / max(0.1, self.args.ramp))
            msg.linear.x = self.args.speed * ramp
            msg.angular.z = omega * ramp
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=float, default=0.3, help='[m/s]')
    parser.add_argument('--radius', type=float, default=1.0, help='[m]')
    parser.add_argument('--duration', type=float, default=0.0, help='[s], 0 = forever')
    parser.add_argument('--ramp', type=float, default=1.0, help='[s] soft start')
    parser.add_argument('--delay', type=float, default=5.0,
                        help='[s] stand still before moving (IMU initialisation)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = CircleDriver(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
