#!/usr/bin/env python3
# Copyright 2024 myrobot_project. Apache-2.0.
"""Compare the SLAM pose against ground truth and fail if it drifts.

Looks up map -> base_link (the whole SLAM chain: map -> odom from the estimator,
odom -> base_link from the wheel odometry) and compares it with /ground_truth,
which fake_sensors publishes. With --verbose it also prints the pose pairs,
which is usually what you want while tuning.

    ros2 run myrobot_custom_slam check_slam_accuracy.py --duration 60
    ros2 run myrobot_custom_slam check_slam_accuracy.py --duration 20 --verbose

Exit code: 0 = within thresholds, 1 = drifted, 2 = could not measure.
"""

import argparse
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(quaternion):
    return 2.0 * math.atan2(quaternion.z, quaternion.w)


class Checker(Node):

    def __init__(self, args):
        super().__init__('check_slam_accuracy')
        self.args = args
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.truth = None
        self.start = None
        self.done = False
        self.samples = []
        self.create_subscription(Odometry, '/ground_truth', self.on_truth, 10)
        self.create_timer(0.1, self.check)

    def on_truth(self, msg):
        self.truth = msg

    def check(self):
        now = self.get_clock().now()
        if self.start is None:
            self.start = now
        elapsed = (now - self.start).nanoseconds * 1e-9
        if self.truth is None or elapsed < self.args.settle:
            if elapsed > self.args.settle + 5.0 and not self.samples:
                self.get_logger().error('no /ground_truth received')
                self.done = True
            return
        if not self.samples and elapsed > self.args.settle + self.args.patience:
            self.get_logger().error(
                f'no map -> base_link after {elapsed:.0f} s: is the SLAM node alive '
                'and the IMU initialised (robot standing still)?')
            self.done = True
            return
        try:
            tf = self.buffer.lookup_transform('map', 'base_link', Time())
        except TransformException as exc:
            self.get_logger().warning(
                f'map -> base_link unavailable: {exc}', throttle_duration_sec=2.0)
            return

        slam_x = tf.transform.translation.x
        slam_y = tf.transform.translation.y
        slam_yaw = yaw_of(tf.transform.rotation)
        truth_x = self.truth.pose.pose.position.x
        truth_y = self.truth.pose.pose.position.y
        truth_yaw = yaw_of(self.truth.pose.pose.orientation)

        error = math.hypot(slam_x - truth_x, slam_y - truth_y)
        yaw_error = abs(math.atan2(math.sin(slam_yaw - truth_yaw),
                                   math.cos(slam_yaw - truth_yaw)))
        self.samples.append((error, yaw_error))
        if self.args.verbose and len(self.samples) % 10 == 1:
            self.get_logger().info(
                f'truth=({truth_x:+.2f}, {truth_y:+.2f}, '
                f'{math.degrees(truth_yaw):+6.1f} deg) '
                f'slam=({slam_x:+.2f}, {slam_y:+.2f}, '
                f'{math.degrees(slam_yaw):+6.1f} deg) '
                f'err={error * 100:.1f} cm')

        if elapsed >= self.args.settle + self.args.duration:
            self.done = True

    def report(self):
        if not self.samples:
            self.get_logger().error('no samples collected')
            return 2
        errors = [s[0] for s in self.samples]
        yaws = [s[1] for s in self.samples]
        mean = sum(errors) / len(errors)
        worst = max(errors)
        yaw_mean = sum(yaws) / len(yaws)
        yaw_worst = max(yaws)
        final = errors[-1]
        print(
            f'samples={len(errors)} '
            f'position error: mean={mean * 100:.2f} cm worst={worst * 100:.2f} cm '
            f'final={final * 100:.2f} cm | yaw error: mean={math.degrees(yaw_mean):.2f} deg '
            f'worst={math.degrees(yaw_worst):.2f} deg')
        ok = final <= self.args.max_position_error and yaw_worst <= self.args.max_yaw_error
        print('RESULT:', 'PASS' if ok else 'FAIL')
        return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--settle', type=float, default=5.0, help='[s] ignore after start')
    parser.add_argument('--duration', type=float, default=60.0, help='[s] measurement window')
    parser.add_argument('--max-position-error', type=float, default=0.15, help='[m]')
    parser.add_argument('--max-yaw-error', type=float, default=0.10, help='[rad]')
    parser.add_argument('--patience', type=float, default=15.0,
                        help='[s] give up if no SLAM pose appears')
    parser.add_argument('--verbose', action='store_true')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Checker(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    code = node.report() if node.done else 2
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
