#!/usr/bin/env python3
# Copyright 2024 myrobot_project. Apache-2.0.
"""Synthetic lidar + IMU + wheel odometry for debugging the SLAM stack offline.

Why this exists: the real platform (Orange Pi Zero 3) is not always available,
and Gazebo brings physics, GUI and a GPU dependency that are useless when the
question is "does my estimator track the truth?". This node reproduces the
sensor interface of the robot exactly - topics, frames, QoS, extrinsics taken
from urdf/myrobot.xacro - and drives the robot along a known path, so the
estimator can be compared against ground truth deterministically and headless.

Published:
  /scan              sensor_msgs/LaserScan   lidar_link, per-ray time stamps
                                             (so motion compensation matters)
  /imu/data_raw      sensor_msgs/Imu        imu_link, 100 Hz, gravity included,
                                             with a small gyro bias and noise
  /odom              nav_msgs/Odometry      wheel odometry with a scale error
  /ground_truth      nav_msgs/Odometry      map -> base_link, the true pose
  TF odom -> base_link                      dead reckoning with a scale error

The room is a rectangle with two cylindrical pillars; timings, ranges and noise
are configurable.
"""

import argparse
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformBroadcaster

# ---------------------------------------------------------------- extrinsics
# Straight from urdf/myrobot.xacro (base_to_lidar / imu_joint).
BASE_TO_LIDAR_RPY = (math.pi, 0.0, math.pi / 4.0)
BASE_TO_LIDAR_XYZ = (0.0, 0.0, 0.0315)
BASE_TO_IMU_RPY = (0.0, 0.0, -math.pi)
BASE_TO_IMU_XYZ = (-0.038, 0.024, -0.0185)

GRAVITY = np.array([0.0, 0.0, -9.81])


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def yaw_matrix(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


R_BASE_LIDAR = rpy_to_matrix(*BASE_TO_LIDAR_RPY)
P_BASE_LIDAR = np.array(BASE_TO_LIDAR_XYZ)
R_BASE_IMU = rpy_to_matrix(*BASE_TO_IMU_RPY)
P_BASE_IMU = np.array(BASE_TO_IMU_XYZ)


class Trajectory:
    """Circular (or static) motion with analytically exact derivatives."""

    def __init__(self, mode, radius, speed, start_delay=5.0):
        self.mode = mode
        self.radius = radius
        self.speed = speed
        # Stand still at the beginning: the estimator needs a static window to
        # estimate the IMU biases, exactly like on the real robot.
        self.start_delay = start_delay

    def state(self, t):
        """Returns (x, y, yaw, vx, vy, omega, ax, ay)."""
        if self.mode == 'static' or t < self.start_delay:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        t -= self.start_delay
        r, v = self.radius, self.speed
        w = v / r
        phi = w * t
        x = r * math.sin(phi)
        y = r * (1.0 - math.cos(phi))
        vx = v * math.cos(phi)
        vy = v * math.sin(phi)
        ax = -r * w * w * math.sin(phi)
        ay = r * w * w * math.cos(phi)
        return x, y, phi, vx, vy, w, ax, ay


class FakeSensors(Node):

    def __init__(self, args):
        super().__init__('fake_sensors')
        self.args = args
        self.traj = Trajectory(args.drive, args.circle_radius, args.speed,
                               args.start_delay)
        self.rng = np.random.default_rng(args.seed)
        self.start = self.get_clock().now()
        self.started = False

        self.half_x = args.room_size / 2.0
        self.half_y = args.room_size / 2.0
        self.pillars = [
            (np.array([1.2, 1.8]), 0.25),
            (np.array([-1.5, -0.8]), 0.35),
        ]

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.scan_pub = self.create_publisher(LaserScan, '/scan', sensor_qos)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.truth_pub = self.create_publisher(Odometry, '/ground_truth', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

        self.cmd = (0.0, 0.0)
        self.external_drive = args.drive == 'cmd_vel'
        # Odometry scale error, the classic wheel-radius miscalibration.
        self.odom_scale = args.odom_scale
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_update = 0.0

        self.imu_timer = self.create_timer(1.0 / args.imu_rate, self.publish_imu)
        self.scan_timer = self.create_timer(1.0 / args.scan_rate, self.publish_scan)
        self.odom_timer = self.create_timer(1.0 / 20.0, self.publish_odometry)

        self.get_logger().info(
            f'fake_sensors: room={args.room_size}x{args.room_size} m, '
            f'drive={args.drive} (r={args.circle_radius} m, v={args.speed} m/s), '
            f'scan={args.scan_rate} Hz x {args.rays} rays, imu={args.imu_rate} Hz, '
            f'odom scale error={args.odom_scale}')

    # ------------------------------------------------------------------ helpers
    def elapsed(self):
        # Do not start the clock until an estimator is actually subscribed:
        # otherwise the IMU initialisation window can be over before the SLAM
        # node has even discovered /scan, and the start is rejected as "moving".
        if not self.started:
            if self.scan_pub.get_subscription_count() == 0:
                self.start = self.get_clock().now()
                return 0.0
            self.started = True
            self.start = self.get_clock().now()
            self.get_logger().info(
                'subscriber connected: standing still for '
                f'{self.args.start_delay:.1f} s, then driving')
        return (self.get_clock().now() - self.start).nanoseconds * 1e-9

    def on_cmd(self, msg):
        if self.external_drive:
            self.cmd = (msg.linear.x, msg.angular.z)

    def cmd_vel_truth(self):
        """Ground-truth velocity, either scripted or from the last /cmd_vel."""
        if not self.external_drive:
            _, _, _, vx, vy, w, _, _ = self.traj.state(self.elapsed())
            yaw = self.traj.state(self.elapsed())[2]
            v = vx * math.cos(yaw) + vy * math.sin(yaw)
            return v, w
        return self.cmd

    def truth_pose(self, t):
        if self.external_drive:
            # Integrate the commanded velocity (Euler, deterministic).
            return self.odom_x / self.odom_scale, self.odom_y / self.odom_scale, self.odom_yaw
        return self.traj.state(t)[:3]

    # --------------------------------------------------------------------- IMU
    def publish_imu(self):
        t = self.elapsed()
        x, y, yaw, vx, vy, w, ax, ay = self.traj.state(t)
        if self.external_drive:
            v, w = self.cmd_vel_truth()
            ax = 0.0
            ay = v * w  # centripetal acceleration of the commanded motion
            yaw = self.odom_yaw
        rot = yaw_matrix(yaw)

        acc_world = np.array([ax, ay, 0.0]) - GRAVITY
        acc_body = rot.T @ acc_world
        gyro_body = rot.T @ np.array([0.0, 0.0, w])

        acc_body += self.rng.normal(0.0, self.args.imu_noise, 3)
        gyro_body += self.rng.normal(0.0, self.args.imu_noise, 3)
        # Constant bias: the ESKF has to estimate it away.
        gyro_body += np.array([0.0, 0.0, self.args.gyro_bias])

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.linear_acceleration.x = float(acc_body[0])
        msg.linear_acceleration.y = float(acc_body[1])
        msg.linear_acceleration.z = float(acc_body[2])
        msg.angular_velocity.x = float(gyro_body[0])
        msg.angular_velocity.y = float(gyro_body[1])
        msg.angular_velocity.z = float(gyro_body[2])
        msg.orientation.w = 1.0
        self.imu_pub.publish(msg)

    # ------------------------------------------------------------------- LIDAR
    def cast_room(self, origins, directions):
        """Box exit distance for every ray, vectorised."""
        eps = 1e-9
        ranges = np.full(len(directions), np.inf)

        for axis, half in ((0, self.half_x), (1, self.half_y)):
            d = directions[:, axis]
            moving = np.abs(d) > eps
            wall = np.where(d > 0.0, half, -half)
            t = np.where(moving, (wall - origins[:, axis]) / np.where(moving, d, 1.0), np.inf)
            ranges = np.minimum(ranges, t)

        for centre, radius in self.pillars:
            oc = origins[:, :2] - centre
            d = directions[:, :2]
            b = 2.0 * np.einsum('ij,ij->i', oc, d)
            c = np.einsum('ij,ij->i', oc, oc) - radius * radius
            disc = b * b - 4.0 * c  # |d| == 1
            hit = disc > 0.0
            t = np.where(hit, (-b - np.sqrt(np.where(hit, disc, 0.0))) / 2.0, np.inf)
            t = np.where(t > 1e-3, t, np.inf)
            ranges = np.minimum(ranges, t)

        return ranges

    def publish_scan(self):
        args = self.args
        n = args.rays
        t_start = self.elapsed()
        dt = 1.0 / args.scan_rate
        ray_times = t_start + np.arange(n) * dt / n

        # Pose of every ray: this is what makes de-skewing meaningful.
        poses = np.array([self.traj.state(t)[:3] for t in ray_times])
        if self.external_drive:
            poses = np.tile(np.array(self.truth_pose(t_start)), (n, 1))

        angles = -math.pi + 2.0 * math.pi * np.arange(n) / n
        local = np.stack(
            [np.cos(angles), np.sin(angles), np.zeros(n)], axis=1)  # in lidar_link

        origins = np.empty((n, 3))
        directions = np.empty((n, 3))
        for i in range(n):
            x, y, yaw = poses[i]
            rot = yaw_matrix(yaw)
            origins[i] = rot @ P_BASE_LIDAR + np.array([x, y, 0.0])
            directions[i] = rot @ (R_BASE_LIDAR @ local[i])

        ranges = self.cast_room(origins, directions)
        valid = np.isfinite(ranges) & (ranges <= args.max_range)
        ranges = np.where(valid, ranges, np.inf)
        ranges = ranges + self.rng.normal(0.0, args.range_noise, n)
        drop = self.rng.random(n) < args.drop_rate
        ranges = np.where(drop, np.inf, ranges)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = float(angles[0])
        msg.angle_max = float(angles[-1])
        msg.angle_increment = float(2.0 * math.pi / n)
        msg.time_increment = float(dt / n)
        msg.scan_time = float(dt)
        msg.range_min = 0.12
        msg.range_max = float(args.max_range)
        msg.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(msg)

    # ---------------------------------------------------------------- ODOMETRY
    def publish_odometry(self):
        t = self.elapsed()
        dt = t - self.last_update
        self.last_update = t

        if self.external_drive:
            v, w = self.cmd_vel_truth()
        else:
            _, _, _, vx, vy, w, _, _ = self.traj.state(t)
            v = math.hypot(vx, vy)

        # Dead reckoning with a wheel radius error and a little noise.
        self.odom_yaw += w * self.odom_scale * dt
        v_scaled = v * self.odom_scale
        self.odom_x += v_scaled * math.cos(self.odom_yaw) * dt
        self.odom_y += v_scaled * math.sin(self.odom_yaw) * dt

        stamp = self.get_clock().now().to_msg()

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.odom_x
        tf_msg.transform.translation.y = self.odom_y
        tf_msg.transform.rotation.z = math.sin(self.odom_yaw / 2.0)
        tf_msg.transform.rotation.w = math.cos(self.odom_yaw / 2.0)
        self.tf_broadcaster.sendTransform(tf_msg)

        odom = Odometry()
        odom.header = tf_msg.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation = tf_msg.transform.rotation
        odom.twist.twist.linear.x = v_scaled
        odom.twist.twist.angular.z = w * self.odom_scale
        self.odom_pub.publish(odom)

        x, y, yaw = self.truth_pose(t)
        truth = Odometry()
        truth.header.stamp = stamp
        truth.header.frame_id = 'map'
        truth.child_frame_id = 'base_link'
        truth.pose.pose.position.x = float(x)
        truth.pose.pose.position.y = float(y)
        truth.pose.pose.orientation.z = math.sin(yaw / 2.0)
        truth.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.truth_pub.publish(truth)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--room-size', type=float, default=6.0, help='square room side [m]')
    parser.add_argument('--drive', default='circle',
                        choices=['circle', 'static', 'cmd_vel'],
                        help='motion profile (cmd_vel listens to /cmd_vel)')
    parser.add_argument('--circle-radius', type=float, default=1.0)
    parser.add_argument('--speed', type=float, default=0.3, help='[m/s]')
    parser.add_argument('--scan-rate', type=float, default=10.0, help='[Hz]')
    parser.add_argument('--imu-rate', type=float, default=100.0, help='[Hz]')
    parser.add_argument('--rays', type=int, default=720)
    parser.add_argument('--max-range', type=float, default=10.0)
    parser.add_argument('--range-noise', type=float, default=0.01, help='[m]')
    parser.add_argument('--drop-rate', type=float, default=0.02)
    parser.add_argument('--imu-noise', type=float, default=0.002)
    parser.add_argument('--gyro-bias', type=float, default=0.002, help='[rad/s]')
    parser.add_argument('--odom-scale', type=float, default=1.02,
                        help='wheel radius scale error of the odometry')
    parser.add_argument('--start-delay', type=float, default=5.0,
                        help='[s] stand still first so the IMU can be initialised')
    parser.add_argument('--seed', type=int, default=42)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = FakeSensors(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
