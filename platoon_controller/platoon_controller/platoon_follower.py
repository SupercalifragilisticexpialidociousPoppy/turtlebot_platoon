#!/usr/bin/env python3
"""
Unified platoon tracking node  --  Robot Platoon project, Roles 3 + 4.

Milestones : Twin-Robot Coupling & Full Platoon Deployment
Combines linear PD gap maintenance (Role 3) with Pure Pursuit angular steering
(Role 4) into a single deployable node for any following TurtleBot3.

Subscribes
    platoon/target   geometry_msgs/PointStamped   from scan_filter

Publishes
    cmd_vel          geometry_msgs/Twist          to the robot
"""

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Twist

from platoon_controller.control_math import (
    compute_linear_pd,
    compute_pure_pursuit_steering,
    rate_limit,
)


class PlatoonFollower(Node):
    """Coupled linear PD + Pure Pursuit angular follower node."""

    def __init__(self):
        super().__init__('platoon_follower')

        # --- Linear PD parameters (Role 3) ---------------------------------
        self.declare_parameter('target_distance', 0.30)
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('max_speed', 0.22)
        self.declare_parameter('max_reverse', 0.08)
        self.declare_parameter('deadband', 0.015)

        # --- Pure Pursuit steering parameters (Role 4) ---------------------
        self.declare_parameter('k_align', 1.4)
        self.declare_parameter('blend_speed', 0.08)
        self.declare_parameter('angular_deadband_deg', 1.7)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('max_angular_accel', 2.0)
        self.declare_parameter('turn_slowdown_gain', 0.5)
        self.declare_parameter('min_speed_scale', 0.35)

        # --- Safety & logging ----------------------------------------------
        self.declare_parameter('emergency_distance', 0.16)
        self.declare_parameter('target_timeout', 0.6)
        self.declare_parameter('deriv_alpha', 0.3)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('log_csv', '')

        self.setpoint = self.get_parameter('target_distance').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_reverse = self.get_parameter('max_reverse').value
        self.deadband = self.get_parameter('deadband').value

        self.k_align = self.get_parameter('k_align').value
        self.blend_speed = self.get_parameter('blend_speed').value
        self.angular_deadband = math.radians(
            self.get_parameter('angular_deadband_deg').value)
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.max_angular_accel = self.get_parameter(
            'max_angular_accel').value
        self.turn_slowdown_gain = self.get_parameter(
            'turn_slowdown_gain').value
        self.min_speed_scale = self.get_parameter('min_speed_scale').value

        self.emergency = self.get_parameter('emergency_distance').value
        self.timeout = self.get_parameter('target_timeout').value
        self.deriv_alpha = self.get_parameter('deriv_alpha').value
        self.rate = self.get_parameter('rate').value

        # --- State ---------------------------------------------------------
        self.target_x = None
        self.target_y = None
        self.distance = None
        self.last_distance = None
        self.last_stamp = None
        self.last_seen = None
        self.d_filtered = 0.0
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # --- Optional CSV log for the calibration report -------------------
        self.csv = None
        path = self.get_parameter('log_csv').value
        if path:
            self.csv = open(path, 'w')
            self.csv.write(
                't,distance,bearing_deg,error,d_filtered,linear_x,angular_z\n'
            )
            self.t0 = time.time()
            self.get_logger().info(f'Logging run to {path}')

        # --- ROS plumbing --------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(
            PointStamped, 'platoon/target', self.target_callback, 10)

        self.create_timer(1.0 / self.rate, self.tick)

        self.get_logger().info(
            f'Platoon follower up. Setpoint {self.setpoint:.3f} m, '
            f'kp={self.kp}, kd={self.kd}, k_align={self.k_align}. '
            'Waiting for platoon/target...'
        )

    def target_callback(self, msg):
        """Record target (x, y) in sensor frame and filtered range rate."""
        now = time.time()
        x = msg.point.x
        y = msg.point.y
        distance = math.hypot(x, y)

        if self.last_distance is not None and self.last_stamp is not None:
            dt = now - self.last_stamp
            if dt > 1e-6:
                raw = (distance - self.last_distance) / dt
                self.d_filtered += self.deriv_alpha * (raw - self.d_filtered)

        self.target_x = x
        self.target_y = y
        self.last_distance = distance
        self.last_stamp = now
        self.distance = distance
        self.last_seen = now

    def tick(self):
        """Run one 20 Hz control step and publish Twist to cmd_vel."""
        cmd = Twist()
        now = time.time()

        # Safety gate 1: no target, or target timed out.
        if self.last_seen is None or (now - self.last_seen) > self.timeout:
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            self.cmd_pub.publish(cmd)
            if self.last_seen is not None:
                self.get_logger().warn(
                    'No target -- holding still.', throttle_duration_sec=2.0)
            return

        # Safety gate 2: emergency backoff straight backward.
        if self.distance < self.emergency:
            self.linear_speed = -self.max_reverse
            self.angular_speed = 0.0
            cmd.linear.x = self.linear_speed
            self.cmd_pub.publish(cmd)
            self.get_logger().warn(
                f'Emergency backoff at {self.distance:.3f} m',
                throttle_duration_sec=1.0,
            )
            return

        base_speed, error, in_deadband = compute_linear_pd(
            distance=self.distance,
            setpoint=self.setpoint,
            d_filtered=self.d_filtered,
            kp=self.kp,
            kd=self.kd,
            deadband=self.deadband,
            max_speed=self.max_speed,
            max_reverse=self.max_reverse,
        )
        if in_deadband:
            self.d_filtered *= 0.5

        linear_x, angular_z, bearing = compute_pure_pursuit_steering(
            x=self.target_x,
            y=self.target_y,
            linear_speed=base_speed,
            k_align=self.k_align,
            blend_speed=self.blend_speed,
            angular_deadband=self.angular_deadband,
            max_angular_speed=self.max_angular_speed,
            turn_slowdown_gain=self.turn_slowdown_gain,
            min_speed_scale=self.min_speed_scale,
        )

        # Limit how fast omega may change. A spinning LiDAR smears its
        # sweep when the robot rotates mid-scan, so an abrupt omega
        # corrupts the very bearing that produced it. See rate_limit().
        angular_z = rate_limit(
            self.angular_speed, angular_z,
            self.max_angular_accel, 1.0 / self.rate)

        self.linear_speed = linear_x
        self.angular_speed = angular_z
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

        bearing_deg = math.degrees(bearing)
        self.get_logger().info(
            f'dist {self.distance:5.3f} m | brg {bearing_deg:+6.2f} deg | '
            f'err {error:+6.3f} | v {linear_x:+6.3f} | w {angular_z:+6.3f}',
            throttle_duration_sec=0.25,
        )

        if self.csv:
            self.csv.write(
                f'{now - self.t0:.3f},{self.distance:.4f},'
                f'{bearing_deg:.3f},{error:.4f},{self.d_filtered:.4f},'
                f'{linear_x:.4f},{angular_z:.4f}\n'
            )

    def stop(self):
        """Publish zero Twist on exit so the robot never coasts."""
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            time.sleep(0.02)
        if self.csv:
            self.csv.close()


def main(args=None):
    """Run the PlatoonFollower ROS 2 node."""
    rclpy.init(args=args)
    node = PlatoonFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
