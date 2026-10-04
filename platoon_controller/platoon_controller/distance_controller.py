#!/usr/bin/env python3
"""
Linear distance controller  --  Robot Platoon project, Role 3.

Milestone : Single-Robot Distance-Lock Simulation
Step      : 2  (close the loop -- this node drives the ROBOT)

Holds a fixed gap to whatever scan_filter is tracking, using a PD law on

    e(t) = distance - target_distance

e > 0 means the target is too far, so drive forward.
e < 0 means it is too close, so decelerate or back up.

Only linear velocity is commanded. angular.z stays zero; steering is Role 4's
job and lands in a later milestone.

Subscribes
    /platoon/target   geometry_msgs/PointStamped   from scan_filter

Publishes
    /cmd_vel          geometry_msgs/Twist          to the robot
"""

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Twist


class DistanceController(Node):

    def __init__(self):
        super().__init__('distance_controller')

        # --- Setpoint and gains -------------------------------------------
        self.declare_parameter('target_distance', 0.30)
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('kd', 0.5)

        # --- Limits --------------------------------------------------------
        # 0.22 m/s is the burger's rated maximum. Reverse is capped lower:
        # the LiDAR cannot see behind the robot, so backing up fast is
        # uncontrolled motion into unknown space.
        self.declare_parameter('max_speed', 0.22)
        self.declare_parameter('max_reverse', 0.08)
        self.declare_parameter('deadband', 0.015)

        # --- Safety --------------------------------------------------------
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
        self.emergency = self.get_parameter('emergency_distance').value
        self.timeout = self.get_parameter('target_timeout').value
        self.deriv_alpha = self.get_parameter('deriv_alpha').value
        self.rate = self.get_parameter('rate').value

        # --- State ---------------------------------------------------------
        self.distance = None       # latest range to target (m)
        self.last_distance = None  # previous range, for the derivative
        self.last_stamp = None     # when that previous range arrived (s)
        self.last_seen = None      # when ANY target last arrived (s)
        self.d_filtered = 0.0      # low-passed closing rate (m/s)
        self.speed = 0.0           # last published command (m/s)

        # --- Optional CSV log for the calibration report -------------------
        self.csv = None
        path = self.get_parameter('log_csv').value
        if path:
            self.csv = open(path, 'w')
            self.csv.write('t,distance,error,d_filtered,speed\n')
            self.t0 = time.time()
            self.get_logger().info(f'Logging run to {path}')

        # --- ROS plumbing --------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            PointStamped, '/platoon/target', self.target_callback, 10)

        self.create_timer(1.0 / self.rate, self.tick)

        self.get_logger().info(
            f'Distance controller up. Setpoint {self.setpoint:.3f} m, '
            f'kp={self.kp}, kd={self.kd}. Waiting for /platoon/target...'
        )

    # ----------------------------------------------------------------------
    def target_callback(self, msg):
        """Range to the tracked object, and its rate of change."""
        now = time.time()

        # scan_filter publishes the target as a point in the sensor frame,
        # so the range is the magnitude, not just x. At small bearings the
        # difference is tiny, but using hypot keeps this correct once Role 4
        # starts steering and bearings get large.
        distance = math.hypot(msg.point.x, msg.point.y)

        if self.last_distance is not None and self.last_stamp is not None:
            dt = now - self.last_stamp
            if dt > 1e-6:
                raw = (distance - self.last_distance) / dt
                # The LiDAR runs at 5 Hz, so a raw derivative over a 0.2 s
                # window amplifies sensor noise badly. Low-pass it, or kd
                # will make the robot buzz instead of damp.
                self.d_filtered += self.deriv_alpha * (raw - self.d_filtered)

        self.last_distance = distance
        self.last_stamp = now
        self.distance = distance
        self.last_seen = now

    # ----------------------------------------------------------------------
    def compute_speed(self):
        """PD law plus limits. Returns the linear velocity to command."""
        error = self.distance - self.setpoint

        # Inside the deadband, stop. Without this the robot hunts around the
        # setpoint forever, creeping back and forth on sensor noise.
        if abs(error) < self.deadband:
            self.d_filtered *= 0.5
            return 0.0, error

        speed = self.kp * error + self.kd * self.d_filtered

        return max(-self.max_reverse, min(self.max_speed, speed)), error

    # ----------------------------------------------------------------------
    def tick(self):
        cmd = Twist()
        now = time.time()

        # --- Safety gate 1: no target, or a stale one ----------------------
        # Covers two different situations that look identical from here: the
        # target genuinely left the sector, OR it came closer than the LDS's
        # 0.12 m minimum range and every beam went invalid. Driving forward
        # would be wrong in the second case, so both stop.
        if self.last_seen is None or (now - self.last_seen) > self.timeout:
            self.speed = 0.0
            self.cmd_pub.publish(cmd)
            if self.last_seen is not None:
                self.get_logger().warn(
                    'No target -- holding still.', throttle_duration_sec=2.0)
            return

        # --- Safety gate 2: too close ---------------------------------------
        if self.distance < self.emergency:
            self.speed = -self.max_reverse
            cmd.linear.x = self.speed
            self.cmd_pub.publish(cmd)
            self.get_logger().warn(
                f'Emergency backoff at {self.distance:.3f} m',
                throttle_duration_sec=1.0)
            return

        # --- Normal control -------------------------------------------------
        self.speed, error = self.compute_speed()
        cmd.linear.x = self.speed
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'dist {self.distance:5.3f} | err {error:+6.3f} | '
            f'rate {self.d_filtered:+6.3f} | cmd {self.speed:+6.3f} m/s',
            throttle_duration_sec=0.25
        )

        if self.csv:
            self.csv.write(
                f'{now - self.t0:.3f},{self.distance:.4f},{error:.4f},'
                f'{self.d_filtered:.4f},{self.speed:.4f}\n'
            )

    # ----------------------------------------------------------------------
    def stop(self):
        """Always leave the robot stationary on exit.

        Without this, Ctrl+C leaves the last non-zero Twist latched and the
        robot keeps driving. On real hardware that is how a TurtleBot ends up
        off the edge of a table.
        """
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            time.sleep(0.02)
        if self.csv:
            self.csv.close()


def main(args=None):
    rclpy.init(args=args)
    node = DistanceController()

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
