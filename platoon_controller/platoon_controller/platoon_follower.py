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

Live tuning (no restart needed):
    ros2 param set /platoon_follower kp 2.5
    ros2 param set /platoon_follower target_distance 0.45

Hardware with reversed motors (drives backward on +x, e.g. swapped
DYNAMIXEL IDs):
    ros2 run platoon_controller platoon_follower --ros-args \
        -p invert_linear:=true
"""

import math
import time

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import PointStamped, Twist

from platoon_controller.control_math import (
    compute_linear_pd,
    compute_pure_pursuit_steering,
    fit_wheel_limits,
    rate_limit,
)


class PlatoonFollower(Node):
    """Coupled linear PD + Pure Pursuit angular follower node."""

    # Parameter name -> instance attribute. Drives both the initial read and
    # the live-update callback, so the two can never drift apart.
    PARAM_ATTRS = {
        'target_distance': 'setpoint',
        'kp': 'kp',
        'kd': 'kd',
        'max_speed': 'max_speed',
        'max_reverse': 'max_reverse',
        'deadband': 'deadband',
        'max_linear_accel': 'max_linear_accel',
        'k_align': 'k_align',
        'blend_speed': 'blend_speed',
        'max_angular_speed': 'max_angular_speed',
        'max_angular_accel': 'max_angular_accel',
        'turn_slowdown_gain': 'turn_slowdown_gain',
        'min_speed_scale': 'min_speed_scale',
        'max_wheel_speed': 'max_wheel_speed',
        'wheel_separation': 'wheel_separation',
        'emergency_distance': 'emergency',
        'target_timeout': 'timeout',
        'deriv_alpha': 'deriv_alpha',
        'invert_linear': 'invert_linear',
        'invert_angular': 'invert_angular',
    }

    def __init__(self):
        super().__init__('platoon_follower')

        # --- Linear PD parameters (Role 3) ---------------------------------
        self.declare_parameter('target_distance', 0.30)
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('max_speed', 0.22)
        self.declare_parameter('max_reverse', 0.08)
        self.declare_parameter('deadband', 0.015)
        self.declare_parameter('max_linear_accel', 0.6)

        # --- Pure Pursuit steering parameters (Role 4) ---------------------
        self.declare_parameter('k_align', 1.4)
        self.declare_parameter('blend_speed', 0.08)
        self.declare_parameter('angular_deadband_deg', 1.7)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('max_angular_accel', 2.0)
        self.declare_parameter('turn_slowdown_gain', 0.5)
        self.declare_parameter('min_speed_scale', 0.35)

        # --- Differential-drive limits -------------------------------------
        self.declare_parameter('max_wheel_speed', 0.22)
        self.declare_parameter('wheel_separation', 0.160)

        # --- Hardware direction corrections --------------------------------
        # Applied only at the output, so every piece of control logic above
        # stays in the standard REP-103 frame (x forward, +z turns left).
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_angular', False)

        # --- Safety & logging ----------------------------------------------
        self.declare_parameter('emergency_distance', 0.16)
        self.declare_parameter('target_timeout', 0.6)
        self.declare_parameter('deriv_alpha', 0.3)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('log_csv', '')

        for name, attr in self.PARAM_ATTRS.items():
            setattr(self, attr, self.get_parameter(name).value)

        # Stored in radians, so it needs converting rather than copying.
        self.angular_deadband = math.radians(
            self.get_parameter('angular_deadband_deg').value)

        # Changing the loop rate at runtime would mean rebuilding the timer,
        # so this one stays read-once.
        self.rate = self.get_parameter('rate').value

        self.add_on_set_parameters_callback(self.on_param_change)

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
        self.tracking = False
        self.last_status = 0.0

        # --- Optional CSV log for the calibration report -------------------
        self.csv = None
        path = self.get_parameter('log_csv').value
        if path:
            self.csv = open(path, 'w')
            self.csv.write(
                't,distance,bearing_deg,error,d_filtered,linear_x,angular_z\n'
            )
            self.t0 = self.now()
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

    # ----------------------------------------------------------------------
    def now(self):
        """Seconds from the ROS clock.

        Not ``time.time()``. The ROS clock honours ``use_sim_time``, so the
        watchdog measures the same seconds Gazebo publishes scans on rather
        than wall-clock seconds -- which diverge badly when physics runs
        below real time under software rendering.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    def on_param_change(self, params):
        """Apply a live ``ros2 param set`` without restarting the node.

        Tuning gains on hardware means watching the robot while changing one
        number. Restarting for each value loses the lock and the context.
        """
        for p in params:
            if p.name == 'angular_deadband_deg':
                self.angular_deadband = math.radians(p.value)
                continue

            attr = self.PARAM_ATTRS.get(p.name)
            if attr is not None:
                setattr(self, attr, p.value)

        return SetParametersResult(successful=True)

    # ----------------------------------------------------------------------
    def target_callback(self, msg):
        """Record target (x, y) in sensor frame and filtered range rate."""
        now = self.now()
        x = msg.point.x
        y = msg.point.y
        distance = math.hypot(x, y)

        if self.last_distance is not None and self.last_stamp is not None:
            dt = now - self.last_stamp
            # Upper bound guards against a clock step. The Pi has no
            # battery-backed clock, so when NTP first syncs after boot the
            # system time can jump by hours; an unguarded divide would then
            # produce a garbage rate that kd multiplies straight into the
            # wheels. Scans arrive at 5 Hz, so anything past 1 s is wrong.
            if 1e-6 < dt < 1.0:
                raw = (distance - self.last_distance) / dt
                self.d_filtered += self.deriv_alpha * (raw - self.d_filtered)

        self.target_x = x
        self.target_y = y
        self.last_distance = distance
        self.last_stamp = now
        self.distance = distance
        self.last_seen = now

    # ----------------------------------------------------------------------
    def target_age(self):
        """Seconds since the last target, or None if none has arrived."""
        if self.last_seen is None:
            return None
        return self.now() - self.last_seen

    def is_lost(self):
        """True when the target is missing, stale, or the clock jumped back.

        A negative age means the clock stepped backward mid-run. Treating it
        as fresh would keep the robot driving on an arbitrarily old reading,
        so it counts as lost.
        """
        age = self.target_age()
        return age is None or age < 0.0 or age > self.timeout

    # ----------------------------------------------------------------------
    def tick(self):
        """Run one control step and publish Twist to cmd_vel."""
        # Safety gate 1: no target, or target timed out.
        if self.is_lost():
            if self.tracking:
                self.get_logger().warn('Leader lost -- stopping.')
            self.tracking = False
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            self.send(0.0, 0.0)
            self.status()
            return

        # Safety gate 2: emergency backoff straight backward.
        # Not rate limited -- a collision response should be immediate.
        # Still tracking: the leader is visible, just too close.
        if self.distance < self.emergency:
            self.linear_speed = -self.max_reverse
            self.angular_speed = 0.0
            self.send(self.linear_speed, 0.0)
            self.get_logger().warn(
                f'Emergency backoff at {self.distance:.3f} m',
                throttle_duration_sec=1.0,
            )
            return

        if not self.tracking:
            self.get_logger().info(
                f'Leader locked at {self.distance:.3f} m')
        self.tracking = True

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

        # Scale v and omega together until both wheels are within their
        # limit, so a saturating outer wheel cannot silently flatten the arc.
        linear_x, angular_z = fit_wheel_limits(
            linear_x, angular_z,
            max_wheel_speed=self.max_wheel_speed,
            wheel_separation=self.wheel_separation,
        )

        dt = 1.0 / self.rate

        # Limit how fast omega may change. A spinning LiDAR smears its
        # sweep when the robot rotates mid-scan, so an abrupt omega
        # corrupts the very bearing that produced it. See rate_limit().
        angular_z = rate_limit(
            self.angular_speed, angular_z, self.max_angular_accel, dt)

        # Limit linear acceleration too, so a step command cannot slip the
        # wheels -- slip corrupts odometry and the gap estimate with it.
        linear_x = rate_limit(
            self.linear_speed, linear_x, self.max_linear_accel, dt)

        self.linear_speed = linear_x
        self.angular_speed = angular_z
        self.send(linear_x, angular_z)

        bearing_deg = math.degrees(bearing)
        self.get_logger().debug(
            f'dist {self.distance:5.3f} m | brg {bearing_deg:+6.2f} deg | '
            f'err {error:+6.3f} | v {linear_x:+6.3f} | w {angular_z:+6.3f}'
        )
        self.status(bearing_deg, error)

        if self.csv:
            self.csv.write(
                f'{self.now() - self.t0:.3f},{self.distance:.4f},'
                f'{bearing_deg:.3f},{error:.4f},{self.d_filtered:.4f},'
                f'{linear_x:.4f},{angular_z:.4f}\n'
            )

    # ----------------------------------------------------------------------
    def send(self, linear, angular):
        """Publish a velocity command, applying hardware direction flips.

        Every command leaves through here, including the emergency backoff.
        That matters: on a robot whose motors run backward, an uncorrected
        backoff would drive the follower INTO the leader it is fleeing.
        """
        cmd = Twist()
        cmd.linear.x = -linear if self.invert_linear else linear
        cmd.angular.z = -angular if self.invert_angular else angular
        self.cmd_pub.publish(cmd)

    # ----------------------------------------------------------------------
    def status(self, bearing_deg=0.0, error=0.0):
        """One summary line per second, saying why if it is not tracking.

        A robot sitting still tells you nothing about which of several
        causes applies. Naming the reason -- and the command that would
        confirm it -- turns a guessing session into one check.
        """
        now = self.now()
        if now - self.last_status < 1.0:
            return
        self.last_status = now

        if self.tracking:
            self.get_logger().info(
                f'TRACKING | dist {self.distance:.3f} m '
                f'(err {error:+.3f}) | brg {bearing_deg:+.1f} deg | '
                f'v {self.linear_speed:+.3f} w {self.angular_speed:+.3f}'
            )
            return

        age = self.target_age()
        if age is None:
            reason = ('no target has ever arrived -- is scan_filter running? '
                      'check: ros2 topic hz platoon/target')
        elif age < 0.0:
            reason = 'clock stepped backward -- waiting for it to settle'
        else:
            reason = f'leader lost {age:.1f} s ago'

        self.get_logger().info(f'SEARCHING | {reason} | cmd 0')

    # ----------------------------------------------------------------------
    def stop(self):
        """Publish zero Twist on exit so the robot never coasts."""
        for _ in range(3):
            try:
                self.cmd_pub.publish(Twist())
            except Exception as exc:  # context may already be shut down
                print(f'[platoon_follower] could not send stop: {exc}')
                break
            time.sleep(0.02)
        if self.csv:
            self.csv.close()


def main(args=None):
    """Run the PlatoonFollower ROS 2 node."""
    # Disable rclpy's own SIGINT handler so Ctrl-C raises KeyboardInterrupt
    # while the context is still valid, leaving us able to publish a final
    # zero Twist. Without this the robot can keep the last command latched.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args,
                   signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
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
