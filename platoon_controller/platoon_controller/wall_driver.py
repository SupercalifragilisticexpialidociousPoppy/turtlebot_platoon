#!/usr/bin/env python3
"""
Dummy wall driver  --  Robot Platoon project.

Milestone : Single-Robot Distance-Lock Simulation
Step      : 2a  (the moving target -- this node drives the WALL, not the robot)

Publishes a slow, bounded, random back-and-forth velocity profile to the dummy
wall so the follower has something non-trivial to lock onto.

The profile is piecewise-constant velocity with acceleration limiting:

    - every few seconds, pick a new random target speed
    - ramp toward it at a fixed acceleration rather than stepping
    - reverse when the wall approaches either end of its travel

Acceleration limiting is the part that matters for your results. A wall that
step-changes velocity is an impulse, and tuning a PD loop against an impulse
gives gains that look great in sim and oscillate badly behind a real robot,
which accelerates smoothly. Ramping keeps the target physically plausible.

Subscribes
    /wall/odom      nav_msgs/Odometry       wall position, for bounds checking

Publishes
    /wall/cmd_vel   geometry_msgs/Twist     velocity command to the wall
"""

import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class WallDriver(Node):

    def __init__(self):
        super().__init__('wall_driver')

        # --- Tunable parameters -------------------------------------------
        #   ros2 run platoon_controller wall_driver --ros-args \
        #       -p max_speed:=0.05 -p x_min:=0.5 -p x_max:=2.0
        self.declare_parameter('x_min', 0.6)
        self.declare_parameter('x_max', 2.5)
        self.declare_parameter('max_speed', 0.08)
        self.declare_parameter('accel', 0.05)
        self.declare_parameter('segment_min', 2.0)
        self.declare_parameter('segment_max', 6.0)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('seed', 0)

        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.max_speed = self.get_parameter('max_speed').value
        self.accel = self.get_parameter('accel').value
        self.segment_min = self.get_parameter('segment_min').value
        self.segment_max = self.get_parameter('segment_max').value
        self.rate = self.get_parameter('rate').value

        # seed=0 means "different every run". Set a real seed when you are
        # comparing PD gain sets -- otherwise each run faces a different
        # disturbance and the calibration report compares nothing.
        seed = self.get_parameter('seed').value
        self.rng = random.Random(seed if seed else None)

        # --- State ---------------------------------------------------------
        self.x = None            # wall position from odom (m)
        self.speed = 0.0         # current commanded speed (m/s)
        self.target_speed = 0.0  # speed we are ramping toward (m/s)
        self.segment_left = 0.0  # time until the next random re-pick (s)

        # --- ROS plumbing --------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, '/wall/cmd_vel', 10)
        self.create_subscription(Odometry, '/wall/odom', self.odom_callback, 10)

        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'Wall driver up. Travel {self.x_min:.2f}..{self.x_max:.2f} m, '
            f'max speed {self.max_speed:.3f} m/s. Waiting for /wall/odom...'
        )

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x

    def pick_target_speed(self):
        """Choose a new random cruise speed for the next segment."""
        speed = self.rng.uniform(-self.max_speed, self.max_speed)

        # Reject near-zero picks. A wall that keeps choosing 0.003 m/s just
        # sits there and tells you nothing about the controller's response.
        if abs(speed) < 0.2 * self.max_speed:
            speed = 0.2 * self.max_speed * (1 if speed >= 0 else -1)

        return speed

    def apply_bounds(self):
        """Force the wall back inside its travel limits.

        Overrides the random choice near the ends. Without this the wall
        wanders out of LiDAR range and the filter reports `Target lost`,
        which is correct behaviour but useless as a test.
        """
        if self.x is None:
            return

        if self.x >= self.x_max and self.target_speed > 0.0:
            self.target_speed = -abs(self.pick_target_speed())
            self.segment_left = self.rng.uniform(
                self.segment_min, self.segment_max)

        elif self.x <= self.x_min and self.target_speed < 0.0:
            self.target_speed = abs(self.pick_target_speed())
            self.segment_left = self.rng.uniform(
                self.segment_min, self.segment_max)

    def tick(self):
        # Hold still until Gazebo reports where the wall actually is.
        if self.x is None:
            self.cmd_pub.publish(Twist())
            return

        # Re-roll the cruise speed when the current segment expires.
        self.segment_left -= self.dt
        if self.segment_left <= 0.0:
            self.target_speed = self.pick_target_speed()
            self.segment_left = self.rng.uniform(
                self.segment_min, self.segment_max)

        self.apply_bounds()

        # Ramp toward the target instead of stepping to it.
        step = self.accel * self.dt
        if self.speed < self.target_speed:
            self.speed = min(self.speed + step, self.target_speed)
        else:
            self.speed = max(self.speed - step, self.target_speed)

        cmd = Twist()
        cmd.linear.x = self.speed
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'wall x {self.x:5.3f} m | speed {self.speed:+6.3f} m/s '
            f'-> target {self.target_speed:+6.3f}',
            throttle_duration_sec=1.0
        )

    def stop(self):
        """Leave the wall stationary rather than coasting on last command."""
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = WallDriver()

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
