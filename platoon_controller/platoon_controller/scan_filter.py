#!/usr/bin/env python3
"""
Spatial LiDAR scan filter  --  Robot Platoon project, Role 2.

Milestone : Single-Robot Distance-Lock Simulation
Step      : 1 of 3  (perception only -- this node never moves the robot)

The node turns the raw 360-degree LDS sweep into two numbers, once per frame:

    distance  -- how far away the leading object is, in metres
    bearing   -- how far off our centre line it sits, in radians
                 (positive = to the LEFT, per REP-103)

Step 2 (the PD loop, Role 3) will consume `distance`.
Step 3 / the real platoon (pure pursuit, Role 4) will consume `bearing`.

Subscribes
    /scan                       sensor_msgs/LaserScan

Publishes
    /platoon/target             geometry_msgs/PointStamped
        The leading object as a point in the base_scan frame. Add this as a
        Point display in RViz to literally watch the tracker lock on.
    /platoon/scan_filtered      sensor_msgs/LaserScan
        The raw sweep with everything outside the sector blanked to +inf.
        Add as a second LaserScan display in RViz to see the window itself.
"""

import math
from statistics import median

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan


def normalize_angle(angle):
    """Wrap any angle into the range (-pi, +pi].

    The TurtleBot3 LDS reports angle_min = 0.0 and angle_max = 2*pi, so beams
    to the robot's RIGHT arrive as values near 6.28, not as negatives. Without
    this wrap, a symmetric test like `-15deg <= angle <= +15deg` silently keeps
    only the left half of the forward cone.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


class ScanFilter(Node):

    def __init__(self):
        super().__init__('scan_filter')

        # --- Tunable parameters -------------------------------------------
        # Override at launch, e.g.:
        #   ros2 run platoon_controller scan_filter --ros-args \
        #       -p sector_half_angle_deg:=20.0 -p ema_alpha:=0.6
        self.declare_parameter('sector_half_angle_deg', 15.0)
        self.declare_parameter('max_track_range', 3.0)
        self.declare_parameter('cluster_tolerance', 0.10)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('ema_alpha', 0.4)
        self.declare_parameter('coast_frames', 5)

        self.half_angle = math.radians(
            self.get_parameter('sector_half_angle_deg').value)
        self.max_range = self.get_parameter('max_track_range').value
        self.cluster_tol = self.get_parameter('cluster_tolerance').value
        self.min_points = self.get_parameter('min_cluster_points').value
        self.alpha = self.get_parameter('ema_alpha').value
        self.coast_frames = self.get_parameter('coast_frames').value

        # --- Tracker state -------------------------------------------------
        self.distance = None     # smoothed range to the leading object (m)
        self.bearing = None      # smoothed angular offset (rad)
        self.misses = 0          # consecutive frames with no usable target

        # --- ROS plumbing --------------------------------------------------
        # qos_profile_sensor_data is BEST_EFFORT. A BEST_EFFORT subscriber can
        # receive from either a BEST_EFFORT or a RELIABLE publisher, so this
        # one profile works in Gazebo AND on the real LDS driver unchanged.
        # A RELIABLE subscriber would get nothing from the real robot.
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.target_pub = self.create_publisher(
            PointStamped, '/platoon/target', 10)
        self.debug_pub = self.create_publisher(
            LaserScan, '/platoon/scan_filtered', 10)

        self.get_logger().info(
            f'Scan filter up. Sector +/-'
            f'{math.degrees(self.half_angle):.0f} deg, '
            f'tracking out to {self.max_range:.2f} m.'
        )

    # ----------------------------------------------------------------------
    # Stage 1: angular window + validity gate
    # ----------------------------------------------------------------------
    def extract_sector(self, msg):
        """Return [(index, angle, range)] for valid beams inside the window."""
        hits = []

        # Gazebo marks no-return beams as +inf; some builds use 0.0 instead.
        # Clamping the floor to range_min catches both, plus any beam that
        # reads closer than the sensor is physically able to resolve.
        near = max(msg.range_min, 0.02)
        far = min(msg.range_max, self.max_range)

        for i, r in enumerate(msg.ranges):
            angle = normalize_angle(msg.angle_min + i * msg.angle_increment)

            if abs(angle) > self.half_angle:
                continue
            if not math.isfinite(r):
                continue
            if r < near or r > far:
                continue

            hits.append((i, angle, r))

        return hits

    # ----------------------------------------------------------------------
    # Stage 2: outlier rejection + target extraction
    # ----------------------------------------------------------------------
    def locate_target(self, hits):
        """Collapse the sector into one (distance, bearing, size) estimate.

        A plain min() over the sector is what you reach for first, and it is
        exactly what a single spurious short reading hijacks -- one floor
        reflection and the controller thinks a wall appeared at 8 cm.

        So instead: find the nearest return, keep only the beams lying within
        cluster_tolerance of it (that set is the near FACE of the leading
        object), and demand that the face be at least min_cluster_points wide.
        An isolated spike fails the width test and is discarded. A real wall
        at 30 cm spans ~30 beams and sails through.
        """
        nearest = min(r for _, _, r in hits)
        face = [(a, r) for _, a, r in hits if r <= nearest + self.cluster_tol]

        if len(face) < self.min_points:
            return None

        # Median over the face, not mean: immune to a straggler beam that
        # clipped an edge and came back long.
        distance = median([r for _, r in face])

        # Centroid of the face gives the angular offset Role 4 needs.
        bearing = sum(a for a, _ in face) / len(face)

        return distance, bearing, len(face)

    # ----------------------------------------------------------------------
    # Stage 3: temporal smoothing + dropout coasting
    # ----------------------------------------------------------------------
    def update_tracker(self, distance, bearing):
        """Exponential moving average. alpha=1.0 disables smoothing."""
        if self.distance is None:
            self.distance = distance
            self.bearing = bearing
        else:
            self.distance += self.alpha * (distance - self.distance)
            self.bearing += self.alpha * (bearing - self.bearing)

    def handle_miss(self):
        """Hold the last estimate briefly, then declare the target lost.

        The LDS drops the occasional frame. Zeroing the target on a single
        bad frame would make the PD controller slam the brakes and then jump
        forward again, which is precisely the accordion oscillation the
        proposal asks Role 3 to eliminate. Coasting rides out the gap.
        """
        self.misses += 1

        if self.distance is not None and self.misses < self.coast_frames:
            self.get_logger().warn(
                f'No return in sector -- coasting on last estimate '
                f'({self.misses}/{self.coast_frames})'
            )
            return True

        if self.distance is not None:
            self.get_logger().warn('Target lost.')

        self.distance = None
        self.bearing = None
        return False

    # ----------------------------------------------------------------------
    # Main callback
    # ----------------------------------------------------------------------
    def scan_callback(self, msg):
        hits = self.extract_sector(msg)
        self.publish_debug_scan(msg, hits)

        target = self.locate_target(hits) if hits else None

        if target is None:
            self.handle_miss()
            return

        distance, bearing, face_size = target
        self.misses = 0
        self.update_tracker(distance, bearing)

        self.get_logger().info(
            f'distance {self.distance:.3f} m | '
            f'bearing {math.degrees(self.bearing):+6.2f} deg | '
            f'sector {len(hits):3d} beams, face {face_size:3d}'
        )

        self.publish_target(msg.header)

    # ----------------------------------------------------------------------
    # Outputs
    # ----------------------------------------------------------------------
    def publish_target(self, header):
        """Leading object as a point in the sensor frame, for RViz + step 2."""
        pt = PointStamped()
        pt.header = header
        pt.point.x = self.distance * math.cos(self.bearing)
        pt.point.y = self.distance * math.sin(self.bearing)
        pt.point.z = 0.0
        self.target_pub.publish(pt)

    def publish_debug_scan(self, msg, hits):
        """Republish the sweep with everything outside the sector blanked."""
        dbg = LaserScan()
        dbg.header = msg.header
        dbg.angle_min = msg.angle_min
        dbg.angle_max = msg.angle_max
        dbg.angle_increment = msg.angle_increment
        dbg.time_increment = msg.time_increment
        dbg.scan_time = msg.scan_time
        dbg.range_min = msg.range_min
        dbg.range_max = msg.range_max

        blanked = [float('inf')] * len(msg.ranges)
        for i, _, r in hits:
            blanked[i] = r

        dbg.ranges = blanked
        dbg.intensities = []
        self.debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
