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

from geometry_msgs.msg import PointStamped
from platoon_controller.control_math import (
    compute_search_center,
    extract_sector_hits,
    locate_target_cluster,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):

    def __init__(self):
        super().__init__('scan_filter')

        # --- Tunable parameters -------------------------------------------
        # Override at launch, e.g.:
        #   ros2 run platoon_controller scan_filter --ros-args \
        #       -p sector_half_angle_deg:=20.0 -p ema_alpha:=0.6
        self.declare_parameter('sector_half_angle_deg', 15.0)
        self.declare_parameter('max_track_range', 1.6)
        self.declare_parameter('cluster_tolerance', 0.10)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('ema_alpha', 0.4)
        self.declare_parameter('coast_frames', 3)
        self.declare_parameter('track_bearing', True)
        self.declare_parameter('max_track_angle_deg', 40.0)
        self.declare_parameter('close_lock_distance', 0.22)
        self.declare_parameter('max_jump_distance', 0.45)

        self.half_angle = math.radians(
            self.get_parameter('sector_half_angle_deg').value)
        self.max_range = self.get_parameter('max_track_range').value
        self.cluster_tol = self.get_parameter('cluster_tolerance').value
        self.min_points = self.get_parameter('min_cluster_points').value
        self.alpha = self.get_parameter('ema_alpha').value
        self.coast_frames = self.get_parameter('coast_frames').value
        self.track_bearing = self.get_parameter('track_bearing').value
        self.max_track_angle = math.radians(
            self.get_parameter('max_track_angle_deg').value)
        self.close_lock_distance = self.get_parameter(
            'close_lock_distance').value
        self.max_jump = self.get_parameter('max_jump_distance').value

        # --- Tracker state -------------------------------------------------
        self.distance = None     # smoothed range to the leading object (m)
        self.bearing = None      # smoothed angular offset (rad)
        self.misses = 0          # consecutive frames with no usable target

        # --- ROS plumbing --------------------------------------------------
        # qos_profile_sensor_data is BEST_EFFORT. A BEST_EFFORT subscriber can
        # receive from either a BEST_EFFORT or a RELIABLE publisher, so this
        # one profile works in Gazebo AND on the real LDS driver unchanged.
        # A RELIABLE subscriber would get nothing from the real robot.
        # Relative topic names allow multi-robot namespacing (e.g. /tb3_2).
        self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

        self.target_pub = self.create_publisher(
            PointStamped, 'platoon/target', 10)
        self.debug_pub = self.create_publisher(
            LaserScan, 'platoon/scan_filtered', 10)

        self.get_logger().info(
            f'Scan filter up. Sector +/-'
            f'{math.degrees(self.half_angle):.0f} deg, '
            f'tracking out to {self.max_range:.2f} m.'
        )

    # ----------------------------------------------------------------------
    # Stage 1: angular window + validity gate
    # ----------------------------------------------------------------------
    def get_search_center(self):
        """Center the sector on the locked bearing to hold lock in turns."""
        return compute_search_center(
            bearing=self.bearing,
            distance=self.distance,
            track_bearing=self.track_bearing,
            max_track_angle=self.max_track_angle,
            close_lock_distance=self.close_lock_distance,
        )

    def extract_sector(self, msg):
        """Return [(index, angle, range)] for valid beams inside the window."""
        return extract_sector_hits(
            ranges=msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min,
            range_max=msg.range_max,
            max_track_range=self.max_range,
            half_angle=self.half_angle,
            center_angle=self.get_search_center(),
        )

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
        result = locate_target_cluster(
            hits, self.cluster_tol, self.min_points)
        if result is None:
            return None

        distance, bearing, face_size = result

        # Reject sudden target jumps (e.g. a wall clipping the edge of the
        # tracking cone) while already locked onto a leader.
        if (
            self.max_jump > 0.0
            and self.distance is not None
            and self.bearing is not None
        ):
            dx = (
                distance * math.cos(bearing)
                - self.distance * math.cos(self.bearing)
            )
            dy = (
                distance * math.sin(bearing)
                - self.distance * math.sin(self.bearing)
            )
            if math.hypot(dx, dy) > self.max_jump:
                self.get_logger().warn(
                    f'Rejected target jump of {math.hypot(dx, dy):.2f} m',
                    throttle_duration_sec=1.0,
                )
                return None

        return distance, bearing, face_size

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
            if self.handle_miss():
                self.publish_target(msg.header)
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
