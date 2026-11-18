"""Unit tests for platoon pure pursuit, PD control, and scan filtering."""

import math
import unittest

from platoon_controller.control_math import (
    compute_linear_pd,
    compute_pure_pursuit_steering,
    extract_sector_hits,
    locate_target_cluster,
    normalize_angle,
)


class TestNormalizeAngle(unittest.TestCase):
    """Tests for 0..2*pi to (-pi, +pi] wrapping."""

    def test_wraps_right_side_lds_angles(self):
        """Beams just below 2*pi map to small negative angles."""
        angle = normalize_angle(2.0 * math.pi - math.radians(10.0))
        self.assertAlmostEqual(angle, -math.radians(10.0), places=6)

    def test_keeps_forward_zero(self):
        """Zero radians remains zero."""
        self.assertAlmostEqual(normalize_angle(0.0), 0.0, places=6)


class TestLinearPD(unittest.TestCase):
    """Tests for the 1D distance gap PD controller."""

    def test_stops_inside_deadband(self):
        """Target within deadband produces zero speed."""
        speed, error, in_db = compute_linear_pd(
            distance=0.31,
            setpoint=0.30,
            d_filtered=0.05,
            kp=2.0,
            kd=0.5,
            deadband=0.015,
            max_speed=0.22,
            max_reverse=0.08,
        )
        self.assertTrue(in_db)
        self.assertEqual(speed, 0.0)
        self.assertAlmostEqual(error, 0.01, places=6)

    def test_clamps_forward_and_reverse(self):
        """Output is bounded to [ -max_reverse, +max_speed ]."""
        fwd, _, in_db_fwd = compute_linear_pd(
            2.0, 0.30, 0.0, 2.0, 0.5, 0.015, 0.22, 0.08)
        rev, _, in_db_rev = compute_linear_pd(
            0.18, 0.30, 0.0, 2.0, 0.5, 0.015, 0.22, 0.08)
        self.assertFalse(in_db_fwd)
        self.assertFalse(in_db_rev)
        self.assertAlmostEqual(fwd, 0.22, places=6)
        self.assertAlmostEqual(rev, -0.08, places=6)


class TestPurePursuitSteering(unittest.TestCase):
    """Tests for Pure Pursuit curvature and stationary alignment."""

    def test_straight_target_zero_angular(self):
        """Target directly ahead yields zero angular velocity."""
        v, w, brg = compute_pure_pursuit_steering(
            x=0.60, y=0.0, linear_speed=0.20)
        self.assertAlmostEqual(v, 0.20, places=6)
        self.assertEqual(w, 0.0)
        self.assertAlmostEqual(brg, 0.0, places=6)

    def test_left_turn_pure_pursuit_curvature(self):
        """Positive y (left) produces positive angular.z matching kappa*v."""
        x, y = 0.50, 0.20
        v_in = 0.16
        v_out, w, brg = compute_pure_pursuit_steering(
            x=x,
            y=y,
            linear_speed=v_in,
            blend_speed=0.05,
            turn_slowdown_gain=0.0,
        )
        expected_kappa = (2.0 * y) / (x * x + y * y)
        self.assertAlmostEqual(v_out, v_in, places=6)
        self.assertGreater(brg, 0.0)
        self.assertAlmostEqual(w, v_out * expected_kappa, places=6)

    def test_right_turn_negative_angular(self):
        """Negative y (right) produces negative angular.z."""
        _, w, brg = compute_pure_pursuit_steering(
            x=0.50, y=-0.20, linear_speed=0.16)
        self.assertLess(brg, 0.0)
        self.assertLess(w, 0.0)

    def test_stationary_alignment_turns_toward_leader(self):
        """When stopped in distance deadband, robot still aligns bearing."""
        v_out, w, _ = compute_pure_pursuit_steering(
            x=0.28, y=0.12, linear_speed=0.0, k_align=1.4)
        self.assertEqual(v_out, 0.0)
        self.assertGreater(w, 0.0)

    def test_reverse_does_not_invert_steering_sign(self):
        """While backing up slightly, robot still turns toward the target."""
        v_out, w, _ = compute_pure_pursuit_steering(
            x=0.20, y=0.10, linear_speed=-0.05, k_align=1.4)
        self.assertEqual(v_out, -0.05)
        self.assertGreater(w, 0.0)

    def test_clamps_max_angular_speed(self):
        """Sharp close target cannot exceed max_angular_speed."""
        _, w, _ = compute_pure_pursuit_steering(
            x=0.05,
            y=0.25,
            linear_speed=0.22,
            max_angular_speed=1.2,
            turn_slowdown_gain=0.0,
        )
        self.assertAlmostEqual(w, 1.2, places=6)

    def test_turn_slowdown_reduces_forward_speed(self):
        """Large bearing reduces linear speed while keeping positive turn."""
        v_out, w, _ = compute_pure_pursuit_steering(
            x=0.20,
            y=0.20,
            linear_speed=0.20,
            turn_slowdown_gain=0.5,
            min_speed_scale=0.35,
        )
        self.assertLess(v_out, 0.20)
        self.assertGreater(v_out, 0.0)
        self.assertGreater(w, 0.0)


class TestScanClustering(unittest.TestCase):
    """Tests for sector extraction, wrap-around, and outlier rejection."""

    def _make_scan(self, default_range=float('inf')):
        """Build a 360-beam 0..2*pi scan array (1 deg resolution)."""
        return [default_range] * 360

    def test_extracts_symmetric_window_across_zero(self):
        """Beams at 355 deg (-5 deg) and 5 deg both stay inside +-15 deg."""
        ranges = self._make_scan()
        ranges[355] = 0.50
        ranges[0] = 0.50
        ranges[5] = 0.50
        ranges[30] = 0.40  # outside +-15 deg

        hits = extract_sector_hits(
            ranges=ranges,
            angle_min=0.0,
            angle_increment=math.radians(1.0),
            range_min=0.12,
            range_max=3.5,
            max_track_range=3.0,
            half_angle=math.radians(15.0),
            center_angle=0.0,
        )
        indices = [idx for idx, _, _ in hits]
        self.assertEqual(indices, [0, 5, 355])

    def test_dynamic_bearing_center_tracks_wide_corner(self):
        """Centering on +30 deg keeps a leader at +35 deg inside +-15 deg."""
        ranges = self._make_scan()
        for deg in (32, 34, 36, 38):
            ranges[deg] = 0.45

        hits = extract_sector_hits(
            ranges=ranges,
            angle_min=0.0,
            angle_increment=math.radians(1.0),
            range_min=0.12,
            range_max=3.5,
            max_track_range=3.0,
            half_angle=math.radians(15.0),
            center_angle=math.radians(30.0),
        )
        result = locate_target_cluster(hits, 0.10, 3)
        self.assertIsNotNone(result)
        dist, bearing, count = result
        self.assertAlmostEqual(dist, 0.45, places=6)
        self.assertAlmostEqual(bearing, math.radians(35.0), places=6)
        self.assertEqual(count, 4)

    def test_rejects_isolated_spike(self):
        """Single close beam fails min_cluster_points width check."""
        ranges = self._make_scan()
        ranges[0] = 0.15  # single spurious floor spike
        hits = extract_sector_hits(
            ranges=ranges,
            angle_min=0.0,
            angle_increment=math.radians(1.0),
            range_min=0.12,
            range_max=3.5,
            max_track_range=3.0,
            half_angle=math.radians(15.0),
        )
        self.assertIsNone(locate_target_cluster(hits, 0.10, 3))


if __name__ == '__main__':
    unittest.main()
