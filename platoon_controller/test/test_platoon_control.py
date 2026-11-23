"""Unit tests for platoon pure pursuit, PD control, and scan filtering."""

import math
import unittest

from platoon_controller.control_math import (
    compute_linear_pd,
    compute_pure_pursuit_steering,
    compute_search_center,
    extract_sector_hits,
    fit_wheel_limits,
    locate_target_cluster,
    normalize_angle,
    rate_limit,
    wheel_speeds,
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

    def test_negative_derivative_never_reverses_when_target_is_far(self):
        """Noisy negative d_filtered slows approach instead of backing up."""
        speed, error, in_db = compute_linear_pd(
            distance=0.40,
            setpoint=0.30,
            d_filtered=-1.0,
            kp=2.0,
            kd=0.5,
            deadband=0.015,
            max_speed=0.22,
            max_reverse=0.08,
        )
        self.assertFalse(in_db)
        self.assertGreater(error, 0.015)
        self.assertEqual(speed, 0.0)


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

    def test_no_turn_distance_suppresses_close_spinning(self):
        """Below no_turn_distance, robot backs out straight without turning."""
        v_out, w, _ = compute_pure_pursuit_steering(
            x=0.14,
            y=0.10,
            linear_speed=-0.08,
            no_turn_distance=0.20,
        )
        self.assertEqual(v_out, -0.08)
        self.assertEqual(w, 0.0)


class TestScanClustering(unittest.TestCase):
    """Tests for sector extraction, wrap-around, and outlier rejection."""

    def _make_scan(self, default_range=float('inf')):
        """Build a 360-beam 0..2*pi scan array (1 deg resolution)."""
        return [default_range] * 360

    def _hits(self, ranges, center_deg=0.0):
        """Extract +-15 deg sector hits from a 1-deg LDS-style scan."""
        return extract_sector_hits(
            ranges=ranges,
            angle_min=0.0,
            angle_increment=math.radians(1.0),
            range_min=0.12,
            range_max=3.5,
            max_track_range=3.0,
            half_angle=math.radians(15.0),
            center_angle=math.radians(center_deg),
        )

    def test_extracts_symmetric_window_across_zero(self):
        """Beams at 355 deg (-5 deg) and 5 deg both stay inside +-15 deg."""
        ranges = self._make_scan()
        ranges[355] = 0.50
        ranges[0] = 0.50
        ranges[5] = 0.50
        ranges[30] = 0.40  # outside +-15 deg

        indices = [idx for idx, _, _ in self._hits(ranges)]
        self.assertEqual(indices, [0, 5, 355])

    def test_dynamic_bearing_center_tracks_wide_corner(self):
        """Centering on +30 deg keeps a leader at +35 deg inside +-15 deg."""
        ranges = self._make_scan()
        for deg in (32, 34, 36, 38):
            ranges[deg] = 0.45

        result = locate_target_cluster(
            self._hits(ranges, center_deg=30.0), 0.10, 3)
        self.assertIsNotNone(result)
        dist, bearing, count = result
        self.assertAlmostEqual(dist, 0.45, places=6)
        self.assertAlmostEqual(bearing, math.radians(35.0), places=6)
        self.assertEqual(count, 4)

    def test_rejects_isolated_spike(self):
        """Single close beam fails min_cluster_points width check."""
        ranges = self._make_scan()
        ranges[0] = 0.15  # single spurious floor spike
        self.assertIsNone(locate_target_cluster(self._hits(ranges), 0.10, 3))

    def test_spike_in_front_of_leader_is_outvoted(self):
        """A lone near spike must not cost the frame when a leader is there.

        Anchoring on the plain nearest return let one spike discard the
        whole frame; five in a row declared the target lost. Anchoring on
        the k-th nearest keeps the lock and ignores the spike.
        """
        ranges = self._make_scan()
        for deg in list(range(0, 6)) + list(range(355, 360)):
            ranges[deg] = 0.50          # leader, 11 beams
        ranges[2] = 0.15                # spurious reflection mid-leader

        result = locate_target_cluster(self._hits(ranges), 0.10, 3)
        self.assertIsNotNone(result)
        dist, _, count = result
        self.assertAlmostEqual(dist, 0.50, places=6)
        self.assertEqual(count, 10)     # the spike beam is excluded

    def test_two_spikes_alone_are_still_rejected(self):
        """Fewer than min_cluster_points agreeing returns is never a target."""
        ranges = self._make_scan()
        ranges[0] = 0.15
        ranges[1] = 0.16
        self.assertIsNone(locate_target_cluster(self._hits(ranges), 0.10, 3))

    def test_search_center_locks_forward_when_too_close(self):
        """Close-range tracking keeps cone centered at 0 to prevent walking."""
        center = compute_search_center(
            bearing=math.radians(30.0),
            distance=0.18,
            track_bearing=True,
            max_track_angle=math.radians(40.0),
            close_lock_distance=0.22,
        )
        self.assertEqual(center, 0.0)

    def test_search_center_clamps_to_max_track_angle(self):
        """Locked bearing beyond max_track_angle is clamped."""
        limit = math.radians(40.0)
        center = compute_search_center(
            bearing=math.radians(55.0),
            distance=0.50,
            track_bearing=True,
            max_track_angle=limit,
            close_lock_distance=0.22,
        )
        self.assertAlmostEqual(center, limit, places=6)


class TestRateLimit(unittest.TestCase):
    """Tests for angular acceleration limiting."""

    def test_clips_large_step(self):
        """A 1.5 rad/s jump is clipped to max_rate * dt."""
        self.assertAlmostEqual(
            rate_limit(0.0, 1.5, 2.0, 0.05), 0.10, places=6)

    def test_lands_exactly_without_overshoot(self):
        """A change smaller than one step reaches the target exactly."""
        self.assertAlmostEqual(
            rate_limit(1.45, 1.5, 2.0, 0.05), 1.5, places=6)

    def test_limits_both_directions(self):
        """Negative targets are limited symmetrically."""
        self.assertAlmostEqual(
            rate_limit(0.0, -1.5, 2.0, 0.05), -0.10, places=6)

    def test_zero_rate_disables_limit(self):
        """A non-positive max_rate passes the target straight through."""
        self.assertEqual(rate_limit(0.0, 1.5, 0.0, 0.05), 1.5)


class TestWheelLimits(unittest.TestCase):
    """Tests for differential-drive per-wheel saturation handling."""

    def test_wheel_speeds_split_correctly(self):
        """Positive omega (left turn) speeds up the right wheel."""
        left, right = wheel_speeds(0.10, 1.0, wheel_separation=0.16)
        self.assertAlmostEqual(left, 0.02, places=6)
        self.assertAlmostEqual(right, 0.18, places=6)

    def test_command_within_limits_is_unchanged(self):
        """A command both wheels can execute passes through untouched."""
        self.assertEqual(fit_wheel_limits(0.10, 0.5), (0.10, 0.5))

    def test_saturating_command_is_scaled_into_range(self):
        """Full speed plus a turn is scaled until the outer wheel fits."""
        v, w = fit_wheel_limits(0.22, 1.0, max_wheel_speed=0.22)
        left, right = wheel_speeds(v, w)
        self.assertLessEqual(max(abs(left), abs(right)), 0.22 + 1e-9)

    def test_scaling_preserves_path_curvature(self):
        """Omega / v is unchanged, so the robot follows the same arc."""
        v, w = fit_wheel_limits(0.22, 1.0, max_wheel_speed=0.22)
        self.assertAlmostEqual(w / v, 1.0 / 0.22, places=6)

    def test_zero_limit_disables_fitting(self):
        """A non-positive max_wheel_speed turns the fit off."""
        self.assertEqual(
            fit_wheel_limits(0.22, 1.5, max_wheel_speed=0.0), (0.22, 1.5))


if __name__ == '__main__':
    unittest.main()
