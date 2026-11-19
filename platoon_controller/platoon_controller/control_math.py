"""Pure-Python control and LiDAR clustering math for the platoon nodes.

Kept free of ``rclpy`` imports so every control law, safety clamp, and scan
clustering helper can be unit-tested on any platform (including macOS) without
a live ROS 2 runtime.
"""

import math
from statistics import median


def clamp(value, low, high):
    """Clamp ``value`` into the closed interval ``[low, high]``."""
    return max(low, min(high, value))


def normalize_angle(angle):
    """Wrap any angle in radians into ``(-pi, +pi]``.

    The TurtleBot3 LDS reports ``angle_min = 0.0`` and ``angle_max = 2*pi``,
    so beams to the robot's right arrive near ``2*pi`` rather than as
    negatives. Wrapping keeps sector comparisons symmetric across zero.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def extract_sector_hits(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    max_track_range,
    half_angle,
    center_angle=0.0,
):
    """Return ``[(index, angle, range)]`` for valid beams inside a cone."""
    hits = []

    # Gazebo marks no-return beams as +inf; some drivers emit 0.0 instead.
    near = max(range_min, 0.02)
    far = min(range_max, max_track_range)

    for i, r in enumerate(ranges):
        angle = normalize_angle(angle_min + i * angle_increment)

        if abs(normalize_angle(angle - center_angle)) > half_angle:
            continue
        if not math.isfinite(r):
            continue
        if r < near or r > far:
            continue

        hits.append((i, angle, r))

    return hits


def locate_target_cluster(hits, cluster_tolerance, min_cluster_points):
    """Collapse sector hits into ``(distance, bearing, face_size)`` or None.

    Finds the nearest return, keeps beams within ``cluster_tolerance`` of it
    (the near face of the leading object), and rejects isolated spikes narrower
    than ``min_cluster_points``.
    """
    if not hits:
        return None

    nearest = min(r for _, _, r in hits)
    face = [(a, r) for _, a, r in hits if r <= nearest + cluster_tolerance]

    if len(face) < min_cluster_points:
        return None

    distance = median([r for _, r in face])
    bearing = sum(a for a, _ in face) / len(face)

    return distance, bearing, len(face)


def compute_linear_pd(
    distance,
    setpoint,
    d_filtered,
    kp,
    kd,
    deadband,
    max_speed,
    max_reverse,
):
    """Compute clamped linear velocity from the 1D PD gap law.

    Returns ``(speed, error, in_deadband)``.
    """
    error = distance - setpoint
    if abs(error) < deadband:
        return 0.0, error, True

    raw_speed = kp * error + kd * d_filtered
    speed = clamp(raw_speed, -max_reverse, max_speed)
    return speed, error, False


def compute_pure_pursuit_steering(
    x,
    y,
    linear_speed,
    k_align=1.4,
    blend_speed=0.08,
    angular_deadband=0.03,
    max_angular_speed=1.5,
    turn_slowdown_gain=0.5,
    min_speed_scale=0.35,
):
    """Compute coupled ``(linear_x, angular_z, bearing)`` via Pure Pursuit.

    When moving forward, angular velocity follows the exact geometric Pure
    Pursuit arc curvature ``kappa = 2*y / (x^2 + y^2)`` so the follower traces
    a smooth circular path to ``(x, y)`` rather than cutting corners.

    At very low or zero forward speed (for example, while holding the gap
    inside the distance deadband), Pure Pursuit alone yields ``omega = 0``.
    To keep the leader centered in the LiDAR sector while stationary or
    backing up, ``omega`` smoothly blends into proportional bearing alignment
    ``k_align * bearing`` as ``linear_speed`` drops below ``blend_speed``.
    """
    distance = math.hypot(x, y)
    bearing = math.atan2(y, x)

    if distance <= 1e-6:
        return linear_speed, 0.0, 0.0

    # Reduce forward speed gently on sharp bearings before computing Pure
    # Pursuit omega so omega = v * kappa preserves the exact geometric arc
    # without saturating differential-drive wheel motors.
    effective_linear = linear_speed
    if linear_speed > 0.0 and turn_slowdown_gain > 0.0:
        scale = max(min_speed_scale, 1.0 - turn_slowdown_gain * abs(bearing))
        effective_linear = linear_speed * scale

    if abs(bearing) < angular_deadband:
        return effective_linear, 0.0, bearing

    curvature = (2.0 * y) / (distance * distance)
    omega_pp = max(effective_linear, 0.0) * curvature
    omega_align = k_align * bearing

    if effective_linear <= 0.0 or blend_speed <= 0.0:
        alpha_pp = 0.0
    else:
        alpha_pp = clamp(effective_linear / blend_speed, 0.0, 1.0)

    omega = alpha_pp * omega_pp + (1.0 - alpha_pp) * omega_align
    omega = clamp(omega, -max_angular_speed, max_angular_speed)

    return effective_linear, omega, bearing
