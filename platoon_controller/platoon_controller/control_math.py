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


def rate_limit(current, target, max_rate, dt):
    """Limit how fast ``current`` may move toward ``target``.

    Returns the new value, moved toward ``target`` by at most
    ``max_rate * dt``. A non-positive ``max_rate`` or ``dt`` disables the
    limit and returns ``target`` unchanged.

    Angular velocity is the critical use. The LDS sweeps its 360 beams
    sequentially over one 0.2 s revolution, so a robot rotating while the
    scan is taken smears the sweep: at 1.5 rad/s the first and last beams
    come from headings 17 deg apart, more than half the width of a +-15 deg
    tracking window. A distorted scan yields a wrong bearing, which commands
    a larger correction, which distorts the next scan further. Limiting how
    fast omega may change breaks that loop before it starts.

    Gazebo samples all beams instantaneously, so this failure mode cannot
    appear in simulation -- only on the real LDS.
    """
    if max_rate <= 0.0 or dt <= 0.0:
        return target

    max_step = max_rate * dt
    delta = target - current

    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


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

    The anchor is the ``min_cluster_points``-th nearest return, not simply
    the nearest one. A lone spurious short reading -- a floor reflection, a
    dust mote, a dropout glitch -- therefore cannot anchor the cluster at
    all, because it takes that many agreeing returns before a range is even
    considered. Beams within ``cluster_tolerance`` of the anchor form the
    near face of the leading object.

    This matters more than it looks. Anchoring on the plain nearest return
    meant a single spike discarded the whole frame, and five discarded
    frames in a row declared the target lost -- a one-second dropout caused
    by one bad beam. Anchoring on the k-th nearest outvotes the spike and
    keeps tracking through it.
    """
    if len(hits) < min_cluster_points:
        return None

    ordered = sorted(r for _, _, r in hits)
    anchor = ordered[min_cluster_points - 1]

    # Symmetric band: beams slightly nearer than the anchor belong to the
    # same face, since the anchor is deliberately not the nearest return.
    face = [(a, r) for _, a, r in hits
            if abs(r - anchor) <= cluster_tolerance]

    if len(face) < min_cluster_points:
        return None

    # Median over the face, not mean: immune to a straggler beam that
    # clipped an edge and came back long.
    distance = median([r for _, r in face])

    # Centroid of the face gives the angular offset Role 4 needs.
    bearing = sum(a for a, _ in face) / len(face)

    return distance, bearing, len(face)


def compute_search_center(
    bearing,
    distance,
    track_bearing,
    max_track_angle,
    close_lock_distance=0.22,
):
    """Return the angular center for the LiDAR tracking sector.

    Centers the cone on ``bearing`` during turns so the leader stays locked,
    except below ``close_lock_distance`` where center LDS beams can drop under
    ``range_min`` (0.12 m) and leave only edge hits that would walk the cone
    sideways onto walls.
    """
    if not track_bearing or bearing is None:
        return 0.0
    if distance is not None and distance < close_lock_distance:
        return 0.0
    return clamp(bearing, -max_track_angle, max_track_angle)


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
    # Damping may slow a fast approach to zero, never drive backward away from
    # a target that is farther than the setpoint.
    if error > deadband and raw_speed < 0.0:
        raw_speed = 0.0

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
    no_turn_distance=0.0,
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

    if distance < no_turn_distance or abs(bearing) < angular_deadband:
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


def wheel_speeds(linear, angular, wheel_separation=0.160):
    """Return ``(left, right)`` wheel speeds in m/s for a differential drive.

    A TurtleBot3 Burger is limited per WHEEL at 0.22 m/s, not on ``v`` and
    ``omega`` independently. At v = 0.22 any non-zero omega asks the outer
    wheel for more than it can give; it saturates, the executed arc stops
    matching the commanded one, and the robot under-turns exactly when it
    most needs to turn.
    """
    half = 0.5 * wheel_separation * angular
    return linear - half, linear + half


def fit_wheel_limits(linear, angular, max_wheel_speed=0.22,
                     wheel_separation=0.160):
    """Scale ``(linear, angular)`` together until both wheels are in range.

    Scaling both by the same factor preserves ``omega / v`` and therefore the
    path curvature, so the robot follows the intended arc more slowly rather
    than a different arc at the intended speed. Returns the pair unchanged
    when it already fits.
    """
    if max_wheel_speed <= 0.0:
        return linear, angular

    left, right = wheel_speeds(linear, angular, wheel_separation)
    worst = max(abs(left), abs(right))

    if worst <= max_wheel_speed:
        return linear, angular

    scale = max_wheel_speed / worst
    return linear * scale, angular * scale
