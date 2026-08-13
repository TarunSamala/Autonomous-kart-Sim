"""ROS-independent geometry used by the local autonomy pipeline."""

import math


def pair_cones(left, right, min_width=1.5, max_width=6.0,
               max_longitudinal_offset=2.0):
    """Greedily pair opposing cones and return ordered lane midpoints."""
    candidates = []
    for left_index, (lx, ly) in enumerate(left):
        for right_index, (rx, ry) in enumerate(right):
            width = math.hypot(lx - rx, ly - ry)
            longitudinal_offset = abs(lx - rx)
            if (min_width <= width <= max_width and
                    longitudinal_offset <= max_longitudinal_offset):
                cost = longitudinal_offset + 0.1 * width
                candidates.append((cost, left_index, right_index))

    used_left = set()
    used_right = set()
    midpoints = []
    for _, left_index, right_index in sorted(candidates):
        if left_index in used_left or right_index in used_right:
            continue
        used_left.add(left_index)
        used_right.add(right_index)
        lx, ly = left[left_index]
        rx, ry = right[right_index]
        midpoints.append(((lx + rx) * 0.5, (ly + ry) * 0.5))

    return sorted(midpoints, key=lambda point: point[0])


def smooth_path(points):
    """Apply a conservative three-point moving average to a local path."""
    if len(points) < 3:
        return list(points)
    smoothed = [points[0]]
    for index in range(1, len(points) - 1):
        window = points[index - 1:index + 2]
        smoothed.append((
            sum(point[0] for point in window) / 3.0,
            sum(point[1] for point in window) / 3.0,
        ))
    smoothed.append(points[-1])
    return smoothed


def select_lookahead(points, lookahead_distance):
    """Select the first forward path point outside the lookahead radius."""
    forward = [point for point in points if point[0] > 0.0]
    if not forward:
        return None
    for point in forward:
        if math.hypot(*point) >= lookahead_distance:
            return point
    return forward[-1]


def pure_pursuit_curvature(target):
    """Return curvature for a target expressed in the vehicle frame."""
    x, y = target
    distance_squared = x * x + y * y
    if distance_squared < 1e-6:
        return 0.0
    return 2.0 * y / distance_squared
