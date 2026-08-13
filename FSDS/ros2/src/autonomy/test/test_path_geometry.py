import math

from autonomy.path_geometry import (
    pair_cones,
    pure_pursuit_curvature,
    select_lookahead,
    smooth_path,
)


def test_pair_cones_returns_centerline_in_forward_order():
    left = [(5.0, 2.0), (1.0, 2.0), (3.0, 2.0)]
    right = [(3.1, -2.0), (5.1, -2.0), (1.1, -2.0)]
    assert pair_cones(left, right) == [
        (1.05, 0.0), (3.05, 0.0), (5.05, 0.0)]


def test_pair_cones_rejects_implausible_track_width():
    assert pair_cones([(2.0, 0.2)], [(2.0, -0.2)]) == []


def test_select_lookahead_ignores_points_behind_vehicle():
    points = [(-1.0, 0.0), (1.0, 0.0), (3.0, 0.0)]
    assert select_lookahead(points, 2.0) == (3.0, 0.0)


def test_pure_pursuit_curvature_sign():
    assert pure_pursuit_curvature((2.0, 1.0)) > 0.0
    assert pure_pursuit_curvature((2.0, -1.0)) < 0.0
    assert math.isclose(pure_pursuit_curvature((2.0, 0.0)), 0.0)


def test_smooth_path_preserves_endpoints():
    points = [(1.0, 0.0), (2.0, 3.0), (3.0, 0.0)]
    result = smooth_path(points)
    assert result[0] == points[0]
    assert result[-1] == points[-1]
    assert result[1] == (2.0, 1.0)
