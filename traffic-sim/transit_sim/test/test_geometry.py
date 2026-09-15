"""Tests for the lane geometry the whole sim depends on."""

import math

import pytest

from transit_sim import city_map
from transit_sim.geometry import pose_on_polyline, right_of

STRAIGHT = ((0.0, 0.0), (10.0, 0.0))
BENT = ((0.0, 0.0), (10.0, 0.0), (10.0, 10.0))


def test_progress_endpoints():
    assert pose_on_polyline(STRAIGHT, 0.0)[:2] == pytest.approx((0.0, 0.0))
    assert pose_on_polyline(STRAIGHT, 1.0)[:2] == pytest.approx((10.0, 0.0))


def test_progress_midpoint():
    assert pose_on_polyline(STRAIGHT, 0.5)[:2] == pytest.approx((5.0, 0.0))


def test_progress_is_clamped():
    """A vehicle that overshoots parks at the end rather than flying off the map."""
    assert pose_on_polyline(STRAIGHT, 5.0)[:2] == pytest.approx((10.0, 0.0))
    assert pose_on_polyline(STRAIGHT, -3.0)[:2] == pytest.approx((0.0, 0.0))


def test_progress_spans_multiple_segments():
    """Halfway along a bent lane is the corner, and the heading follows the bend."""
    x, y, heading = pose_on_polyline(BENT, 0.5)
    assert (x, y) == pytest.approx((10.0, 0.0))
    x, y, heading = pose_on_polyline(BENT, 0.75)
    assert (x, y) == pytest.approx((10.0, 5.0))
    assert heading == pytest.approx(math.pi / 2)


def test_right_of_headings():
    """Right of north is east, right of east is south."""
    assert right_of(math.pi / 2) == pytest.approx((1.0, 0.0), abs=1e-9)
    assert right_of(0.0) == pytest.approx((0.0, -1.0), abs=1e-9)
    assert right_of(math.pi) == pytest.approx((0.0, 1.0), abs=1e-9)


def test_every_lane_is_on_the_map():
    for lane in city_map.LANES:
        assert city_map.lane(lane.lane_id) is lane
        assert lane.length > 0


def test_unknown_lane_is_none():
    assert city_map.lane(99) is None


def test_signals_sit_on_the_right_of_their_approach():
    """European convention: each light is right of its own incoming lane."""
    # The light is offset from its lane's centre, so it lands beyond the kerb.
    side = city_map.HALF_LANE + city_map.SIGNAL_OFFSET
    stop = city_map.JUNCTION_HALF
    expected = {
        1: (side, -stop),  # northbound, light to the east
        2: (-side, stop),  # southbound, light to the west
        3: (-stop, -side),  # eastbound, light to the south
        4: (stop, side),  # westbound, light to the north
    }
    assert side > city_map.LANE_WIDTH, 'light should sit clear of the road surface'
    for lane_id, (want_x, want_y) in expected.items():
        pose = city_map.signal_pose(lane_id)
        assert pose is not None
        assert pose[0] == pytest.approx(want_x, abs=0.3)
        assert pose[1] == pytest.approx(want_y, abs=0.3)


def test_stop_line_is_outside_the_junction_box():
    for lane_id in (1, 2, 3, 4):
        x, y, _ = city_map.stop_line_pose(lane_id)
        outside = (
            abs(x) > city_map.JUNCTION_HALF - 0.2
            or abs(y) > city_map.JUNCTION_HALF - 0.2
        )
        assert outside, f'lane {lane_id} stop line is inside the junction'


def test_rail_lane_is_not_a_road():
    assert city_map.lane(5).kind == 'rail'
    assert 5 not in [lane.lane_id for lane in city_map.road_lanes()]
    assert city_map.signal_pose(5) is None


def test_level_crossing_is_where_rail_meets_road():
    x, y = city_map.LEVEL_CROSSING
    assert x == pytest.approx(city_map.RAIL_X)
    assert abs(y) < city_map.LANE_WIDTH
