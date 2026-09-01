"""The map: the fixed geometry of the city, owned entirely by the sim.

Recruits never edit this. They only reference `lane_id`, which is the shared
vocabulary between their nodes and the sim (SIMULATION_PLAN.md sections 3 and 5).

Coordinates are metres in the `map` frame, with the origin at the centre of the
four-way junction. Extending the city means adding rows to LANES, not changing
the render loop.
"""

import math
from dataclasses import dataclass

from transit_sim.geometry import (
    Point,
    Polyline,
    offset,
    polyline_length,
    pose_on_polyline,
    right_of,
)

# --- City dimensions -------------------------------------------------------

LANE_WIDTH = 3.5
HALF_LANE = LANE_WIDTH / 2.0
ROAD_REACH = 40.0
JUNCTION_HALF = 5.0
RAIL_X = -20.0
SIGNAL_OFFSET = 4.5
STATION_CENTRE: Point = (-27.0, 12.0)


@dataclass(frozen=True)
class Lane:
    """One directional lane. A physical road is two of these."""

    lane_id: int
    name: str
    polyline: Polyline
    kind: str = 'road'

    @property
    def length(self) -> float:
        """Total length of the lane in metres."""
        return polyline_length(self.polyline)

    def pose_at(self, progress: float) -> tuple[float, float, float]:
        """Return (x, y, heading) at a fraction along this lane."""
        return pose_on_polyline(self.polyline, progress)


@dataclass(frozen=True)
class Junction:
    """A road intersection. Its box is where lanes must stop short."""

    name: str
    centre: Point
    half_size: float
    approach_lane_ids: tuple[int, ...]


# --- The minimal starting city ---------------------------------------------
#
# One vertical road (lanes 1 and 2), one horizontal road (lanes 3 and 4), a
# four-way junction where they cross, and a rail line parallel to the vertical
# road (lane 5) crossing the horizontal road at a level crossing.

LANES: tuple[Lane, ...] = (
    Lane(
        lane_id=1,
        name='northbound',
        polyline=((HALF_LANE, -ROAD_REACH), (HALF_LANE, ROAD_REACH)),
    ),
    Lane(
        lane_id=2,
        name='southbound',
        polyline=((-HALF_LANE, ROAD_REACH), (-HALF_LANE, -ROAD_REACH)),
    ),
    Lane(
        lane_id=3,
        name='eastbound',
        polyline=((-ROAD_REACH, -HALF_LANE), (ROAD_REACH, -HALF_LANE)),
    ),
    Lane(
        lane_id=4,
        name='westbound',
        polyline=((ROAD_REACH, HALF_LANE), (-ROAD_REACH, HALF_LANE)),
    ),
    Lane(
        lane_id=5,
        name='railway',
        polyline=((RAIL_X, -ROAD_REACH), (RAIL_X, ROAD_REACH)),
        kind='rail',
    ),
)

JUNCTIONS: tuple[Junction, ...] = (
    Junction(
        name='junction1',
        centre=(0.0, 0.0),
        half_size=JUNCTION_HALF,
        approach_lane_ids=(1, 2, 3, 4),
    ),
)

# Where the rail line crosses the horizontal road.
LEVEL_CROSSING: Point = (RAIL_X, 0.0)
LEVEL_CROSSING_LANE_ID = 5

_LANES_BY_ID = {lane.lane_id: lane for lane in LANES}


def lane(lane_id: int) -> Lane | None:
    """Look up a lane by its printed number, or None if it is not on the map."""
    return _LANES_BY_ID.get(lane_id)


def road_lanes() -> tuple[Lane, ...]:
    """Return the lanes cars drive on."""
    return tuple(item for item in LANES if item.kind == 'road')


def rail_lanes() -> tuple[Lane, ...]:
    """Return the lanes trains run on."""
    return tuple(item for item in LANES if item.kind == 'rail')


def junction_for_lane(lane_id: int) -> Junction | None:
    """Return the junction a lane feeds into, if any."""
    for junction in JUNCTIONS:
        if lane_id in junction.approach_lane_ids:
            return junction
    return None


def stop_line_pose(lane_id: int) -> tuple[float, float, float] | None:
    """Return (x, y, heading) of a lane's stop line, just before its junction.

    Walks the lane until it first enters the junction box and backs off to the
    boundary. Returns None if the lane does not feed a junction.
    """
    target_lane = lane(lane_id)
    junction = junction_for_lane(lane_id)
    if target_lane is None or junction is None:
        return None

    samples = 800
    previous = target_lane.pose_at(0.0)
    for step in range(1, samples + 1):
        x, y, heading = target_lane.pose_at(step / samples)
        inside_x = abs(x - junction.centre[0]) <= junction.half_size
        inside_y = abs(y - junction.centre[1]) <= junction.half_size
        if inside_x and inside_y:
            return previous
        previous = (x, y, heading)
    return None


def signal_pose(lane_id: int) -> tuple[float, float, float] | None:
    """Return (x, y, heading) where a lane's traffic light should be drawn.

    The light sits at the stop line, offset to the right of the direction of
    travel: the European convention from SIMULATION_PLAN.md section 4.
    """
    stop = stop_line_pose(lane_id)
    if stop is None:
        return None
    x, y, heading = stop
    light_x, light_y = offset((x, y), right_of(heading), SIGNAL_OFFSET)
    return light_x, light_y, heading


def lane_label_pose(lane_id: int) -> tuple[float, float] | None:
    """Return where a lane's printed number goes: beside the lane, on its right.

    The numbers are what make debugging visual, so a viewer can see which lane a
    car claims to be on (SIMULATION_PLAN.md section 5).
    """
    target_lane = lane(lane_id)
    if target_lane is None:
        return None
    x, y, heading = target_lane.pose_at(0.22)
    return offset((x, y), right_of(heading), HALF_LANE + 1.4)


def road_centrelines() -> tuple[tuple[Point, Point], ...]:
    """Return the centre line of each physical road, for drawing dashes."""
    return (
        ((0.0, -ROAD_REACH), (0.0, ROAD_REACH)),
        ((-ROAD_REACH, 0.0), (ROAD_REACH, 0.0)),
    )


def heading_of(lane_id: int) -> float:
    """Return the heading of a straight lane in radians."""
    target_lane = lane(lane_id)
    if target_lane is None:
        return 0.0
    start, end = target_lane.polyline[0], target_lane.polyline[-1]
    return math.atan2(end[1] - start[1], end[0] - start[0])
