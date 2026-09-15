"""The static picture: everything that never changes.

Grass, roads, dashed centre lines, printed lane numbers, the rail line, the level
crossing, the station and the houses. Built once and published on a latched
topic, so a viewer that connects later still receives the city.

Only the roads, lanes, rail line and crossing are functional. Grass, trees and
buildings are decoration (SIMULATION_PLAN.md section 2).
"""

from visualization_msgs.msg import Marker

from transit_sim import city_map, colors, markers

NS = 'city'

GROUND_SIZE = 100.0
ROAD_WIDTH = city_map.LANE_WIDTH * 2.0

# Flat city, so everything is stacked in thin layers to avoid z-fighting.
Z_GROUND = -0.15
Z_ROAD = 0.0
Z_MARKING = 0.08
Z_RAIL = 0.06

DASH_LENGTH = 2.0
DASH_GAP = 2.0
SLEEPER_SPACING = 2.4
RAIL_GAUGE = 1.44


class _Ids:
    """Hands out unique marker ids within the static namespace."""

    def __init__(self) -> None:
        self._next = 0

    def take(self) -> int:
        """Return the next unused marker id."""
        value = self._next
        self._next += 1
        return value


def _ground(stamp, ids: _Ids) -> list[Marker]:
    return [
        markers.cube(
            NS,
            ids.take(),
            stamp,
            x=0.0,
            y=0.0,
            z=Z_GROUND,
            size=(GROUND_SIZE, GROUND_SIZE, 0.2),
            color=colors.rgba(colors.GRASS),
        )
    ]


def _roads(stamp, ids: _Ids) -> list[Marker]:
    reach = city_map.ROAD_REACH
    return [
        markers.cube(
            NS,
            ids.take(),
            stamp,
            x=0.0,
            y=0.0,
            z=Z_ROAD,
            size=(ROAD_WIDTH, reach * 2.0, 0.1),
            color=colors.rgba(colors.ASPHALT),
        ),
        markers.cube(
            NS,
            ids.take(),
            stamp,
            x=0.0,
            y=0.0,
            z=Z_ROAD,
            size=(reach * 2.0, ROAD_WIDTH, 0.1),
            color=colors.rgba(colors.ASPHALT),
        ),
    ]


def _dashes_along(
    start: float, end: float, fixed: float, vertical: bool, skip: float
) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Build dash segments along an axis, leaving the junction box empty."""
    segments = []
    position = start
    while position < end:
        dash_end = min(position + DASH_LENGTH, end)
        midpoint = (position + dash_end) / 2.0
        if abs(midpoint) > skip:
            if vertical:
                segments.append(((fixed, position), (fixed, dash_end)))
            else:
                segments.append(((position, fixed), (dash_end, fixed)))
        position += DASH_LENGTH + DASH_GAP
    return segments


def _road_markings(stamp, ids: _Ids) -> list[Marker]:
    reach = city_map.ROAD_REACH
    skip = city_map.JUNCTION_HALF

    dashes = _dashes_along(-reach, reach, 0.0, vertical=True, skip=skip)
    dashes += _dashes_along(-reach, reach, 0.0, vertical=False, skip=skip)

    edges = [
        ((-ROAD_WIDTH / 2.0, -reach), (-ROAD_WIDTH / 2.0, -ROAD_WIDTH / 2.0)),
        ((ROAD_WIDTH / 2.0, -reach), (ROAD_WIDTH / 2.0, -ROAD_WIDTH / 2.0)),
        ((-ROAD_WIDTH / 2.0, ROAD_WIDTH / 2.0), (-ROAD_WIDTH / 2.0, reach)),
        ((ROAD_WIDTH / 2.0, ROAD_WIDTH / 2.0), (ROAD_WIDTH / 2.0, reach)),
        ((-reach, -ROAD_WIDTH / 2.0), (-ROAD_WIDTH / 2.0, -ROAD_WIDTH / 2.0)),
        ((-reach, ROAD_WIDTH / 2.0), (-ROAD_WIDTH / 2.0, ROAD_WIDTH / 2.0)),
        ((ROAD_WIDTH / 2.0, -ROAD_WIDTH / 2.0), (reach, -ROAD_WIDTH / 2.0)),
        ((ROAD_WIDTH / 2.0, ROAD_WIDTH / 2.0), (reach, ROAD_WIDTH / 2.0)),
    ]

    return [
        markers.line_list(
            NS,
            ids.take(),
            stamp,
            segments=dashes,
            width=0.22,
            color=colors.rgba(colors.ROAD_MARKING),
            z=Z_MARKING,
        ),
        markers.line_list(
            NS,
            ids.take(),
            stamp,
            segments=edges,
            width=0.18,
            color=colors.rgba(colors.ROAD_MARKING, 0.75),
            z=Z_MARKING,
        ),
    ]


def _pedestrian_crossings(stamp, ids: _Ids) -> list[Marker]:
    """Zebra stripes on each approach, just outside the junction box."""
    stripes = []
    offset = city_map.JUNCTION_HALF + 1.6
    half = ROAD_WIDTH / 2.0

    for sign in (-1.0, 1.0):
        for step in range(6):
            position = -half + 0.6 + step * 1.1
            stripes.append(
                ((position, sign * offset - 0.7), (position, sign * offset + 0.7))
            )
            stripes.append(
                ((sign * offset - 0.7, position), (sign * offset + 0.7, position))
            )

    return [
        markers.line_list(
            NS,
            ids.take(),
            stamp,
            segments=stripes,
            width=0.55,
            color=colors.rgba(colors.ROAD_MARKING, 0.9),
            z=Z_MARKING,
        )
    ]


def _lane_numbers(stamp, ids: _Ids) -> list[Marker]:
    """The printed lane numbers: the shared vocabulary made visible."""
    out = []
    for target_lane in city_map.LANES:
        position = city_map.lane_label_pose(target_lane.lane_id)
        if position is None:
            continue
        out.append(
            markers.text(
                NS,
                ids.take(),
                stamp,
                x=position[0],
                y=position[1],
                content=str(target_lane.lane_id),
                height=2.6,
                color=colors.rgba(colors.LABEL),
            )
        )
    return out


def _railway(stamp, ids: _Ids) -> list[Marker]:
    reach = city_map.ROAD_REACH
    rail_x = city_map.RAIL_X
    half_gauge = RAIL_GAUGE / 2.0

    sleepers = []
    position = -reach
    while position < reach:
        sleepers.append(((rail_x - 1.3, position), (rail_x + 1.3, position)))
        position += SLEEPER_SPACING

    out = [
        markers.line_list(
            NS,
            ids.take(),
            stamp,
            segments=sleepers,
            width=0.5,
            color=colors.rgba(colors.SLEEPER),
            z=Z_RAIL,
        )
    ]
    for side in (-half_gauge, half_gauge):
        out.append(
            markers.line_strip(
                NS,
                ids.take(),
                stamp,
                points=[(rail_x + side, -reach), (rail_x + side, reach)],
                width=0.3,
                color=colors.rgba(colors.RAIL),
                z=Z_RAIL + 0.03,
            )
        )
    return out


def _level_crossing(stamp, ids: _Ids) -> list[Marker]:
    """The road surface over the rails, plus the two barrier posts."""
    crossing_x, crossing_y = city_map.LEVEL_CROSSING
    out = [
        markers.cube(
            NS,
            ids.take(),
            stamp,
            x=crossing_x,
            y=crossing_y,
            z=Z_RAIL + 0.05,
            size=(4.4, ROAD_WIDTH, 0.08),
            color=colors.rgba(colors.PLATFORM),
        )
    ]
    for side in (-1.0, 1.0):
        out.append(
            markers.cube(
                NS,
                ids.take(),
                stamp,
                x=crossing_x + side * 3.4,
                y=side * (ROAD_WIDTH / 2.0 + 0.6),
                z=0.6,
                size=(0.5, 0.5, 1.2),
                color=colors.rgba(colors.LAMP_HOUSING),
            )
        )
    return out


def _station(stamp, ids: _Ids) -> list[Marker]:
    x, y = city_map.STATION_CENTRE
    return [
        markers.cube(
            NS,
            ids.take(),
            stamp,
            x=city_map.RAIL_X - 2.8,
            y=y,
            z=0.1,
            size=(2.6, 18.0, 0.25),
            color=colors.rgba(colors.PLATFORM),
        ),
        markers.cube(
            NS,
            ids.take(),
            stamp,
            x=x - 2.0,
            y=y,
            z=1.6,
            size=(6.0, 12.0, 3.2),
            color=colors.rgba(colors.BUILDING),
        ),
        markers.cube(
            NS,
            ids.take(),
            stamp,
            x=x - 2.0,
            y=y,
            z=3.4,
            size=(6.8, 12.8, 0.4),
            color=colors.rgba(colors.ROOF),
        ),
        markers.text(
            NS,
            ids.take(),
            stamp,
            x=x - 2.0,
            y=y - 8.0,
            z=1.0,
            content='STATION',
            height=1.8,
            color=colors.rgba(colors.LABEL),
        ),
    ]


HOUSE_SITES = (
    (11.0, 11.0),
    (18.0, 12.5),
    (11.5, 19.0),
    (-11.0, 12.0),
    (-11.5, 22.0),
    (11.0, -12.0),
    (19.0, -13.5),
    (11.5, -20.0),
    (-12.0, -11.0),
    (-12.5, -20.0),
    (-30.0, -12.0),
    (26.0, 22.0),
)

TREE_SITES = (
    (7.0, 25.0),
    (24.0, 8.0),
    (-8.0, 30.0),
    (-25.0, -26.0),
    (28.0, -8.0),
    (-32.0, 28.0),
    (16.0, 31.0),
    (-18.0, -30.0),
    (31.0, -25.0),
    (-30.0, 6.0),
)


def _buildings(stamp, ids: _Ids) -> list[Marker]:
    out = []
    for index, (x, y) in enumerate(HOUSE_SITES):
        width = 5.0 + (index % 3)
        depth = 4.5 + (index % 2)
        out.append(
            markers.cube(
                NS,
                ids.take(),
                stamp,
                x=x,
                y=y,
                z=1.3,
                size=(width, depth, 2.6),
                color=colors.rgba(colors.BUILDING),
            )
        )
        out.append(
            markers.cube(
                NS,
                ids.take(),
                stamp,
                x=x,
                y=y,
                z=2.8,
                size=(width + 0.7, depth + 0.7, 0.35),
                color=colors.rgba(colors.ROOF),
            )
        )
    return out


def _trees(stamp, ids: _Ids) -> list[Marker]:
    out = []
    for index, (x, y) in enumerate(TREE_SITES):
        diameter = 3.4 + (index % 3) * 0.7
        out.append(
            markers.sphere(
                NS,
                ids.take(),
                stamp,
                x=x,
                y=y,
                z=0.9,
                diameter=diameter,
                color=colors.rgba(colors.TREE),
            )
        )
    return out


def build_static_scene(stamp) -> list[Marker]:
    """Build every marker that makes up the unchanging city."""
    ids = _Ids()
    scene: list[Marker] = []
    scene += _ground(stamp, ids)
    scene += _roads(stamp, ids)
    scene += _road_markings(stamp, ids)
    scene += _pedestrian_crossings(stamp, ids)
    scene += _railway(stamp, ids)
    scene += _level_crossing(stamp, ids)
    scene += _station(stamp, ids)
    scene += _buildings(stamp, ids)
    scene += _trees(stamp, ids)
    scene += _lane_numbers(stamp, ids)
    return scene
