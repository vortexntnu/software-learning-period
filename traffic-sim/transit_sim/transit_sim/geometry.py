"""Geometry helpers for turning lane progress into map positions.

The sim owns all geometry. Recruits' nodes think in "lane 2, 40% along"; these
helpers turn that into an (x, y, heading) on the map.
"""

import math

Point = tuple[float, float]
Polyline = tuple[Point, ...]


def segment_lengths(polyline: Polyline) -> list[float]:
    """Return the length of each segment of a polyline."""
    return [math.dist(polyline[i], polyline[i + 1]) for i in range(len(polyline) - 1)]


def polyline_length(polyline: Polyline) -> float:
    """Return the total arc length of a polyline."""
    return sum(segment_lengths(polyline))


def pose_on_polyline(polyline: Polyline, progress: float) -> tuple[float, float, float]:
    """Return (x, y, heading) at a fraction along a polyline.

    Args:
        polyline: The path, as at least two (x, y) points.
        progress: Fraction along the path. Clamped to 0.0 .. 1.0, so a vehicle
            that overshoots parks at the end of its lane rather than flying off
            the map.

    Returns:
        The position and the heading in radians of the segment it sits on.
    """
    if len(polyline) < 2:
        raise ValueError('A polyline needs at least two points')

    progress = min(max(progress, 0.0), 1.0)
    lengths = segment_lengths(polyline)
    total = sum(lengths)

    if total == 0.0:
        x, y = polyline[0]
        return x, y, 0.0

    target = progress * total
    travelled = 0.0

    for index, length in enumerate(lengths):
        if travelled + length >= target or index == len(lengths) - 1:
            start = polyline[index]
            end = polyline[index + 1]
            local = (target - travelled) / length if length else 0.0
            local = min(max(local, 0.0), 1.0)
            x = start[0] + (end[0] - start[0]) * local
            y = start[1] + (end[1] - start[1]) * local
            heading = math.atan2(end[1] - start[1], end[0] - start[0])
            return x, y, heading
        travelled += length

    # Unreachable: the loop above always returns on its final iteration.
    raise AssertionError('pose_on_polyline failed to locate a segment')


def right_of(heading: float) -> Point:
    """Return the unit vector pointing right of a heading.

    Traffic lights sit on the right-hand side of their approach (European
    convention, SIMULATION_PLAN.md section 4), so this is what places them.
    """
    return math.sin(heading), -math.cos(heading)


def offset(point: Point, direction: Point, distance: float) -> Point:
    """Move a point along a unit direction vector by a distance."""
    return point[0] + direction[0] * distance, point[1] + direction[1] * distance


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return (x, y, z, w) for a rotation about the z axis.

    The city is flat, so a marker's orientation is only ever a yaw.
    """
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
