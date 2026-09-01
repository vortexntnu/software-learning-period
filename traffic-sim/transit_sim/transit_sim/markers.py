"""Small builders for visualization_msgs Markers.

Everything the sim draws is a Marker, so handlers only ever call these instead
of filling the message out by hand. Foxglove's 3D panel renders MarkerArray
natively, and so does RViz2, which is why the sim is not tied to either viewer.
"""

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

from transit_sim.geometry import yaw_to_quaternion

FRAME_ID = 'map'


def _header(stamp) -> Header:
    return Header(frame_id=FRAME_ID, stamp=stamp)


def _pose(x: float, y: float, z: float = 0.0, yaw: float = 0.0) -> Pose:
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


def _point(x: float, y: float, z: float = 0.0) -> PointMsg:
    return PointMsg(x=float(x), y=float(y), z=float(z))


def _base(ns: str, marker_id: int, marker_type: int, stamp) -> Marker:
    marker = Marker()
    marker.header = _header(stamp)
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose = _pose(0.0, 0.0)
    marker.lifetime = Duration(sec=0, nanosec=0)
    return marker


def cube(
    ns: str,
    marker_id: int,
    stamp,
    *,
    x: float,
    y: float,
    z: float = 0.0,
    size: tuple[float, float, float],
    color: ColorRGBA,
    yaw: float = 0.0,
) -> Marker:
    """A box. Cars, buildings, barriers and the ground are all boxes."""
    marker = _base(ns, marker_id, Marker.CUBE, stamp)
    marker.pose = _pose(x, y, z, yaw)
    marker.scale = Vector3(x=float(size[0]), y=float(size[1]), z=float(size[2]))
    marker.color = color
    return marker


def sphere(
    ns: str,
    marker_id: int,
    stamp,
    *,
    x: float,
    y: float,
    z: float = 0.0,
    diameter: float,
    color: ColorRGBA,
) -> Marker:
    """A ball. Used for traffic light lamps."""
    marker = _base(ns, marker_id, Marker.SPHERE, stamp)
    marker.pose = _pose(x, y, z)
    marker.scale = Vector3(x=float(diameter), y=float(diameter), z=float(diameter))
    marker.color = color
    return marker


def line_strip(
    ns: str,
    marker_id: int,
    stamp,
    *,
    points: list[tuple[float, float]],
    width: float,
    color: ColorRGBA,
    z: float = 0.0,
) -> Marker:
    """A connected run of line segments, e.g. a road edge or a rail."""
    marker = _base(ns, marker_id, Marker.LINE_STRIP, stamp)
    marker.scale = Vector3(x=float(width), y=0.0, z=0.0)
    marker.color = color
    marker.points = [_point(px, py, z) for px, py in points]
    return marker


def line_list(
    ns: str,
    marker_id: int,
    stamp,
    *,
    segments: list[tuple[tuple[float, float], tuple[float, float]]],
    width: float,
    color: ColorRGBA,
    z: float = 0.0,
) -> Marker:
    """Disconnected segments in one marker: dashes, sleepers, zebra stripes."""
    marker = _base(ns, marker_id, Marker.LINE_LIST, stamp)
    marker.scale = Vector3(x=float(width), y=0.0, z=0.0)
    marker.color = color
    marker.points = []
    for start, end in segments:
        marker.points.append(_point(start[0], start[1], z))
        marker.points.append(_point(end[0], end[1], z))
    return marker


def text(
    ns: str,
    marker_id: int,
    stamp,
    *,
    x: float,
    y: float,
    z: float = 0.4,
    content: str,
    height: float,
    color: ColorRGBA,
) -> Marker:
    """Readable text on the map, e.g. the printed lane numbers."""
    marker = _base(ns, marker_id, Marker.TEXT_VIEW_FACING, stamp)
    marker.pose = _pose(x, y, z)
    marker.scale = Vector3(x=0.0, y=0.0, z=float(height))
    marker.color = color
    marker.text = content
    return marker


def delete(ns: str, marker_id: int, stamp) -> Marker:
    """Remove a previously published marker, e.g. when a vehicle goes away."""
    marker = _base(ns, marker_id, Marker.CUBE, stamp)
    marker.action = Marker.DELETE
    return marker


def stable_id(key: str) -> int:
    """Map an arbitrary entity id to a stable marker id.

    Markers are addressed by (ns, id), so a vehicle needs the same integer every
    frame or it would be redrawn as a new marker each time instead of moving.
    """
    digest = 0
    for char in key:
        digest = (digest * 131 + ord(char)) & 0x7FFFFFF
    return digest
