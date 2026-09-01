"""Draws traffic lights from `SignalState`.

Every junction approach on the map gets a light from the moment the sim starts.
Until a node claims it, all three lamps sit dim, which is an honest picture:
nobody is driving that light yet. The same happens if a recruit's light node
dies, rather than the light vanishing off the map.

The lamps are laid out along the road surface rather than stacked vertically,
because the city is viewed from directly above and a vertical mast would be
invisible.
"""

import math

from visualization_msgs.msg import Marker

from transit_msgs.msg import SignalState
from transit_sim import city_map, colors, markers
from transit_sim.geometry import offset
from transit_sim.handlers.base import EntityHandler

LAMP_SPACING = 1.15
LAMP_DIAMETER = 0.95
HOUSING_LENGTH = 4.0
HOUSING_WIDTH = 1.5
HOUSING_Z = 0.3

_LIT = {
    SignalState.RED: colors.LAMP_RED,
    SignalState.YELLOW: colors.LAMP_YELLOW,
    SignalState.GREEN: colors.LAMP_GREEN,
}

#: Lamp order along the approach: red furthest upstream, green nearest the junction.
_ORDER = (SignalState.RED, SignalState.YELLOW, SignalState.GREEN)


class SignalHandler(EntityHandler):
    """One traffic light per junction approach, keyed by the lane it governs."""

    msg_type = SignalState
    topic = '/signal_state'
    ns = 'signals'
    slots = 8

    def __init__(self, node) -> None:
        """Note every junction approach the map defines, so each gets a light."""
        super().__init__(node)
        self._approaches: set[str] = set()
        for junction in city_map.JUNCTIONS:
            for lane_id in junction.approach_lane_ids:
                self._approaches.add(str(lane_id))

    def key_of(self, msg: SignalState) -> str:
        """Lights are identified by the lane they govern."""
        return str(msg.lane_id)

    def on_message(self, msg: SignalState) -> None:
        """Record the light's state, warning if it names a lane off the map."""
        if city_map.signal_pose(msg.lane_id) is None:
            self.node.get_logger().warn(
                f"signal '{msg.signal_id}' claims lane {msg.lane_id}, which is not a"
                ' junction approach on the map',
                throttle_duration_sec=5.0,
            )
            return
        super().on_message(msg)

    def on_retire(self, key: str, stamp) -> list[Marker]:
        """Draw nothing: the dim fallback below redraws this light the same tick."""
        return []

    def markers(self, stamp) -> list[Marker]:
        """Draw claimed lights from their messages, and every other approach dim."""
        out = super().markers(stamp)
        for key in sorted(self._approaches):
            if key not in self._latest:
                out += self._draw_light(
                    key, state=None, label=f'lane {key}', stamp=stamp
                )
        return out

    def draw(self, key: str, msg: SignalState, stamp) -> list[Marker]:
        """Draw a light with the lamp matching its state lit."""
        return self._draw_light(key, state=msg.state, label=msg.signal_id, stamp=stamp)

    def _draw_light(self, key: str, state, label: str, stamp) -> list[Marker]:
        pose = city_map.signal_pose(int(key))
        if pose is None:
            return []
        x, y, heading = pose
        along = (math.cos(heading), math.sin(heading))

        out = [
            markers.cube(
                self.ns,
                self.slot(key, 0),
                stamp,
                x=x,
                y=y,
                z=HOUSING_Z,
                size=(HOUSING_LENGTH, HOUSING_WIDTH, 0.35),
                color=colors.rgba(colors.LAMP_HOUSING),
                yaw=heading,
            )
        ]

        for index, lamp in enumerate(_ORDER):
            centre = offset((x, y), along, (index - 1) * LAMP_SPACING)
            lit = state is not None and lamp == state
            color = colors.rgba(_LIT[lamp] if lit else colors.LAMP_OFF)
            out.append(
                markers.sphere(
                    self.ns,
                    self.slot(key, 1 + index),
                    stamp,
                    x=centre[0],
                    y=centre[1],
                    z=HOUSING_Z + 0.3,
                    diameter=LAMP_DIAMETER,
                    color=color,
                )
            )

        behind = offset((x, y), along, -(HOUSING_LENGTH / 2.0 + 1.0))
        out.append(
            markers.text(
                self.ns,
                self.slot(key, 4),
                stamp,
                x=behind[0],
                y=behind[1],
                z=1.2,
                content=label,
                height=1.0,
                color=colors.rgba(colors.LABEL, 0.85),
            )
        )
        return out
