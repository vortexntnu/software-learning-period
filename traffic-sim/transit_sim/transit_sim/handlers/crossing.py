"""Lowers the level-crossing barriers when a train approaches.

This handler is an example of an entity that has no message of its own. It
watches `VehicleState` on the rail lane and derives its state from the train's
position, which is why it overrides `markers` rather than drawing per-entity.

The booms swing in the plane of the map rather than tipping up, because the city
is viewed from above and a raised boom pointing at the camera would be invisible.
Parallel to the road means open, across the road means closed.
"""

import math

from visualization_msgs.msg import Marker

from transit_msgs.msg import VehicleState
from transit_sim import city_map, colors, markers
from transit_sim.handlers.base import EntityHandler

#: How close the train has to be before the barriers come down, in metres.
APPROACH_DISTANCE = 26.0
#: Seconds for a boom to travel from fully open to fully closed.
SWING_TIME = 1.4
#: Flashes per second of the warning lamps while the crossing is active.
FLASH_HZ = 1.4

BOOM_WIDTH = 0.32
BOOM_HEIGHT = 0.3
BOOM_Z = 1.0


class LevelCrossingHandler(EntityHandler):
    """The barriers at the level crossing, driven by the train's position."""

    msg_type = VehicleState
    topic = '/vehicle_state'
    ns = 'level_crossing'
    slots = 8

    def __init__(self, node) -> None:
        """Start with the barriers open and no train seen."""
        super().__init__(node)
        self._closed = 0.0
        self._last_tick: float | None = None

    def key_of(self, msg: VehicleState) -> str:
        """Trains are tracked by vehicle id, like any other vehicle."""
        return msg.vehicle_id

    def on_message(self, msg: VehicleState) -> None:
        """Only rail vehicles are of interest, everything else is ignored."""
        target_lane = city_map.lane(msg.lane_id)
        if target_lane is None or target_lane.kind != 'rail':
            return
        super().on_message(msg)

    def draw(self, key: str, msg: VehicleState, stamp) -> list[Marker]:
        """Trains are drawn by the vehicle handler, not here."""
        return []

    def on_retire(self, key: str, stamp) -> list[Marker]:
        """Nothing per-train is drawn, so a departed train leaves nothing behind."""
        return []

    def _train_is_near(self) -> bool:
        crossing = city_map.LEVEL_CROSSING
        for key, msg in self._latest.items():
            if self.is_stale(key):
                continue
            target_lane = city_map.lane(msg.lane_id)
            if target_lane is None:
                continue
            x, y, _ = target_lane.pose_at(msg.progress)
            if math.dist((x, y), crossing) <= APPROACH_DISTANCE:
                return True
        return False

    def _advance(self) -> None:
        """Move the booms toward their target position."""
        now = self.now()
        elapsed = 0.0 if self._last_tick is None else max(now - self._last_tick, 0.0)
        self._last_tick = now

        target = 1.0 if self._train_is_near() else 0.0
        step = elapsed / SWING_TIME if SWING_TIME > 0 else 1.0
        if self._closed < target:
            self._closed = min(self._closed + step, target)
        elif self._closed > target:
            self._closed = max(self._closed - step, target)

    def markers(self, stamp) -> list[Marker]:
        """Draw the two booms and their warning lamps at the current position."""
        super().markers(stamp)  # Expire trains that have gone quiet.
        self._advance()

        crossing_x, crossing_y = city_map.LEVEL_CROSSING
        road_half = city_map.LANE_WIDTH
        post_offset = road_half + 0.6
        boom_length = post_offset

        out: list[Marker] = []
        for index, side in enumerate((1.0, -1.0)):
            post_x = crossing_x + side * 3.4
            post_y = crossing_y + side * post_offset

            # Open: boom lies along the road, out of the way. Closed: across it.
            open_centre = (post_x - side * boom_length / 2.0, post_y)
            closed_centre = (post_x, post_y - side * boom_length / 2.0)

            centre_x = (
                open_centre[0] + (closed_centre[0] - open_centre[0]) * self._closed
            )
            centre_y = (
                open_centre[1] + (closed_centre[1] - open_centre[1]) * self._closed
            )
            yaw = self._closed * math.pi / 2.0 * side

            out.append(
                markers.cube(
                    self.ns,
                    self.slot('boom', index),
                    stamp,
                    x=centre_x,
                    y=centre_y,
                    z=BOOM_Z,
                    size=(boom_length, BOOM_WIDTH, BOOM_HEIGHT),
                    color=colors.rgba(colors.BARRIER),
                    yaw=yaw,
                )
            )

            lit = self._closed > 0.05 and (self.now() * FLASH_HZ) % 1.0 < 0.5
            lamp = colors.LAMP_RED if lit else colors.LAMP_OFF
            out.append(
                markers.sphere(
                    self.ns,
                    self.slot('lamp', index),
                    stamp,
                    x=post_x,
                    y=post_y,
                    z=1.5,
                    diameter=0.7,
                    color=colors.rgba(lamp),
                )
            )

        return out
