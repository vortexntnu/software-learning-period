"""Draws vehicles from `VehicleState`.

Any node that publishes a `VehicleState` with a new `vehicle_id` shows up
automatically: the sim needs no change to make room for another recruit
(SIMULATION_PLAN.md section 7, tier 1).
"""

import math

from visualization_msgs.msg import Marker

from transit_msgs.msg import VehicleState
from transit_sim import city_map, colors, markers
from transit_sim.geometry import offset, right_of
from transit_sim.handlers.base import EntityHandler

CAR_LENGTH = 4.3
CAR_WIDTH = 1.9
CAR_HEIGHT = 1.3
CAR_Z = 0.7

CARRIAGE_LENGTH = 7.0
CARRIAGE_WIDTH = 2.6
CARRIAGE_GAP = 1.2
CARRIAGE_COUNT = 3


class VehicleHandler(EntityHandler):
    """One car (or train) per `vehicle_id`, placed by lane and progress."""

    msg_type = VehicleState
    topic = '/vehicle_state'
    ns = 'vehicles'
    slots = 8

    def key_of(self, msg: VehicleState) -> str:
        """Vehicles are distinguished by their id."""
        return msg.vehicle_id

    def draw(self, key: str, msg: VehicleState, stamp) -> list[Marker]:
        """Draw one vehicle at its position along its lane."""
        target_lane = city_map.lane(msg.lane_id)
        if target_lane is None:
            self.node.get_logger().warn(
                f"vehicle '{key}' claims lane {msg.lane_id}, which is not on the map",
                throttle_duration_sec=5.0,
            )
            return self.on_retire(key, stamp)

        if not colors.is_known(msg.color):
            self.node.get_logger().warn(
                f"vehicle '{key}' has unknown colour '{msg.color}', drawing magenta"
                f' (known: {", ".join(colors.known_names())})',
                throttle_duration_sec=5.0,
            )

        x, y, heading = target_lane.pose_at(msg.progress)
        body = colors.by_name(msg.color)

        if target_lane.kind == 'rail':
            out = self._train(key, stamp, x, y, heading, body)
        else:
            out = self._car(key, stamp, x, y, heading, body)

        label = key if msg.moving else f'{key} (stopped)'
        side = offset((x, y), right_of(heading), -(CAR_WIDTH + 1.1))
        out.append(
            markers.text(
                self.ns,
                self.slot(key, 7),
                stamp,
                x=side[0],
                y=side[1],
                z=1.6,
                content=label,
                height=1.2,
                color=colors.rgba(colors.LABEL),
            )
        )
        return out

    def _car(self, key, stamp, x, y, heading, body) -> list[Marker]:
        cabin_at = offset((x, y), (math.cos(heading), math.sin(heading)), -0.35)
        return [
            markers.cube(
                self.ns,
                self.slot(key, 0),
                stamp,
                x=x,
                y=y,
                z=CAR_Z,
                size=(CAR_LENGTH, CAR_WIDTH, CAR_HEIGHT),
                color=body,
                yaw=heading,
            ),
            markers.cube(
                self.ns,
                self.slot(key, 1),
                stamp,
                x=cabin_at[0],
                y=cabin_at[1],
                z=CAR_Z + 0.75,
                size=(CAR_LENGTH * 0.45, CAR_WIDTH * 0.8, 0.3),
                color=colors.rgba(colors.ASPHALT, 0.9),
                yaw=heading,
            ),
        ]

    def _train(self, key, stamp, x, y, heading, body) -> list[Marker]:
        """A locomotive plus carriages, trailing back along the lane."""
        along = (math.cos(heading), math.sin(heading))
        out = [
            markers.cube(
                self.ns,
                self.slot(key, 0),
                stamp,
                x=x,
                y=y,
                z=1.2,
                size=(CARRIAGE_LENGTH + 1.5, CARRIAGE_WIDTH, 2.4),
                color=body,
                yaw=heading,
            )
        ]
        for index in range(1, CARRIAGE_COUNT + 1):
            back = (
                (CARRIAGE_LENGTH + 1.5) / 2.0
                + index * (CARRIAGE_LENGTH + CARRIAGE_GAP)
                - CARRIAGE_LENGTH / 2.0
            )
            position = offset((x, y), along, -back)
            out.append(
                markers.cube(
                    self.ns,
                    self.slot(key, index),
                    stamp,
                    x=position[0],
                    y=position[1],
                    z=1.1,
                    size=(CARRIAGE_LENGTH, CARRIAGE_WIDTH, 2.2),
                    color=colors.by_name('silver'),
                    yaw=heading,
                )
            )
        return out
