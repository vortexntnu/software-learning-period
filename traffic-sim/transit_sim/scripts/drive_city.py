#!/usr/bin/env python3
"""Bring the city to life, so you can see the sim working.

This is a **demo driver, not part of the sim**. It stands in for the nodes the
recruits will write: it publishes `VehicleState` and `SignalState` exactly the way
their nodes will, and the sim draws whatever it hears. Delete this file once real
nodes exist.

It deliberately does all the deciding here, not in the sim: the cars stop at red
lights, the lights cycle, and the train triggers the crossing barriers. The sim
knows none of that. It only draws.

    ros2 launch transit_sim transit_sim.launch.py     # terminal 1
    python3 .../scripts/drive_city.py                 # terminal 2
"""

import sys

import rclpy
from rclpy.node import Node

from transit_msgs.msg import SignalState, VehicleState
from transit_sim import city_map

TICK = 0.05

# One full light cycle, as (lanes that are green, lanes on yellow, seconds).
PHASES = (
    ((1, 2), (), 9.0),
    ((), (1, 2), 2.0),
    ((), (), 1.0),
    ((3, 4), (), 9.0),
    ((), (3, 4), 2.0),
    ((), (), 1.0),
)

CARS = (
    ('car_red', 1, 'red', 9.0),
    ('car_blue', 1, 'blue', 7.0),
    ('car_white', 2, 'white', 8.0),
    ('car_orange', 3, 'orange', 8.5),
    ('car_green', 3, 'green', 6.5),
    ('car_purple', 4, 'purple', 9.5),
)

TRAIN_ID = 'night_train'
TRAIN_SPEED = 10.0
TRAIN_PERIOD = 30.0
FIRST_TRAIN = 8.0


def stop_progress(lane_id: int) -> float:
    """Return how far along a lane its stop line sits, as a 0..1 fraction."""
    stop = city_map.stop_line_pose(lane_id)
    lane = city_map.lane(lane_id)
    if stop is None or lane is None:
        return 2.0
    best, best_distance = 0.0, float('inf')
    for step in range(1001):
        progress = step / 1000.0
        x, y, _ = lane.pose_at(progress)
        distance = (x - stop[0]) ** 2 + (y - stop[1]) ** 2
        if distance < best_distance:
            best, best_distance = progress, distance
    return best


class Driver(Node):
    """Publishes the state of a handful of cars, four lights and a train."""

    def __init__(self) -> None:
        """Place the cars on their lanes and start the light cycle."""
        super().__init__('drive_city')
        self.vehicles = self.create_publisher(VehicleState, '/vehicle_state', 10)
        self.signals = self.create_publisher(SignalState, '/signal_state', 10)

        self.stop_at = {lane_id: stop_progress(lane_id) for lane_id in (1, 2, 3, 4)}
        self.cars = [
            {
                'id': name,
                'lane': lane_id,
                'color': color,
                'speed': speed,
                'progress': 0.08 * index,
            }
            for index, (name, lane_id, color, speed) in enumerate(CARS)
        ]
        self.light_state = {}
        self.elapsed = 0.0
        self.train_progress = None
        self.next_train = FIRST_TRAIN
        self.create_timer(TICK, self.tick)

        self.get_logger().info(
            'driving the city: 6 cars, 4 lights, a train every '
            f'{TRAIN_PERIOD:g}s. Ctrl-C to stop.'
        )

    def phase(self) -> None:
        """Advance the traffic light cycle and publish each light's state."""
        cycle = sum(step[2] for step in PHASES)
        position = self.elapsed % cycle
        green, yellow = (), ()
        for greens, yellows, duration in PHASES:
            if position < duration:
                green, yellow = greens, yellows
                break
            position -= duration

        for lane_id in (1, 2, 3, 4):
            if lane_id in green:
                state = SignalState.GREEN
            elif lane_id in yellow:
                state = SignalState.YELLOW
            else:
                state = SignalState.RED
            self.light_state[lane_id] = state
            self.signals.publish(
                SignalState(
                    signal_id=f'junction1_lane{lane_id}',
                    lane_id=lane_id,
                    state=state,
                )
            )

    def may_proceed(self, car) -> bool:
        """Decide whether a car is held at its stop line.

        This is the recruit's job, not the sim's. The sim has no idea a red light
        should stop anything.
        """
        stop = self.stop_at.get(car['lane'], 2.0)
        approaching = stop - 0.02 <= car['progress'] <= stop
        red = self.light_state.get(car['lane']) in (
            SignalState.RED,
            SignalState.YELLOW,
        )
        return not (approaching and red)

    def drive(self) -> None:
        """Move each car along its lane and publish where it got to."""
        for car in self.cars:
            lane = city_map.lane(car['lane'])
            moving = self.may_proceed(car)
            if moving:
                car['progress'] += car['speed'] * TICK / lane.length
                if car['progress'] >= 1.0:
                    car['progress'] = 0.0
            self.vehicles.publish(
                VehicleState(
                    vehicle_id=car['id'],
                    lane_id=car['lane'],
                    progress=car['progress'],
                    velocity=car['speed'] if moving else 0.0,
                    color=car['color'],
                    moving=moving,
                )
            )

    def run_train(self) -> None:
        """Send a train down the rail line every so often."""
        if self.train_progress is None:
            # Compare against a deadline rather than using a modulo of elapsed
            # time, which floating-point drift can step straight over.
            if self.elapsed >= self.next_train:
                self.train_progress = 0.0
                self.next_train = self.elapsed + TRAIN_PERIOD
                self.get_logger().info('train departing, barriers will come down')
            return

        lane = city_map.lane(5)
        self.train_progress += TRAIN_SPEED * TICK / lane.length
        if self.train_progress > 1.0:
            self.train_progress = None
            return

        self.vehicles.publish(
            VehicleState(
                vehicle_id=TRAIN_ID,
                lane_id=5,
                progress=self.train_progress,
                velocity=TRAIN_SPEED,
                color='black',
                moving=True,
            )
        )

    def tick(self) -> None:
        """One step of the whole city."""
        self.elapsed += TICK
        self.phase()
        self.drive()
        self.run_train()


def main() -> int:
    """Drive the city until interrupted."""
    rclpy.init()
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nstopped. The cars will fade off the map in 2s.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
