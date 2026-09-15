"""The handler registry: the one place the sim learns about a new entity type.

To make the sim draw something it has never drawn before:

1. Add a file next to this one with a class extending `EntityHandler`
   (see `vehicle.py` for the simplest example).
2. Add it to `HANDLERS` below.

That is the whole change. `sim_node.py` subscribes to whatever each handler
declares and merges whatever markers it returns, so the render loop never needs
editing (SIMULATION_PLAN.md section 7).
"""

from transit_sim.handlers.base import EntityHandler
from transit_sim.handlers.crossing import LevelCrossingHandler
from transit_sim.handlers.signal import SignalHandler
from transit_sim.handlers.vehicle import VehicleHandler

HANDLERS: tuple[type[EntityHandler], ...] = (
    VehicleHandler,
    SignalHandler,
    LevelCrossingHandler,
    # A new member adds one line here, plus one small handler file:
    # PedestrianHandler,
)

__all__ = [
    'HANDLERS',
    'EntityHandler',
    'LevelCrossingHandler',
    'SignalHandler',
    'VehicleHandler',
]
