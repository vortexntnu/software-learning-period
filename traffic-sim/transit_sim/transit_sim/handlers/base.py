"""The handler interface every drawable entity implements.

The sim never grows a special case for a new kind of entity. It walks the
registry in `handlers/__init__.py`, subscribes to whatever each handler declares,
and merges whatever markers each handler produces. Adding a pedestrian, a boat or
a new kind of signal is one new file plus one line in that registry, and the
render loop is never touched (SIMULATION_PLAN.md section 7).
"""

from abc import ABC, abstractmethod
from typing import Any, ClassVar

from rclpy.node import Node
from visualization_msgs.msg import Marker

from transit_sim import markers


class EntityHandler(ABC):
    """Turns messages of one type into markers.

    Subclasses declare what to subscribe to, then implement `key_of` and `draw`.
    Tracking which entities are live, giving them stable marker ids and cleaning
    up after ones that go quiet is handled here.
    """

    #: The message type to subscribe to, e.g. `VehicleState`.
    msg_type: ClassVar[type]
    #: The topic recruits publish it on, e.g. `/vehicle_state`.
    topic: ClassVar[str]
    #: Marker namespace, so one handler's markers never collide with another's.
    ns: ClassVar[str]
    #: How many markers one entity may draw. Bounds its block of marker ids.
    slots: ClassVar[int] = 8
    #: Seconds of silence after which an entity is considered gone.
    timeout_sec: ClassVar[float] = 2.0

    def __init__(self, node: Node) -> None:
        """Store the node, for its clock and logger."""
        self.node = node
        self._latest: dict[str, Any] = {}
        self._last_seen: dict[str, float] = {}

    # --- to implement -----------------------------------------------------

    @abstractmethod
    def key_of(self, msg: Any) -> str:
        """Return the id that distinguishes one entity of this type from another."""

    @abstractmethod
    def draw(self, key: str, msg: Any, stamp) -> list[Marker]:
        """Return the markers for one entity. Use `self.slot` for marker ids."""

    # --- provided ---------------------------------------------------------

    def now(self) -> float:
        """Current time in seconds."""
        return self.node.get_clock().now().nanoseconds / 1e9

    def slot(self, key: str, index: int) -> int:
        """Return a stable marker id for one of an entity's markers.

        The same entity must get the same ids every frame, otherwise a moving car
        would leave a trail of stale markers instead of moving.
        """
        if not 0 <= index < self.slots:
            raise ValueError(f'slot {index} outside 0..{self.slots - 1}')
        return markers.stable_id(key) * self.slots + index

    def on_message(self, msg: Any) -> None:
        """Record the newest state for an entity. Called by the sim node."""
        key = self.key_of(msg)
        self._latest[key] = msg
        self._last_seen[key] = self.now()

    def is_stale(self, key: str) -> bool:
        """Return whether an entity has gone quiet for longer than the timeout."""
        last = self._last_seen.get(key)
        return last is None or (self.now() - last) > self.timeout_sec

    def on_retire(self, key: str, stamp) -> list[Marker]:
        """Markers to publish when an entity goes away. Deletes it by default."""
        return [
            markers.delete(self.ns, self.slot(key, i), stamp) for i in range(self.slots)
        ]

    def markers(self, stamp) -> list[Marker]:
        """Return every marker this handler currently wants drawn."""
        out: list[Marker] = []
        for key in list(self._latest):
            if self.is_stale(key):
                out += self.on_retire(key, stamp)
                self._latest.pop(key, None)
                self._last_seen.pop(key, None)
                continue
            out += self.draw(key, self._latest[key], stamp)
        return out
