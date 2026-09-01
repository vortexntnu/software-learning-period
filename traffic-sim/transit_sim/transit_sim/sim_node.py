"""The sim node: a pure subscriber that draws whatever the city is saying.

It owns the map and nothing else. It never decides how a vehicle moves or what a
light does; it listens to the topics the handlers declare and turns what it hears
into markers (SIMULATION_PLAN.md section 1).

Markers are `visualization_msgs/MarkerArray`, which Foxglove's 3D panel and RViz2
both render natively, so the sim is not tied to either viewer.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from transit_sim.handlers import HANDLERS
from transit_sim.static_scene import build_static_scene

STATIC_TOPIC = '/transit_sim/static_markers'
DYNAMIC_TOPIC = '/transit_sim/markers'


class TransitSim(Node):
    """Draws the city from the state its inhabitants publish."""

    def __init__(self) -> None:
        """Set up publishers, build the static scene and wire up the handlers."""
        super().__init__('transit_sim')

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('static_republish_period', 2.0)
        rate = self.get_parameter('publish_rate').value
        static_period = self.get_parameter('static_republish_period').value

        # Latched, so a viewer that connects after startup still gets the city.
        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._static_pub = self.create_publisher(MarkerArray, STATIC_TOPIC, latched)
        self._dynamic_pub = self.create_publisher(MarkerArray, DYNAMIC_TOPIC, 10)

        self._handlers = [handler(self) for handler in HANDLERS]
        self._subscribe()

        self._publish_static(first=True)
        self.create_timer(static_period, self._publish_static)
        self.create_timer(1.0 / rate, self._publish_dynamic)

        topics = ', '.join(sorted({handler.topic for handler in self._handlers}))
        self.get_logger().info(
            f'transit_sim drawing the city from {topics} '
            f'-> {DYNAMIC_TOPIC} at {rate:g} Hz'
        )

    def _subscribe(self) -> None:
        """One subscription per topic, fanned out to every handler that wants it.

        Handlers may share a topic: the level crossing watches the same
        `VehicleState` stream the vehicles are drawn from.
        """
        by_topic: dict[tuple[str, type], list] = {}
        for handler in self._handlers:
            by_topic.setdefault((handler.topic, handler.msg_type), []).append(handler)

        self._subscriptions_kept = []
        for (topic, msg_type), handlers in by_topic.items():

            def callback(msg, handlers=handlers):
                for handler in handlers:
                    handler.on_message(msg)

            self._subscriptions_kept.append(
                self.create_subscription(msg_type, topic, callback, 10)
            )

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _publish_static(self, first: bool = False) -> None:
        """Publish the unchanging city."""
        stamp = self._stamp()
        array = MarkerArray()
        if first:
            # Clear anything left over from a previous run of the sim.
            reset = Marker()
            reset.action = Marker.DELETEALL
            array.markers.append(reset)
        array.markers.extend(build_static_scene(stamp))
        self._static_pub.publish(array)

    def _publish_dynamic(self) -> None:
        """Publish everything the handlers currently want drawn."""
        stamp = self._stamp()
        array = MarkerArray()
        for handler in self._handlers:
            array.markers.extend(handler.markers(stamp))
        self._dynamic_pub.publish(array)


def main(args=None) -> None:
    """Spin the sim until interrupted."""
    rclpy.init(args=args)
    node = TransitSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
