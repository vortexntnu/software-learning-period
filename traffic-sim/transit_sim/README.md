# transit_sim

The visual layer of the learning period: a top-down view of a small city where
your node comes alive as a moving car or a cycling traffic light.

The sim **owns the map and nothing else**. It does not decide how any vehicle
moves or what any light does — it subscribes to what your node publishes and
draws it. If the sim is off, the city still runs; you just can't see it.

Markers are plain `visualization_msgs/MarkerArray`, so **Foxglove** and **RViz2**
both render them with no change to the sim.

---

## Running it

```bash
cd ~/ros2_ws
colcon build --packages-select transit_msgs transit_sim
source install/setup.bash
ros2 launch transit_sim transit_sim.launch.py
```

That starts three things: the sim, a static transform for the `map` frame, and
the Foxglove bridge on port 8765.

### See it moving

There are no vehicle or light nodes yet — that's what the recruits build — so the
city starts empty. To watch the whole thing work, run the demo driver in a second
terminal:

```bash
cd ~/ros2_ws && source install/setup.bash && \
  python3 src/control-autonomy-learning-period/transit_sim/scripts/drive_city.py
```

Six cars circulate, the four lights cycle north-south then east-west, cars hold at
their stop line on red, and a train runs the rail line every 30 seconds so you can
watch the level-crossing barriers swing down and back up.

Every decision in that script — when to stop, when to go, when the light changes —
happens **in the script, not in the sim**. That is the point: the sim is only
drawing what it is told. `scripts/drive_city.py` is a throwaway stand-in for the
nodes recruits will write, and can be deleted once real ones exist.

### Viewing in Foxglove

1. Install the [Foxglove desktop app](https://foxglove.dev/download). Prefer the
   desktop app over the web app — a browser on `https://` will often refuse the
   plain `ws://` connection as mixed content.
2. Open a connection to `ws://localhost:8765`.
3. Import the layout: **Layouts → Import from file →**
   `transit_sim/config/transit_city.json`. It sets up a top-down orthographic 3D
   panel plus raw views of both state topics.

If the import misbehaves, add a **3D** panel by hand, enable the
`/transit_sim/markers` and `/transit_sim/static_markers` topics, set the display
frame to `map`, and turn off the perspective camera to look straight down.

**Running ROS in Docker?** That's the reason we use Foxglove rather than a
desktop window: no X11 forwarding needed. Just expose the port:

```bash
docker run -p 8765:8765 ...
```

### Viewing in RViz2 instead

```bash
ros2 launch transit_sim transit_sim.launch.py bridge:=false
rviz2
```

Add two **MarkerArray** displays for `/transit_sim/markers` and
`/transit_sim/static_markers`, and set the fixed frame to `map`.

---

## What you publish

The sim listens to two topics. Both are shared: **everyone publishes to the same
topic**, keyed by id. That is what makes a new vehicle appear with no change to
the sim.

### `/vehicle_state` — `transit_msgs/msg/VehicleState`

```
string  vehicle_id     # unique, e.g. "car_anbit"
uint16  lane_id        # which numbered lane (the numbers printed on the map)
float64 progress       # 0.0 .. 1.0 along that lane
float64 velocity       # m/s, informational
string  color          # colour name, see below
bool    moving         # false draws "(stopped)" next to your vehicle
```

You never send screen coordinates. You say *which lane* and *how far along*, and
the sim converts that to a position using the lane geometry it owns. This is why
the lane numbers are printed on the map: if your car says lane 2 but appears on
the wrong road, you can see it instantly.

Colour names: `black`, `blue`, `brown`, `cyan`, `gray`, `green`, `grey`,
`orange`, `pink`, `purple`, `red`, `silver`, `white`, `yellow`. Anything else
draws **magenta** on purpose, so a typo is obvious rather than silently fine.

### `/signal_state` — `transit_msgs/msg/SignalState`

```
uint8 RED=0
uint8 YELLOW=1
uint8 GREEN=2

string signal_id       # unique, e.g. "junction1_north"
uint16 lane_id         # the incoming lane this light governs
uint8  state
```

A light is identified by the lane it governs, and the sim places it at that
lane's stop line, offset to the **right** of the direction of travel (European
convention). Every junction approach is drawn from the moment the sim starts; a
light with nobody driving it sits with all three lamps dim. If your light node
dies, its light goes dim rather than disappearing.

### Stale entities

Anything the sim hasn't heard from for **2 seconds** is removed. Kill your node
and your car leaves the map. Publish at a few Hz or better.

---

## The map

| lane | direction | notes |
|---|---|---|
| 1 | northbound | vertical road, east side |
| 2 | southbound | vertical road, west side |
| 3 | eastbound | horizontal road, south side |
| 4 | westbound | horizontal road, north side |
| 5 | railway | runs parallel to the vertical road, west of it |

The two roads meet at one four-way junction, so there are four traffic lights.
The railway crosses the horizontal road at a level crossing, whose barriers come
down automatically when a train on lane 5 gets close.

**The train is not a special entity.** It is a `VehicleState` on lane 5 — the sim
draws a locomotive plus carriages because the lane is a rail lane. Growing the
city means adding rows to `LANES` in `city_map.py`, not changing the renderer.

---

## Testing without writing a node

Drive a car along lane 1:

```bash
for p in $(seq 0 0.01 1); do
  ros2 topic pub --once /vehicle_state transit_msgs/msg/VehicleState \
    "{vehicle_id: 'test_car', lane_id: 1, progress: $p, velocity: 8.0, color: 'red', moving: true}"
done
```

Turn the north approach green:

```bash
ros2 topic pub -r 5 /signal_state transit_msgs/msg/SignalState \
  "{signal_id: 'junction1_north', lane_id: 1, state: 2}"
```

Send a train through the level crossing and watch the barriers drop:

```bash
for p in $(seq 0 0.01 1); do
  ros2 topic pub --once /vehicle_state transit_msgs/msg/VehicleState \
    "{vehicle_id: 'test_train', lane_id: 5, progress: $p, velocity: 20.0, color: 'blue', moving: true}"
done
```

Note that `ros2 topic pub` on its own draws a *parked* vehicle — you have to
sweep `progress` to see motion, because the sim never integrates velocity for
you. Moving your vehicle is your node's job.

---

## Adding a new kind of entity

Most work needs **no sim change at all**. A new vehicle or a new light is just
another id on an existing topic.

You only need to touch the sim if you are drawing something it has never drawn —
a pedestrian, a boat, a new kind of signal. That is two steps, and the render
loop is never edited.

**1. Add a handler file** in `transit_sim/handlers/`:

```python
# transit_sim/handlers/pedestrian.py
from transit_msgs.msg import PedestrianState

from transit_sim import city_map, colors, markers
from transit_sim.handlers.base import EntityHandler


class PedestrianHandler(EntityHandler):
    msg_type = PedestrianState
    topic = '/pedestrian_state'
    ns = 'pedestrians'

    def key_of(self, msg):
        return msg.pedestrian_id

    def draw(self, key, msg, stamp):
        lane = city_map.lane(msg.lane_id)
        if lane is None:
            return []
        x, y, heading = lane.pose_at(msg.progress)
        return [
            markers.sphere(
                self.ns, self.slot(key, 0), stamp,
                x=x, y=y, z=0.9, diameter=0.6,
                color=colors.by_name('yellow'),
            )
        ]
```

**2. Add one line** to `HANDLERS` in `transit_sim/handlers/__init__.py`:

```python
HANDLERS = (
    VehicleHandler,
    SignalHandler,
    LevelCrossingHandler,
    PedestrianHandler,   # <- your line
)
```

That's it. The sim subscribes to your topic, calls your `draw` every frame, gives
your entity stable marker ids via `self.slot(key, i)`, and deletes it when it
goes quiet.

Look at `handlers/vehicle.py` for the simplest example, and
`handlers/crossing.py` for one that has no message of its own and derives its
state by watching another topic.

---

## Layout

| file | what it holds |
|---|---|
| `city_map.py` | the map: lanes, junction, rail line, level crossing, signal placement |
| `geometry.py` | lane progress → position, and the right-hand-side offset rule |
| `colors.py` | every colour in the city, in one place |
| `markers.py` | small marker builders so handlers never fill the message by hand |
| `static_scene.py` | grass, roads, dashes, lane numbers, rail, station, houses |
| `handlers/` | one file per drawable entity, plus the registry |
| `sim_node.py` | the node: subscribes what handlers declare, publishes what they draw |

## Tests

```bash
cd src/control-autonomy-learning-period/transit_sim
python3 -m pytest test/ -q
```
