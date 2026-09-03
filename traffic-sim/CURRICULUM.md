# Recruit curriculum

A task list for building the real nodes that drive this city. It assumes
you've already done the root [`software-learning-period`](../README.md)
pub/sub tutorial (workspace setup, a publisher, a subscriber, a launch file)
— this picks up from there with a real payload instead of `std_msgs/String`.

Read [`transit_sim/README.md`](transit_sim/README.md) first. It documents
the message fields, the map, and the "testing without writing a node"
`ros2 topic pub` one-liners this list leans on. This file is the order to
attempt things in; that one is the reference you'll keep coming back to.

Don't start from a blank file. [`transit_starter/`](transit_starter/) has
two nodes, `vehicle_node_ros` and `signal_node_ros`, with the ROS 2 plumbing
(node, publisher, subscription, timer) already wired up and a
`TODO (Task N)` comment at each spot a task below wants you to add code —
C++ / `ament_cmake`, like the rest of this repo's tutorial packages
(`talker`, `listener`, `simple_publisher`); only the launch file is Python,
which is normal regardless of what language the nodes are written in. Every
task below names exactly which file and TODO to go fill in.

Every task is checkable on its own, without needing anyone else's node
running yet. `scripts/drive_city.py` is the answer key — it already does
everything this list asks you to build (in Python, as a throwaway demo, not
as a style guide). Use it to check your work *after* you've had a go, not
to copy from directly; the point is to derive the logic yourself.

---

## Task 0 — Orientation

Build and run the sim, then the demo driver, per the quick start in the
top-level [`README.md`](README.md). Watch it work for a minute. Then kill
`drive_city.py` (Ctrl-C in its terminal — your cars will fade off the map
after 2s) and, with the sim still running, poke at the raw topics yourself:

```bash
ros2 topic echo /vehicle_state
ros2 topic echo /signal_state
```

Then try the `ros2 topic pub` examples in `transit_sim/README.md`'s
"Testing without writing a node" section. No code yet — the goal is to see
the wire format before you write to it.

**Verify:** you can explain, in one sentence, what `lane_id` and `progress`
mean without looking anything up.

## Task 1 — Publish one `VehicleState`

Start here: **`transit_starter/src/vehicle_node_ros.cpp`, the
`TODO (Task 1)` in `tick()`**. Set `vehicle_id_` / `lane_id_` / `color_`
near the top of `vehicle_node_ros.hpp`, then build and publish a
`VehicleState` with `moving = false`.

**Verify:** your car appears in Foxglove/RViz2, parked, labeled
`your_id (stopped)`, on the lane number you chose.

## Task 2 — Make it drive

Same file, same `tick()` — the **`TODO (Task 2)`** just below Task 1's.
Advance `progress_` a little each call using `speed_` and `lane_length_`
from the header, wrapping back to `0.0` at `1.0`. Set `moving = true` and a
real velocity. There's no C++ accessor for the sim's map (it's a
Python-internal detail transit_sim owns), so measure your lane's length
yourself: sweep `progress` with `ros2 topic pub` (as in Task 0) and watch
in Foxglove until you're confident, then hardcode `lane_length_`.

**Verify:** your car loops its lane continuously instead of sitting still.

## Task 3 — One traffic light

Switch files: **`transit_starter/src/signal_node_ros.cpp`, the
`TODO (Task 3)` in `tick()`**. Publish `SignalState` for `lane_id_`, cycling
RED → GREEN → YELLOW → RED against `green_seconds_` / `yellow_seconds_` /
`all_red_seconds_` (in `signal_node_ros.hpp`).

**Verify:** your lane's light changes color on schedule. The other three
approaches stay dim — that's the sim's built-in fallback for a light nobody
is driving yet, not a bug in your node.

## Task 4 — The full junction

Same file, the **`TODO (Task 4)`** just below Task 3's. Drive all four
approaches at once, with correct opposing-pair phasing: lanes 1 & 2
(north/south) green together, then lanes 3 & 4 (east/west) green together,
never all four at once. Try to derive the phase shape yourself before
checking it against `PHASES` in `drive_city.py`.

**Verify:** all four lights at the junction cycle correctly, and a light
is never green on two lanes that cross each other.

## Task 5 — Obey the light

Back to `vehicle_node_ros.cpp`: uncomment the subscription in the
`VehicleNode` constructor, then fill in **`on_signal` and the
`TODO (Task 5)` in `tick()`**. Hold at the stop line when your light is red
or yellow — compare against `transit_msgs::msg::SignalState::RED` /
`YELLOW` / `GREEN`. You'll need roughly where your lane's stop line sits as
a `progress` fraction; find it the same empirical way as `lane_length_` in
Task 2. (`drive_city.py`'s `stop_progress()` shows the pattern if you get
stuck, once you've had a go yourself — it cheats by importing the sim's
Python-internal map directly, which your C++ node can't do.)

**Verify:** your car stops just short of the junction box on red, and
proceeds cleanly on green — not stopped on top of the junction, not
overshooting it.

## Task 6 — Stretch: multiple vehicles, one node

No new TODO for this one — extend what you wrote for Tasks 1, 2 and 5 in
`vehicle_node_ros.cpp` to drive several vehicles (unique `vehicle_id`s and
colors) from a single process, each independently obeying its own lane's
light.

## Task 7 — Stretch: level crossing

Also freehand, in `vehicle_node_ros.cpp` or a copy of it: publish a train —
just a `VehicleState` on lane 5 — on a timer, e.g. every 30s. No new
message type or sim change needed.

**Verify:** the level-crossing barriers swing down as your train
approaches and back up after it clears, with no code on your side beyond
publishing lane-5 states. If this works, it's confirmation you understood
"the train is not a special entity" rather than just having pattern-matched
Tasks 1–2.

## Task 8 — Integration

Run your vehicle node next to a teammate's light node (or both of your own)
— `transit_starter/launch/transit_starter.launch.py` launches both at
once, if that's convenient. Confirm the junction behaves correctly
end-to-end as two independent processes, then retire `drive_city.py` for
good — the city no longer needs its stand-in.

---

## Where your code goes

`transit_starter/` is a template, not where your finished work should
live. Copy it into [`your_code/`](../your_code/) at the repo root, same
convention as the C++ tutorial packages, and rename the package — see
[`transit_starter/README.md`](transit_starter/README.md) for the exact
steps. Leave `traffic-sim/` itself untouched; it's the sim and its
starter template, not recruit code.

`transit_starter` is C++ (`rclcpp`) with `ament_cmake`, matching the rest
of this repo's tutorial packages. `transit_msgs` generates bindings for
both C++ and Python, so a Python (`rclpy`) node is technically possible too
— `drive_city.py` and `transit_sim` itself are Python — but it's the
exception here, not the convention; stick to C++ unless you have a specific
reason not to.
