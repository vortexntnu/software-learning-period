# Learning Tasks

You build the whole city, one node at a time. By the end you will have
written a vehicle that drives its lane and obeys the lights, and a
controller that runs all four lights at the junction, and you will have the
two of them running together as separate processes.

Work through the tasks in order — each one builds directly on the last.
Before you start, you should have finished the tutorial in the main
[`software-learning-period`](../README.md) README: setting up a workspace,
writing a publisher, writing a subscriber, and writing a launch file. The
tasks here continue from that point. The difference is that you will send
real traffic data instead of plain text (`std_msgs/String`).

Read [`transit_sim/README.md`](transit_sim/README.md) first. It explains the
message fields, the map, and how to test things from the terminal with
`ros2 topic pub` before you have written any node at all. You will keep
going back to that file while you work. This file just tells you what order
to do things in.

You do not have to start with an empty file.
[`transit_starter/`](transit_starter/) already contains two nodes,
`vehicle_node_ros` and `signal_node_ros`. Everywhere you need to add code
there is a comment that says `TODO (Task N)`, and each task below tells you
exactly which file and which TODO to work on.

The nodes are C++.
**Up to Task 5 you never have to write a new function, and never have to
touch the `.hpp` files** — every function already exists and is already
called from the right place, so you only fill in what goes inside them.
Tasks 6 and 7 are stretch tasks and do let you restructure things.
[`transit_starter/README.md`](transit_starter/README.md) explains how that
skeleton fits together, and how to copy it into `your_code/` when you are
done.

This is which function belongs to which task:

| Vehicle node | |
| --- | --- |
| `set_subscribers_and_publisher()` | Tasks 1 and 5 |
| `publish_state()` | Task 1 |
| `advance_progress()` | Task 2 |
| `must_stop_for_light()`, `on_signal()` | Task 5 |

| Signal node | |
| --- | --- |
| `set_publisher()` | Task 3 |
| `publish_lights()` | Tasks 3 and 4 |
| `state_for_lane()` | Tasks 3 and 4 |

Anything not listed there is written for you. Some tasks below also send
you to
[`transit_starter/config/transit_params.yaml`](transit_starter/config/transit_params.yaml)
instead of to any C++, for values like your lane number and your speed.

**These get harder as you go, on purpose.** Task 1 spells out every line,
because it is the first ROS code you write here. By Task 4 you get the
requirement and the constraints, and are expected to work the logic out
yourself using what you already wrote as the reference. When a later task
says "the same as you did in Task 1", go and read your own Task 1 code
rather than looking for the answer again here.

Every task is checkable on its own — you never need a node you have not
written yet. `scripts/drive_city.py` already does everything this list asks
for, so it is the answer key: have a real go first, then compare. It is a
quick throwaway demo in Python, so do not read it as an example of good
style.

---

## Task 0 — Look around

Build and run the sim, then run the demo driver. The steps are under
"Running it" in [`transit_sim/README.md`](transit_sim/README.md). Watch it
for a minute. Then stop `drive_city.py` with Ctrl-C in its terminal. The
demo's cars vanish from the map about 2 seconds later, because the sim
drops anything it has stopped hearing from. Leave the sim itself running
and look at what was on the topics:

```bash
ros2 topic echo /vehicle_state
ros2 topic echo /signal_state
```

Then try the `ros2 topic pub` commands from the "Testing without writing a
node" section of `transit_sim/README.md`. You are not writing code yet. The
goal is just to see what the messages look like before you start sending
them yourself.

**Check:** you can say in one sentence what `lane_id` and `progress` mean,
without looking them up.

## Task 1 — Publish one `VehicleState`

Three places, all marked `TODO (Task 1)`.

**1. `config/transit_params.yaml`**, in the `vehicle:` block. Pick a
`vehicle_id`, the `lane_id` you want to drive on, and a `color`. No C++
involved.

**2. `set_subscribers_and_publisher()` in
`transit_starter/src/vehicle_node_ros.cpp`.** Create two things here:

- a publisher that sends `transit_msgs::msg::VehicleState` on the topic
  named by `vehicle_topic_`, stored in `vehicle_pub_`
- a timer that calls `tick()` every `1.0 / tick_hz_` seconds, stored in
  `timer_`

Both variables already exist in the header, so you write only the two lines
that fill them. The shape of each call is in
[`transit_starter/README.md`](transit_starter/README.md) under "The ROS 2
calls you will need" — write the timer period as
`std::chrono::duration<double>(1.0 / tick_hz_)`.

**3. `publish_state()` in the same file.** Make an empty `VehicleState`,
set its six fields from the member variables, and publish it with
`vehicle_pub_`. The field names are listed in the TODO comment and in
[`transit_msgs/msg/VehicleState.msg`](transit_msgs/msg/VehicleState.msg).

Delete the two `throw` lines as you go. They are there so an unfinished
node tells you which task it is waiting for instead of failing silently.

**Check:** your car shows up in Foxglove or RViz2, parked, labeled
`your_id (stopped)`, on the lane number you picked. It will not move yet —
that is Task 2.

## Task 2 — Make it drive

One function and one config value. The function is
**`advance_progress()`** in `vehicle_node_ros.cpp`, marked
`TODO (Task 2)`. It is empty, which is exactly why the car stands still.
`tick()` already calls it once per tick, so whatever you put in there
happens over and over.

Move `progress_` a little further along the lane on each call. Work the
step out from what you have: `speed_` is metres per second, `tick_hz_` is
how many times a second this runs, and `lane_length_` is how many metres
the whole lane is. `progress_` is not metres — it is a fraction of the
lane, `0.0` at the start and `1.0` at the end — so watch your units on the
way to it. When `progress_` runs past `1.0` the car has reached the end of
the lane, and should carry on from the start.

Set `moving_` and `velocity_` here too, or the map keeps drawing the car as
parked. You do not touch `publish_state()` again — it publishes whatever
`progress_`, `moving_` and `velocity_` happen to be when it runs.

The config value is in **`config/transit_params.yaml`**, marked
`TODO (Task 2)`: how long your lane actually is. The sim keeps its map to
itself and offers no way to ask, so measure the lane yourself. Publish
different
`progress` values with `ros2 topic pub` the way you did in Task 0, watch
where the car lands in Foxglove, and write the length you settle on into
`lane_length`. Change it, relaunch, look again — no rebuild needed.

**Check:** your car drives around its lane again and again instead of
standing still.

## Task 3 — One traffic light

Change files: **`transit_starter/src/signal_node_ros.cpp`**. Three
functions, all marked `TODO (Task 3)`, and they are laid out the same way
as the vehicle node.

**1. `set_publisher()`.** The same publisher-and-timer pair you wrote in
Task 1, with `SignalState` instead of `VehicleState` and `signal_pub_`
instead of `vehicle_pub_`. Read your own Task 1 code rather than starting
from scratch.

**2. `state_for_lane()`.** Write this one before the next. It answers a
single question: given the time right now, which colour should this lane
show? A light repeats one round forever — `green_seconds_` of green, then
`yellow_seconds_` of yellow, then `all_red_seconds_` of red, then the same
round again.

`tick()` keeps `elapsed_` up to date for you, but `elapsed_` only counts
up, and a light goes round and round. Turning one into the other is what
the remainder of a division does: `std::fmod(a, b)` from `<cmath>` is the
remainder of `a / b` for doubles, and it tells you how far into the current
round you are. Comparing that against the three phase lengths gives you the
colour.

For now you can ignore the `lane` argument and give every lane the same
answer.

**3. `publish_lights()`.** Fill in a `SignalState` and publish it, taking
the colour from `state_for_lane(lane_id_)`. Its three fields are in
[`transit_msgs/msg/SignalState.msg`](transit_msgs/msg/SignalState.msg).

Which lane you light first and how long each phase lasts are in
**`config/transit_params.yaml`** under `signal:`, marked `TODO (Task 3)`.

**Check:** your lane's light changes color on time. The other three
directions stay dim. That is not a bug in your node. It is what the sim
shows for a light that nobody is controlling yet.

## Task 4 — The whole junction

Same file, and the same two functions you just wrote. Both have a
`TODO (Task 4)` under the Task 3 one.

**`publish_lights()`:** every approach needs its own message, so send four
per tick instead of one, each with its own `lane_id`, its own `signal_id`,
and its own colour.

**`state_for_lane()`:** the `lane` argument you ignored in Task 3 now has
to matter. Two rules define a junction:

1. Lanes that face each other share a phase — they go green together.
2. Lanes that cross each other are **never** green at the same time. That
   is a crash, and it is the one thing this task is really testing.

The lane table in [`transit_sim/README.md`](transit_sim/README.md) says
which direction each lane runs, which is how you work out which two lanes
belong in each pair.

The rest is yours: how long a full cycle lasts now that two pairs take
turns, and how you get from a time inside that cycle to a colour for one
given lane. You already have a single round from Task 3 — this is that idea
one level up.

Once it works, compare your cycle against `PHASES` in `drive_city.py`.

**Check:** all four lights at the junction cycle correctly, and two lanes
that cross each other are never green at the same time.

## Task 5 — Obey the light

Back to `vehicle_node_ros.cpp`, for the first subscriber you write here.
Until now your nodes have only talked; this one listens. Three functions,
all marked `TODO (Task 5)`.

**1. `set_subscribers_and_publisher()`**, below the publisher you wrote in
Task 1. Create a subscription that listens for
`transit_msgs::msg::SignalState` on `signal_topic_`, calls `on_signal` for
every message, and is stored in `signal_sub_`. Its shape is in
[`transit_starter/README.md`](transit_starter/README.md), alongside the
publisher and timer you already wrote.

**2. `on_signal()`.** This runs once for every message that arrives, and
you get every traffic light in the city, not only yours. So first check
whether the message is even about you: `msg->lane_id` is the lane that
light governs, `lane_id_` is the lane you drive on, and if they differ,
`return` and do nothing. If they match, store `msg->state` in
`light_state_`. Note the `->` instead of a `.`, because `msg` arrives as a
pointer.

**3. `must_stop_for_light()`.** Return `true` when the car has to wait,
`false` when it can drive. `tick()` does the rest: when this returns `true`
it stops the car instead of calling `advance_progress()`.

Getting this right is the whole task, and the obvious version is wrong.
Stopping whenever the light is red freezes the car wherever it happens to
be standing — usually in the middle of the junction, which is worse than
not stopping at all. A car only stops **at the stop line**, so being at the
line is a condition in its own right, alongside the colour. Work out how to
express "has reached the line but not yet crossed it" from `progress_` and
`stop_progress_`.

Two details you would otherwise find out the hard way. `light_state_` is
`-1` until the first message arrives, and `-1` is not a colour, so decide
what the car does before it has heard from any light. And the light being
yellow means the same as red for this purpose — you are deciding whether to
enter the junction, not whether the light is literally red.

Where the stop line is goes in **`config/transit_params.yaml`** as
`stop_progress`, marked `TODO (Task 5)`. Find it by trying values, the same
way you found the lane length in Task 2. If you are still stuck after a
real go, `stop_progress()` in `drive_city.py` shows the idea — though it
takes a shortcut your node cannot, since it runs in Python and can read the
sim's map directly.

**Check:** your car stops just before the junction on red and drives on
normally when the light turns green. It should not stop in the middle of
the junction, and it should not drive past the stop line.

## Task 6 — Extra: several vehicles in one node

No TODO comment, and no skeleton — this is the first task where you change
the shape of the code instead of filling in a blank. Make one node drive
several vehicles at once, each with its own `vehicle_id` and color, each
obeying the light on its own lane.

The awkward part is not the publishing. One publisher still covers all of
them, because every `VehicleState` names the vehicle it describes. The
problem is that `progress_`, `moving_`, `velocity_` and `light_state_` are
single member variables describing exactly one car, and now there are
several. Decide how to hold per-vehicle state, and how `on_signal()` routes
an incoming light to whichever vehicles care about that lane.

## Task 7 — Extra: level crossing

Also freehand. In `vehicle_node_ros.cpp`, or a copy of it, send a train
along lane 5 every 30 seconds or so. No new message type and no change to
the sim: work out from `transit_sim/README.md` what a train actually is
here before you write anything.

**Check:** the level-crossing barriers come down as your train gets close
and go back up after it has passed, without you writing any code beyond
publishing lane 5 states. If that works, you understood why
`transit_sim/README.md` insists the train is not a special entity, rather
than just copying the shape of Tasks 1 and 2.

## Task 8 — The city, running on your code

Everything so far you checked one node at a time. Now run your vehicle node
and your signal node at once — they are separate programs that have never
met, and they only agree on the lane numbers and the two topics.
`transit_starter/launch/transit_starter.launch.py` starts both and hands
them `config/transit_params.yaml`.

Watch a full cycle. The car should come up to the line, wait through red
and yellow, and pull away on green, with nothing coordinating the two
processes except the messages they publish. If it does not, the useful
question is which node is wrong — `ros2 topic echo` on both topics tells
you what each one is actually saying, rather than what you think it is.

That is the point the demo driver becomes redundant. You have not needed
`drive_city.py` since Task 0, and now nothing in the city does.

**Check:** the junction runs correctly with no demo script involved, and
you can point at which of your two nodes decides each thing that happens.

---

## Where your code goes

Not in `traffic-sim/`. Copy `transit_starter/` into
[`your_code/`](../your_code/) and rename it, leaving this folder untouched.
[`transit_starter/README.md`](transit_starter/README.md) has the steps.

Write your nodes in C++. `transit_msgs` builds bindings for Python too, and
`drive_city.py` and the sim itself are Python, so an `rclpy` node would
work — but it is the exception here, not the convention.
