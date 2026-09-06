# transit_starter

The skeleton the task list ([`../CURRICULUM.md`](../CURRICULUM.md)) is
built around: a vehicle node and a signal node, with numbered
`TODO (Task N)` comments everywhere a task wants you to add code.

The nodes are C++ and build with `ament_cmake`. Only
`launch/transit_starter.launch.py` is Python, which is normal ROS 2
practice no matter what language the nodes themselves are written in.

## How the files are laid out

Each node is split over three files, the same way the team lays out its
real robot code:

```
include/transit_starter/vehicle_node_ros.hpp   the class declaration
src/vehicle_node_ros.cpp                       the class: setup, callbacks, logic
src/vehicle_node.cpp                           only main(), which starts the node

include/transit_starter/signal_node_ros.hpp    same three files for the
src/signal_node_ros.cpp                        signal node
src/signal_node.cpp

config/transit_params.yaml                     the values both nodes read
launch/transit_starter.launch.py               starts both, with that config
```

`CMakeLists.txt` builds the two `_ros.cpp` files into one library and links
each `main()` against it. Splitting `main()` out this way means the node
class can be built once and reused — by a test, or by another program that
wants to run it — without dragging a second `main()` along with it.

## What is written for you, and what is not

You never have to write a new function, and you never have to touch a
`.hpp` file. Every function you need already exists, is already declared in
the header, and is already called from the right place. Each one is empty,
with a `TODO (Task N)` comment inside saying what goes there. You only fill
in insides of functions.

`set_parameters()` and `tick()` are written for you in both nodes. `tick()`
is what the timer calls, and it calls the empty functions in the right
order, so as you fill them in one at a time the node does a little more
each time. An empty one is harmless: it either does nothing or returns a
value meaning "carry on".

[`../CURRICULUM.md`](../CURRICULUM.md) lists which function belongs to
which task, including the TODOs that are in
[`config/transit_params.yaml`](config/transit_params.yaml) rather than in
the C++.

## The ROS 2 calls you will need

Between them, the tasks need three calls. Everything else is ordinary C++.
These are the shapes; the TODO comments say which message type and which
variable to use in each case.

**A publisher** — what type of message, on which topic, and how many
messages to hold if the receiver is slow (10 is fine everywhere here):

```cpp
some_pub_ = this->create_publisher<some_msgs::msg::SomeType>(topic_name, 10);
```

To send one, fill in a message and hand it over. A message starts out empty
and you set its fields by name, exactly like a struct:

```cpp
some_msgs::msg::SomeType message;
message.some_field = some_value;
some_pub_->publish(message);
```

**A timer** — how often to fire, and what to call each time. The
`std::bind(&ClassName::function, this)` part means "call this function, on
this object":

```cpp
timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period_in_seconds),
    std::bind(&ClassName::function_to_call, this));
```

**A subscription** — type and topic as before, plus the function to run for
each arriving message. The extra `std::placeholders::_1` is a slot that
says "the message goes here" when it is called:

```cpp
some_sub_ = this->create_subscription<some_msgs::msg::SomeType>(
    topic_name, 10,
    std::bind(&ClassName::callback_name, this, std::placeholders::_1));
```

The callback receives the message as a pointer, so read its fields with
`->` rather than `.`:

```cpp
void ClassName::callback_name(const some_msgs::msg::SomeType::SharedPtr msg) {
    if (msg->some_field == something) { ... }
}
```

You can see all of this in use already: `set_parameters()` and `tick()` are
written for you in both nodes, and are worth reading before you start.

## Parameters instead of hardcoded values

Your lane number, speed, color and phase timings live in
[`config/transit_params.yaml`](config/transit_params.yaml), not in the C++.
Change it, relaunch, and the node picks the new values up with no rebuild —
which matters a lot in Tasks 2 and 5, where you find the lane length and
the stop line by trying values until they look right.

The defaults in the C++ are only used when no config file is loaded, so
`ros2 run` without `--params-file` still works.

## Using it

This package is a template, not where your finished work should live.
Once you have built the workspace and are ready to start:

```bash
cp -r traffic-sim/transit_starter your_code/<yourname>_traffic
```

Then in your copy:

1. Rename the package: `transit_starter` → your package name in
   `package.xml` (`<name>`) and `CMakeLists.txt` (`project(...)`).
2. Rename the `include/transit_starter/` folder to match, and update the
   `#include "transit_starter/..."` lines at the top of all four `.cpp`
   files to the new path.
3. Update the package name in `launch/transit_starter.launch.py`, in both
   `Node(package=...)` entries and in `get_package_share_directory(...)`.
4. `colcon build --packages-select transit_msgs <your_package_name>`
5. Work through the TODOs in order.

Keep the file layout and the config file when you copy. They are the
conventions the team's real packages follow, so code written this way will
look familiar to whoever reviews it later.

## Running it as-is (before copying and renaming)

You can build and run the untouched skeleton to check your workspace is set
up correctly. It builds, starts, and then throws, naming the function and
the task to begin with:

```bash
colcon build --packages-select transit_msgs transit_starter
source install/setup.bash
ros2 run transit_starter vehicle_node    # -> throws, Task 1
ros2 run transit_starter signal_node     # -> throws, Task 3
```

Once you have copied it into `your_code/` and started filling things in,
you can launch both of your nodes together with:

```bash
ros2 launch <your_package_name> transit_starter.launch.py
```

That passes `config/transit_params.yaml` to both nodes. Running each node
in its own terminal with `ros2 run` works too, but then pass the config
yourself:

```bash
ros2 run <your_package_name> vehicle_node --ros-args \
  --params-file install/<your_package_name>/share/<your_package_name>/config/transit_params.yaml
```
