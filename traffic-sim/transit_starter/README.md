# transit_starter

The skeleton this repo's curriculum ([`../CURRICULUM.md`](../CURRICULUM.md))
is built around: two nodes, `vehicle_node_ros.cpp`/`.hpp` and
`signal_node_ros.cpp`/`.hpp`, with the ROS 2 boilerplate (node, publisher,
subscription, timer) already wired up and numbered `TODO (Task N)` comments
where each task wants you to add logic. C++ and `ament_cmake`, same as the
rest of this repo's tutorial packages (`talker`, `listener`,
`simple_publisher`) — only `launch/transit_starter.launch.py` is Python,
which is normal ROS 2 practice regardless of what language the nodes are
written in. Start reading at `tick()` in whichever `.cpp` file matches the
task you're on.

## Using it

This package is a template, not where your finished work should live.
Once you've built the workspace and are ready to start:

```bash
cp -r traffic-sim/transit_starter your_code/<yourname>_traffic
```

Then in your copy:
1. Rename the package: `transit_starter` → your package name in
   `package.xml` (`<name>`) and `CMakeLists.txt` (`project(...)`).
2. Rename the `include/transit_starter/` folder to match, and update the
   `#include "transit_starter/..."` lines at the top of both `.cpp` files
   to the new path.
3. `colcon build --packages-select transit_msgs <your_package_name>`
4. Work through the TODOs in order.

## Running it as-is (before copying/renaming)

You can build and run the unmodified skeleton directly to confirm your
workspace is set up correctly — it will build, run, and throw on the first
tick, pointing you at Task 1 or Task 3:

```bash
colcon build --packages-select transit_msgs transit_starter
source install/setup.bash
ros2 run transit_starter vehicle_node    # -> throws, Task 1
ros2 run transit_starter signal_node     # -> throws, Task 3
```

Once you've copied it into `your_code/` and started filling things in, you
can launch both of your nodes together with:

```bash
ros2 launch <your_package_name> transit_starter.launch.py
```

(rename that launch file too, or just keep `ros2 run`-ing each node in its
own terminal — either works.)
