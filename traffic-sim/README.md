# Introduction

```
Learning period for new Control and Autonomy recruits. Each member builds one ROS 2 node a traffic light, a vehicle, a junction. Together they form a small city that moves. Start with the basics of ROS 2, then graduate to real controllers and state machines.
```

# control-autonomy-learning-period

> ⚠️ **These instructions are for the development and preparation period only.**
> A new, detailed README for new members will come later.

## Running the simulation (dev period)

### 1. Clone the repo into your `src` folder
```bash
git clone https://github.com/vortexntnu/software-learning-period.git
```

### 2. Build the packages
```bash
colcon build --packages-select transit_msgs transit_sim
```

### 3. Open Foxglove and import the city layout
Import `transit_sim/config/transit_city.json`, then connect to `ws://localhost:8765`.

### 4. Run the two terminals

**Terminal 1 — start the city:**
```bash
source install/setup.bash && \
  ros2 launch transit_sim transit_sim.launch.py
```

**Terminal 2 — bring it to life (demo driver):**
```bash
source install/setup.bash && \
  python3 src/software-learning-period/traffic-sim/transit_sim/scripts/drive_city.py
```

You should now see the city in Foxglove: cars moving along their lanes, the four junction lights cycling, cars holding at red, and a train crossing every 30 s (first at 8 s).

> `scripts/drive_city.py` is a throwaway demo publisher, not a real node — delete it once real control/auto nodes exist.

## What to build next

Once the city is running, see [`CURRICULUM.md`](CURRICULUM.md) for the task
list: the order to build your vehicle/traffic-light nodes in, and how to
check each step without needing anyone else's node yet.
