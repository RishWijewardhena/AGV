# agv_sim

A lightweight ROS 2 Jazzy simulation of a magnetic-tape-guided AGV (automated guided vehicle).

The AGV drives from a start point to one of three stations along magnetic tape, chooses a branch at each fork, recognises stations by Navicode markers, turns around at the station, and returns to the start. The simulated guide sensor imitates the **Naviq MTS160**, the sensor used on the real robot, and publishes the same `naviq_msgs` messages as Naviq's real driver. Code written against the simulation therefore runs unchanged on the hardware.

There is no physics engine. The simulation is purely kinematic and is displayed in RViz. Gazebo is planned for later work on the lidar and on docking with the trolley.

```
        S1            S2
        │             │
   ╭────Y─────────────Y────╮      loop: 10 m × 5 m, driven counter-clockwise
START─Y                    │      spurs: 0.8 m junction arcs and a 1.5 m stem
   ╰──────────Y────────────╯      each station has a Navicode tag on its stem
              │
              S3
```

## Status

The package is being rebuilt in steps. "Planned" items are not in the code yet.

| Step | What it does | Status |
|---|---|---|
| 1 | Add `naviq_msgs` to the workspace | Done |
| 2 | `Path` builder and the station loop track | In progress |
| 3 | Virtual MTS160 publishing `/mts160/track` | Planned (the sensor is still the old 16-cell model on `/tape_cells`) |
| 4 | Navicode station tags on `/mts160/navicode` | Planned |
| 5 | Follower v2: left/right track choice and junction counter | Planned (the follower still reads `/tape_cells`) |
| 6 | Station behaviour: stop, wait, spin 180° | Planned |
| 7 | Missions: `target:=S1` / `S2` / `S3` | Planned |
| 8 | Launch file, RViz config, cleanup | Planned |

## Nodes

### `sim_node`: the simulator
- Integrates a unicycle (differential-drive) model at 100 Hz from `/cmd_vel`. If no command arrives for 0.5 s, the robot stops.
- Broadcasts the TF transform `odom → base_link`.
- Publishes the tape and the station markers on `/tape_map` (`visualization_msgs/MarkerArray`, transient-local QoS, so RViz receives it even when started later).
- Simulates the guide sensor, mounted 0.25 m ahead of the wheel axle.

### `line_follower`: steering and mission logic
- Reads the guide sensor and publishes `/cmd_vel`, using PD steering on the lateral tape offset.
- At a fork it takes the side given by the route.

## Topics

| Topic | Type | Published by |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `line_follower` (or teleop) |
| `/tf` (`odom → base_link`) | `tf2_msgs/TFMessage` | `sim_node` |
| `/tape_map` | `visualization_msgs/MarkerArray` | `sim_node` |
| `/sensor_markers` | `visualization_msgs/Marker` | `sim_node` |
| `/tape_cells` (to be removed in step 3) | `std_msgs/UInt16` | `sim_node` |
| `/mts160/track` (step 3) | `naviq_msgs/TrackDetection` | `sim_node` |
| `/mts160/navicode` (step 4) | `naviq_msgs/Navicode` | `sim_node` |

`/mts160/track` follows the sensor's own sign convention: positions are in mm, with **left negative and right positive**. ROS uses +left, so the follower flips the sign.

## Parameters

`line_follower`:

| Parameter | Default | Meaning |
|---|---|---|
| `choice` | `center` | `left`, `right` or `center` side at a fork (replaced by a route in step 5) |
| `speed` | `0.3` | Forward speed (m/s) |
| `kp` | `8.0` | Proportional steering gain (rad/s per m) |
| `kd` | `0.1` | Derivative steering gain |

## Build

From the workspace root (`ros2_ws`), never from inside `src`:

```bash
cd "/media/rishmika/Shared Data/Idea8/AGV/ros2_ws"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to agv_sim
source install/setup.bash
```

- `--packages-up-to` also builds `naviq_msgs`, which this package depends on.
- With `--symlink-install`, edits to the Python files take effect when you restart the node.
- A change to `setup.py` (for example a new node) needs a rebuild.

`naviq_msgs` comes from Naviq's repository, cloned into `src/`:

```bash
cd src && git clone https://github.com/naviq-ch/naviq_mts160_ros2.git
```

## Run

Run each command in its own terminal, with the workspace sourced:

```bash
ros2 run agv_sim sim_node
ros2 run agv_sim line_follower --ros-args -p choice:=left
rviz2
```

In RViz:
- set **Fixed Frame** to `odom`;
- add **TF**;
- add **MarkerArray** on `/tape_map`, with **Durability** set to `Transient Local`;
- add **Marker** on `/sensor_markers`.

To drive by hand instead of running the follower:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Run either the follower or teleop, not both, since both publish `/cmd_vel`.

## Useful checks

```bash
ros2 node list                                  # /sim, /line_follower
ros2 topic echo /cmd_vel
ros2 param get /line_follower choice
ros2 interface show naviq_msgs/msg/TrackDetection
ros2 run tf2_ros tf2_echo odom base_link
```

## Files

```
agv_sim/
├── agv_sim/
│   ├── sim_node.py        simulator: motion, TF, tape markers, guide sensor
│   ├── line_follower.py   steering and (later) route and station logic
│   └── tape_map.py        track geometry (no ROS): Path builder, station_loop, segment helpers
├── package.xml            depends on rclpy, geometry_msgs, visualization_msgs, tf2_ros, naviq_msgs, …
└── setup.py               console_scripts: sim_node, line_follower
```

## Robot assumptions

- Sensor mounted 0.25 m ahead of the wheel axle.
- Tape 25–30 mm wide.
- Forward speed 0.3 m/s.

These are placeholders until the robot's URDF gives the real dimensions. The real wheels are 131 mm in diameter. Hardware details are in the project's top-level `CLAUDE.md` and in `docs/`.
