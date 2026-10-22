# agv_sim

A lightweight ROS 2 Jazzy simulation of a magnetic-tape-guided AGV (automated guided vehicle).

The AGV leaves START, drives to one or more stations along magnetic tape, and chooses a branch at every fork and merge. It recognises each station by its Navicode marker, stops, waits, turns 180° on the spot, and finally returns to START.

The simulated guide sensor imitates the **Naviq MTS160** and publishes `naviq_msgs`.
Shared behaviour lives in the sibling `agv_control` package; this package contains
only simulation code and assets. The shared follower still needs hardware
compatibility and commissioning work; see `../agv_control/README.md`.

The original simulation is kinematic and displayed in RViz. A separate Gazebo
launch now supports physical manual driving and a virtual MTS160 over a straight
tape strip; its station loop, lidar, and trolley docking are not connected yet.

```
 y (m)
 7.3        S1■              S2■
             │                │
 5.0    ╭────Y────────────────Y────╮      loop 10 m × 5 m, 1 m corners
        │                          │      spurs: 0.8 m arcs and a 1.5 m stem
 2.5 START■──Y                     │      Navicode marker 1.0 m up each stem
        │                          │
 0.0    ╰────────────Y─────────────╯
                     │
-2.3                 ■S3
     x: -3.3  -1     2.5   4    6      9
```

## Quick start

### Gazebo: manual driving and magnetic-sensor readings

```bash
cd "/media/rishmika/Shared Data/Idea8/AGV/ros2_ws"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to agv_sim
source install/setup.bash
ros2 launch agv_sim gazebo.launch.py
```

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --rate 10 /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}"
```

Press Ctrl+C to end the publisher, then explicitly stop the robot:

```bash
ros2 topic pub --once /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

The shared obstacle-stop node is the sole `/cmd_vel` publisher. Send all manual
or follower requests to `/cmd_vel_raw`; a stale request or sensor reading causes
the gate to publish zero. To turn in place, request `linear.x: 0.0` and
`angular.z: 0.4`. Gazebo's drive plugin itself retains its last command, so keep
the gate running until the robot is stopped.

This launch runs Gazebo, the ROS bridge, simulated MTS160 and ultrasonic nodes,
and `agv_control/obstacle_stop`. Start the shared follower separately:

```bash
ros2 run agv_control line_follower --ros-args -p speed:=0.1 \
  -p use_sim_time:=true -r cmd_vel:=/cmd_vel_raw
```

It does not start the kinematic `sim_node` or the real Naviq CAN driver.
Close the lightweight simulation before using it to avoid competing `/cmd_vel`
and `/tf` publishers.

The initial model uses a 600 x 460 mm envelope, 131 mm drive wheels at the
longitudinal centre, and four corner supports aligned with the drive-wheel
centre lines. The 280 mm height is interpreted as floor-to-body-top, not body
thickness or clearance. Provisional values are 20 kg body mass, 60 mm drive-wheel
width, 400 mm wheel-centre spacing, 50 mm support diameter, and support centres
at x = +/-250 mm. Supports are low-friction spheres approximating casters, with
compliant vertical mounts: +/-5 mm travel, 2000 N/m stiffness, 20 N s/m damping,
and 2 mm spring preload. These provisional simulation parameters keep the drive
wheels loaded when a corner support crosses the raised tape. Rigid coplanar
six-point supports caused the wheels to slip during turning. The model does not
represent measured suspension, caster swivel, rolling resistance, or brackets.

From the repository root, verify physical turning (requires Gazebo networking):

```bash
python3 scripts/check_gazebo_turn.py
python3 scripts/check_gazebo_turn.py --angular-speed -0.4
```

The check starts its own isolated headless world and compares chassis world
heading with wheel odometry; wheel odometry alone does not prove actual motion.

The MTS160 is a box placeholder with its bottom face 28 mm above the floor and
250 mm ahead of the axle. Its STEP mesh is not connected yet.
Tape is a straight 4 m test strip, 25 mm wide and 1.3 mm thick, with collision
geometry. Sensor clearance above its top is therefore 26.7 mm. Straight following
works with the shared follower; the full station loop is not connected in Gazebo yet.

Inspect the sensor in a third terminal:

```bash
cd "/media/rishmika/Shared Data/Idea8/AGV/ros2_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /mts160/track --once
ros2 topic hz /mts160/track
```

At the initial pose, `tape_detected` and `single_track` should be true, both
positions approximately 0 mm and both angles approximately 0 degrees. The topic
publishes at 100 Hz of simulation time (wall-time rate depends on simulation
speed). Stop `topic hz` with Ctrl+C. Rotate manually to move the sensor bar off
the strip: it should eventually report `tape_detected: false`. No-tape position
and angle values default to zero; always check `tape_detected` first.

Gazebo publishes the model's true world pose at 100 Hz, bridged privately to
`/gazebo/ground_truth` as `tf2_msgs/TFMessage`. The sensor selects the `agv` pose,
reads tape/mount dimensions from the same SDF world file, and reuses `read_mts160`
to publish `/mts160/track`. It does not use wheel `/odom` or integrate `/cmd_vel`.
It publishes no-tape readings when ground truth is missing or older than 0.1 s
of simulation time. A static `base_link -> mts160_link` transform identifies the
bottom sensing-face centre (the Gazebo housing link origin is at its centre).

This is a geometric model, not a magnetic-field simulation. It assumes a planar
straight tape and uses a simple 10–50 mm clearance gate above the tape, rejecting
body tilt above 15 degrees. It does not model field strength, edge effects,
noise, or Navicode patterns; strength is strong whenever a track is detected.
There are no station markers or `/mts160/navicode` output in this world yet.

Run the geometry tests from the repository root:

```bash
PYTHONPATH=ros2_ws/src/agv_sim python3 -m unittest discover -s ros2_ws/src/agv_sim/test -p 'test_gazebo_sensor.py'
```

Run the isolated headless end-to-end sensor check from a sourced workspace:

```bash
python3 ../scripts/check_gazebo_sensor.py
```

### Lightweight kinematic simulation

```bash
cd "/media/rishmika/Shared Data/Idea8/AGV/ros2_ws"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to agv_sim
source install/setup.bash

ros2 launch agv_sim sim.launch.py targets:="[S1,S3]"
```

This starts the simulator, the follower and RViz with the saved configuration. In the example, the AGV visits S1, then S3, then returns to START. Ctrl+C stops everything.

## How it works

```
                 /cmd_vel
   ┌────────────────────────────────────────┐
   ▼                                        │
┌──────────┐  /mts160/track    (100 Hz) ┌───────────────┐
│ sim_node │ ─────────────────────────▶ │ line_follower │
│          │  /mts160/navicode (10 Hz)  │               │
└──────────┘ ─────────────────────────▶ └───────────────┘
   │ /tf, /tape_map, /sensor_markers
   ▼
  RViz
```

### `sim_node`: the world and the sensor

Every 10 ms it:
- moves the robot (unicycle model) according to `/cmd_vel`. If no command arrives for 0.5 s, the robot stops.
- computes what an MTS160 would see. The sensor is modelled as a 160 mm bar, 0.25 m ahead of the axle. Every crossing of that bar with a tape gives a position in mm and an angle in degrees. Up to two tracks are reported, `left` and `right`, and they are identical on a single tape.
- checks whether the sensor is over a station marker. When the sensor leaves a marker, it publishes that station's code on `/mts160/navicode`.

### `line_follower`: the brain

It only sees what the real sensor would report. It is a state machine:

```
FOLLOW ──(Navicode = next stop)──▶ DWELL ──(3 s)──▶ SPIN ──(≥150° and centred)──▶ FOLLOW
                                                      └──(last stop)──▶ DONE
```

- **Steering:** `w = kp·offset + ka·angle`, using the left or the right track as the route says.
- **Junctions:** two tracks seen for 5 messages means a junction has started. One track for 20 messages means it is over, and the follower moves to the next route entry.
- **Route planning** (`agv_control.mission_planner`): each leg between stations takes the shorter way round the configured loop.
  - Counter-clockwise, the spurs are on the robot's right: entering or leaving a spur is `R,R`, and passing one is `L,L`.
  - Clockwise, everything is mirrored.

## Topics

| Topic | Type | Publisher |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `line_follower` (or teleop) |
| `/mts160/track` | `naviq_msgs/TrackDetection` | `sim_node` |
| `/mts160/navicode` | `naviq_msgs/Navicode` | `sim_node` |
| `/tf` (`odom → base_link`) | `tf2_msgs/TFMessage` | `sim_node` |
| `/tape_map` | `visualization_msgs/MarkerArray` (transient local) | `sim_node` |
| `/sensor_markers` | `visualization_msgs/Marker` | `sim_node` |

`/mts160/track` uses the sensor's own sign convention: **left is negative and right is positive**, in mm. ROS uses +left, so the follower flips the sign.

## Parameters (`line_follower`)

| Parameter | Default | Meaning |
|---|---|---|
| `targets` | `['']` | Stations to visit in order, for example `[S1,S3]`. When set, it generates `route` and `stops`. |
| `route` | `['R']` | `L` or `R` for each junction, used when `targets` is empty |
| `stops` | `[2, 4]` | Navicode values to stop at, in order |
| `speed` | `0.3` | Forward speed (m/s) |
| `kp` | `8.0` | Steering gain on the tape offset (rad/s per m) |
| `ka` | `1.0` | Steering gain on the tape angle (rad/s per rad) |
| `dwell` | `3.0` | Waiting time at a stop (s) |
| `spin_speed` | `0.6` | Turning speed when turning around (rad/s) |

Launch argument: `targets` (default `[S2]`).

## Track and sensor constants (`tape_map.py`)

| Constant | Value | Meaning |
|---|---|---|
| `TAPE_WIDTH` | 0.025 m | Naviq MTRK25 tape (0.05 for MTRK50) |
| `MARKER_SIZE`, `MARKER_GAP` | 0.05 m, 0.025 m | Marker size, and gap from the tape edge |
| `R_SPUR`, `STEM`, `TAG_AT` | 0.8, 1.5, 1.0 m | Junction arc radius, stem length, marker position on the stem |
| `SPURS` | START 4, S3 3, S2 2, S1 1 | Station names and Navicodes, in counter-clockwise order |
| `START_POSE` | (−3.25, 2.5, 0) | Robot start on the START stem |

`tape_map.py` contains simulated geometry only. The shared planner reads station
configuration from `agv_control`; after building and sourcing the workspace:

```bash
python3 -c "from agv_control.mission_planner import plan_mission; print(plan_mission(['S1','S3']))"
```

## Running nodes by hand

```bash
ros2 run agv_sim sim_node
ros2 run agv_control line_follower --ros-args -p targets:="[S2]"
rviz2 -d src/agv_sim/rviz/agv_sim.rviz
```

To drive by hand instead of running the follower (do not run both, since both publish `/cmd_vel`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Useful checks:

```bash
ros2 topic echo /mts160/track --once
ros2 topic echo /mts160/navicode
ros2 param get /line_follower route
```

## Files

```
agv_sim/
├── agv_sim/
│   ├── sim_node.py        simulator: motion, TF, virtual MTS160, Navicode, RViz markers
│   ├── gazebo_sensor.py   virtual MTS160 from physical Gazebo pose
│   ├── gazebo_sensor_geometry.py  straight-strip sensor maths
│   ├── ultrasonic_sim.py  scans to simulated ultrasonic Range readings
│   └── tape_map.py        track geometry and virtual sensor maths (no ROS)
├── launch/sim.launch.py   starts sim_node, agv_control follower and RViz
├── launch/gazebo.launch.py  Gazebo, simulated sensors, agv_control obstacle stop
├── worlds/               Gazebo worlds and robot/sensor visuals
├── rviz/agv_sim.rviz      RViz configuration
├── package.xml            depends on naviq_msgs, among others
└── setup.py               simulation executables only
```

`naviq_msgs` comes from Naviq's repository, cloned into `ros2_ws/src/naviq_mts160_ros2`.

## Limits and next steps

- The robot dimensions (sensor 0.25 m ahead of the axle) are placeholders until the imported CAD URDF is checked against the real chassis. The real wheels are 131 mm in diameter.
- `urdf/AGV.urdf` and `meshes/` can be displayed separately; `gazebo.launch.py` still uses the simplified robot in `worlds/agv_world.sdf`. The URDF and meshes are not installed by `setup.py` yet.
- A Navicode is modelled as one 50 mm square. The real sensor decodes a pattern of several markers.
- The kinematic model has no slip, inertia or motor limits. Gazebo adds physics,
  but real masses, caster behaviour, lidar, and docking still need work.
