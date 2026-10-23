# agv_sim

Simulation of a magnetic-tape-guided AGV (automated guided vehicle) for ROS 2 Jazzy.

The AGV leaves START, drives to one or more stations along magnetic tape, and chooses a branch at every fork and merge. It recognises each station by its Navicode marker, stops, waits, turns 180° on the spot, and finally returns to START.

The simulated guide sensor imitates the **Naviq MTS160** and publishes the same `naviq_msgs` messages as Naviq's real driver.

The steering, mission and obstacle logic lives in the sibling package `agv_control`, and the same nodes will run on the real robot. This package contains only simulation code and assets. The shared follower still needs hardware commissioning; see `../agv_control/README.md`.

There are two simulations of the same track:

| | RViz (kinematic) | Gazebo (physics) |
|---|---|---|
| Launch | `sim.launch.py` | `gazebo.launch.py` |
| Robot motion | ideal unicycle model in `sim_node` | Gazebo physics, `DiffDrive` plugin |
| Robot model | none (TF axes only) | `models/agv_box.sdf`, or any SDF/URDF |
| Ultrasonic obstacle stop | no | yes |
| Status | complete | new; full missions still being verified |

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

## Build

From the workspace root, never from inside `src`:

```bash
cd "/media/rishmika/Shared Data/Idea8/AGV/ros2_ws"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to agv_sim
source install/setup.bash
```

This also builds `agv_control` and `naviq_msgs`, which comes from Naviq's repository cloned into `ros2_ws/src/naviq_mts160_ros2`. A change to `setup.py` needs a rebuild, even with `--symlink-install`.

## RViz simulation

```bash
ros2 launch agv_sim sim.launch.py targets:="[S1,S3]"
```

This starts `sim_node`, the follower and RViz. In this example, the AGV visits S1, then S3, then returns to START. Ctrl+C stops everything.

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

Every 10 ms, `sim_node`:
- moves the robot according to `/cmd_vel`. If no command arrives for 0.5 s, the robot stops.
- computes what an MTS160 would see. The sensor is a 160 mm bar, 0.25 m ahead of the axle. Each tape crossing gives a position (mm) and an angle (degrees). Up to two tracks are reported, `left` and `right`, and they are identical on a single tape.
- publishes a station's code on `/mts160/navicode` when the sensor leaves that station's marker.

## Gazebo simulation

One command starts Gazebo with the station loop and the robot at START, the bridge, the virtual MTS160, the ultrasonic sensors, the obstacle stop and the follower:

```bash
ros2 launch agv_sim gazebo.launch.py targets:="[S1,S3]"
```

| Launch argument | Default | Meaning |
|---|---|---|
| `targets` | `[S2]` | Stations to visit in order |
| `robot_file` | `models/agv_box.sdf` | Robot model to spawn (SDF or URDF) |
| `headless` | `false` | `true` runs Gazebo without its window |

The follower's output goes through the obstacle gate:

```
line_follower ─▶ /cmd_vel_raw ─▶ obstacle_stop ─▶ /cmd_vel ─▶ Gazebo DiffDrive
                                      ▲
     Gazebo scans ─▶ ultrasonic_sim ─▶ /ultrasonic/<corner>   (Range)

Gazebo true pose ─▶ /gazebo/ground_truth ─▶ gazebo_mts160 ─▶ /mts160/track, /mts160/navicode
```

### How the pieces work

- **World:** at every launch, `world_builder.py` writes `/tmp/agv_station_loop.sdf` from `tape_map.py`. The Gazebo track is therefore always identical to the RViz one. Tape (25 × 1.3 mm) and Navicode markers are visual only, so the wheels never bump over tape edges.
- **Robot:** spawned at START with `ros_gz_sim create`. Choose a different model with `robot_file:=/path/to/model.sdf` (or `.urdf`). Add `headless:=true` to run without the Gazebo window.
- **Virtual MTS160** (`gazebo_mts160`): uses the same `read_mts160` and marker maths as `sim_node`, but takes the pose from Gazebo's physics instead of integrating `/cmd_vel`. The mounting position is the `mts160_x` parameter (default 0.25 m ahead of the axle).
- **Obstacle stop** (`agv_control/obstacle_stop`): the only `/cmd_vel` publisher.
  - It holds the robot when any of the 4 sensors reads 0.40 m or less, or when sensor data is missing or older than 0.3 s.
  - It resumes after all 4 sensors read at least 0.55 m for 0.5 s.
  - All 4 sensors count, whatever the direction of travel.
- To test the stop, drop a box in front of the robot with Gazebo's box tool. The `obstacle_stop` log shows `HOLD` and then `CLEAR`.

Gazebo's `DiffDrive` keeps the last command it received. `obstacle_stop` sends zero when commands stop, so keep it running until the robot has stopped.

### The box robot: `models/agv_box.sdf`

Provisional values, from the planned dimensions:

| Part | Value |
|---|---|
| Body | 600 × 460 mm, top 280 mm above the floor, 20 kg |
| Drive wheels | 131 mm diameter, 60 mm wide, 0.4 m apart, at the longitudinal centre |
| Casters | four 50 mm low-friction spheres at x = ±0.25 m, on sprung mounts (±5 mm) |
| MTS160 | 160 mm box placeholder, 0.25 m ahead of the axle, bottom 28 mm above the floor |
| Ultrasonic | 4 narrow `gpu_lidar` fans (±15°, 0.02–5 m, 10 Hz) at the body corners, 0.18 m up |
| Drive limits | 0.5 m/s, 1 rad/s |

### Using another robot model

A model passed with `robot_file:=` must provide, for example through `<gazebo>` tags in a URDF:
1. a `DiffDrive` plugin on `/cmd_vel`, with the real wheel radius and spacing;
2. a `PosePublisher` plugin, used for `/gazebo/ground_truth`;
3. four ultrasonic sensors on `/ultrasonic/<corner>/scan` (`front_left`, `front_right`, `rear_left`, `rear_right`).

Then set `mts160_x` on `gazebo_mts160` to the sensor's real position.

`ultrasonic_sim.py` still hard-codes the box robot's sensor positions for its TF frames.

## The follower (`agv_control/line_follower`)

It only sees what the real sensor would report. It is a state machine:

```
FOLLOW ──(Navicode = next stop)──▶ DWELL ──(3 s)──▶ SPIN ──(≥150° and centred)──▶ FOLLOW
                                                      └──(last stop)──▶ DONE
```

- **Steering:** `w = kp·offset + ka·angle`, using the left or the right track as the route says.
- **Junctions:** two tracks seen for 5 messages means a junction has started. One track for 20 messages means it is over.
- **Route planning** (`agv_control.mission_planner`): each leg takes the shorter way round the loop.
  - Counter-clockwise, entering or leaving a spur is `R,R` and passing one is `L,L`.
  - Clockwise, everything is mirrored.

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
| `use_sim_time` | `false` | Set `true` in Gazebo |

## Topics

| Topic | Type | RViz publisher | Gazebo publisher |
|---|---|---|---|
| `/cmd_vel_raw` | `geometry_msgs/Twist` | – | `line_follower` (remapped) or teleop |
| `/cmd_vel` | `geometry_msgs/Twist` | `line_follower` | `obstacle_stop` |
| `/mts160/track` | `naviq_msgs/TrackDetection` | `sim_node` | `gazebo_mts160` |
| `/mts160/navicode` | `naviq_msgs/Navicode` | `sim_node` | `gazebo_mts160` |
| `/ultrasonic/<corner>` | `sensor_msgs/Range` | – | `ultrasonic_sim` |
| `/gazebo/ground_truth` | `tf2_msgs/TFMessage` | – | Gazebo `PosePublisher` via the bridge |
| `/odom`, `/tf` | `nav_msgs/Odometry`, `tf2_msgs/TFMessage` | `sim_node` (TF only) | Gazebo `DiffDrive` via the bridge |
| `/tape_map`, `/sensor_markers` | `visualization_msgs` | `sim_node` | – |

`/mts160/track` uses the sensor's own sign convention: **left is negative and right is positive**, in mm. ROS uses +left, so the follower flips the sign.

## Track and sensor constants (`tape_map.py`)

| Constant | Value | Meaning |
|---|---|---|
| `TAPE_WIDTH` | 0.025 m | Naviq MTRK25 tape (0.05 for MTRK50) |
| `MARKER_SIZE`, `MARKER_GAP` | 0.05 m, 0.025 m | Marker size, and gap from the tape edge |
| `R_SPUR`, `STEM`, `TAG_AT` | 0.8, 1.5, 1.0 m | Junction arc radius, stem length, marker position on the stem |
| `SPURS` | START 4, S3 3, S2 2, S1 1 | Station names and Navicodes, in counter-clockwise order |
| `START_POSE` | (−3.25, 2.5, 0) | Robot start on the START stem |

`tape_map.py` holds the simulated geometry only. The planner reads its station configuration from `agv_control/config/station_loop.json`:

```bash
python3 -c "from agv_control.mission_planner import plan_mission; print(plan_mission(['S1','S3']))"
```

## Useful commands

```bash
ros2 topic echo /mts160/track --once
ros2 topic echo /mts160/navicode
ros2 param get /line_follower route
ros2 run teleop_twist_keyboard teleop_twist_keyboard                       # RViz sim
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_raw   # Gazebo
```

Run only one `/cmd_vel` source at a time: either the follower or teleop.

In Gazebo, `obstacle_stop` drops commands older than 0.3 s, so hold the teleop key down. A single key press only moves the robot briefly.

## Files

```
agv_sim/
├── agv_sim/
│   ├── tape_map.py         track geometry, MTS160 and marker maths (no ROS)
│   ├── sim_node.py         RViz simulator: motion, TF, virtual MTS160, Navicode, markers
│   ├── world_builder.py    writes the Gazebo world from tape_map.py (no ROS)
│   ├── gazebo_mts160.py    virtual MTS160 and Navicode from Gazebo's true pose
│   └── ultrasonic_sim.py   Gazebo scans to ultrasonic Range readings
├── launch/
│   ├── sim.launch.py       RViz simulation
│   └── gazebo.launch.py   Gazebo station loop, robot spawn, sensors, obstacle stop
├── models/agv_box.sdf      box robot for Gazebo
├── urdf/, meshes/          CAD export (display only; not yet drivable in Gazebo)
├── rviz/agv_sim.rviz       RViz configuration
├── package.xml
└── setup.py
```

## Limits and next steps

- Robot dimensions are placeholders until the new URDF is checked against the real chassis. The real wheels are 131 mm in diameter.
- The current `urdf/AGV.urdf` can't drive in Gazebo yet. It has no Gazebo plugins, its suspension joints are free, its left wheel axis is mirrored, it has no MTS160, and its collision meshes are heavy.
- A Navicode is modelled as one 50 mm square. The real sensor decodes a pattern of several markers.
- The virtual MTS160 is geometric. It doesn't model magnetic field strength, edge effects or noise.
- The turn-around counts its angle from message timing, so an obstacle hold during a spin makes it think it has turned further than it did.
- Full Gazebo missions, the lidar, and docking with the trolley still need work.
