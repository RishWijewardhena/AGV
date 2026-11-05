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
| Robot model | none (TF axes only) | `models/agv_cad.sdf` (Fusion CAD), or `models/agv_box.sdf` |
| Ultrasonic obstacle stop | no | yes |
| Mapping (recorder, exploration) | no | yes |
| Status | complete | missions and exploration verified headless |

```
 y (m)
 7.3        S1■              S2■
             │                │
 5.0    ╭────Y────────────────Y────╮      loop 10 m × 5 m, 1 m corners
        │                          │      spurs: 0.8 m arcs and a 1.5 m stem
 2.5 START■──Y                     Y──■S4  Navicode marker 1.0 m up each stem
        │                          │
 0.0    ╰────────────Y─────────────╯
                     │
-2.3                 ■S3
     x: -3.3  -1     2.5   4    6      9    11.3
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
| `site_map` | `''` | Taught map to plan `targets` on; empty uses the loop planner |
| `route`, `stops` | `[R]`, `[2, 4]` | Explicit junction sides and stop codes, used with `targets:="['']"` |
| `explore` | `false` | `true`: the follower waits for an `Explore` action goal instead of a mission |
| `trip_file` | `maps/trips/trip.json` | Where `tape_recorder` saves the trip (relative to the launch folder) |
| `robot` | `cad` | `cad` (AGV_Design_4 from Fusion) or `box` (provisional box robot) |
| `headless` | `false` | `true` runs Gazebo without its window |

The follower's output goes through the obstacle gate:

```
line_follower ─▶ /cmd_vel_raw ─▶ obstacle_stop ─▶ /cmd_vel ─▶ Gazebo DiffDrive
                                      ▲
     Gazebo scans ─▶ ultrasonic_sim ─▶ /ultrasonic/<corner>   (Range)

Gazebo true pose ─▶ /gazebo/ground_truth ─▶ gazebo_mts160 ─▶ /mts160/track, /mts160/navicode

/joint_states ─▶ wheel_odometry ─▶ /wheel/odom ─▶ tape_recorder ◀─ /junction_events (follower)
```

### Mapping the layout

The robot can learn the tape graph instead of being told it:

```bash
ros2 launch agv_sim gazebo.launch.py explore:=true
ros2 action send_goal --feedback /line_follower/explore agv_interfaces/action/Explore \
  "{map_file: maps/site_map.json}"
ros2 launch agv_sim gazebo.launch.py targets:="[S4]" site_map:=maps/site_map.json
```

Exploration of the current layout takes about 6 minutes (≈ 87 m of tape and one turn at every
station). `ros2 param set /line_follower dwell 0.0` before sending the goal removes the
3 s waits. Details are in `../agv_control/README.md`.

### Changing the tape layout

Each `SPURS` entry in `tape_map.py` is `(name, Navicode, x, y, direction)`. `spur()` only draws
the Y and stem, so `(x, y)` must lie **on** a straight part of the loop and `direction` is the
loop's direction of travel there:

| Loop side | Tape | Direction |
|---|---|---|
| bottom | y = 0, x 0 → 8 | `0` |
| right | x = 9, y 1 → 4 | `90` |
| top | y = 5, x 8 → 0 | `180` |
| left | x = −1, y 4 → 1 | `-90` |

A spur uses 0.8 m of loop either side of its point, so keep that clear of corners and other
spurs. Keep `SPURS` in counter-clockwise order. When adding a station, also:

1. add it to `agv_control/config/stations.json` (name → Navicode) for `targets`;
2. add it at the same position in `agv_control/config/station_loop.json`, with its
   `loop_position_m` from `loop_positions()`, so the loop planner and `test_route_geometry`
   stay correct, and update the leg table in `agv_control/test/test_mission_planner.py`;
3. explore again and replace `agv_control/config/site_map.json` with the result.

### How the pieces work

- **World:** at every launch, `world_builder.py` writes `/tmp/agv_station_loop.sdf` from `tape_map.py`. The Gazebo track is therefore always identical to the RViz one. Tape (25 × 1.3 mm) and Navicode markers are visual only, so the wheels never bump over tape edges.
- **Robot:** spawned at START with `ros_gz_sim create`; `robot:=box` swaps in the box robot. The launch's `ROBOTS` table passes each model's geometry to the nodes below. Add `headless:=true` to run without the Gazebo window.
- **Virtual MTS160** (`gazebo_mts160`): uses the same `read_mts160` and marker maths as `sim_node`, but takes the pose from Gazebo's physics instead of integrating `/cmd_vel`. The mounting position is the `mts160_x` parameter (default 0.27 m ahead of the axle, the CAD bracket).
- **Obstacle stop** (`agv_control/obstacle_stop`): the only `/cmd_vel` publisher.
  - It holds the robot when any of the 4 sensors reads 0.40 m or less, or when sensor data is missing or older than 0.3 s.
  - It resumes after all 4 sensors read at least 0.55 m for 0.5 s.
  - All 4 sensors count, whatever the direction of travel.
- To test the stop, drop a box in front of the robot with Gazebo's box tool. The `obstacle_stop` log shows `HOLD` and then `CLEAR`.

Gazebo's `DiffDrive` keeps the last command it received. `obstacle_stop` sends zero when commands stop, so keep it running until the robot has stopped.

### The CAD robot: `models/agv_cad.sdf`

Built from the Fusion export `AGV_Design_4` (`/home/rishmika/Documents/URDF4`). The visuals are the
CAD meshes in `meshes/cad/`; physics uses simple shapes. The model origin is the floor under the
drive axle, +x forward.

| Part | Value |
|---|---|
| Body | 0.70 × 0.46 m plate, 56 kg with the CAD inertia; collision box 0.70 × 0.46 × 0.29 m |
| Drive wheels | 130 mm diameter, centres 0.3744 m apart; sphere collisions, both axes +y |
| Casters | four 2-inch wheels at x = 0.272 / −0.318 m, y = ±0.175 m, as sprung spheres (±5 mm) |
| MTS160 | not in the CAD; placeholder at the bracket, x = 0.27 m, bottom 20 mm above the floor |
| Ultrasonic | 4 `gpu_lidar` fans (±30° like the URM09, 0.02–5 m, 10 Hz) at x = ±0.38, y = ±0.18 m, 0.105 m up |
| Lidar | x = 0.312 m, 0.317 m up (RPLidar head in the CAD) |
| Drive limits | 0.5 m/s, 1 rad/s |

Casters and the linear rails are drawn fixed to the body (the CAD has them swivel and slide). The
CAD's left wheel axis is reversed and its casters sit 2 mm below the drive wheels; the model fixes
the first, and the caster springs absorb the second.

The CAD body mesh was 523 MB (6.9 M triangles). `tools/decimate_dae.py` reduces it to about
0.23 M triangles (18 MB), part by part so materials stay. After a new Fusion export:

```bash
cd "/media/rishmika/Shared Data/Idea8/AGV"
venv/bin/pip install fast-simplification numpy          # once
venv/bin/python ros2_ws/src/agv_sim/tools/decimate_dae.py \
    <export>/meshes/base_link/base_link.dae ros2_ws/src/agv_sim/meshes/cad/body.dae
```

The box robot (`robot:=box`) keeps the earlier provisional values: 600 × 460 mm body, 131 mm
wheels 0.40 m apart, MTS160 at 0.25 m.

### Using another robot model

Add an entry to `ROBOTS` in `launch/gazebo.launch.py`. The model must provide:
1. a `DiffDrive` plugin on `/cmd_vel`, with the real wheel radius and spacing;
2. a `PosePublisher` plugin, used for `/gazebo/ground_truth`;
3. a `JointStatePublisher` for `left_wheel_joint` and `right_wheel_joint` (wheel odometry);
4. four ultrasonic sensors on `/ultrasonic/<corner>/scan` (`front_left`, `front_right`, `rear_left`, `rear_right`) and a lidar on `/scan`.

Its `params` override the nodes' CAD defaults: `wheel_diameter` and `track_width`
(`wheel_odometry`), `mts160_x` (`gazebo_mts160`), `lidar_x` and `bumper_x` (`obstacle_stop`),
and the four sensor mounts `[x, y, z, yaw]` (`ultrasonic_sim`); `lidar` is the lidar's (x, z).

## The follower (`agv_control/line_follower`)

It only sees what the real sensor would report. It is a state machine:

```
FOLLOW ──(Navicode = next stop)──▶ DWELL ──(3 s)──▶ SPIN ──(≥150° and centred)──▶ FOLLOW
                                                      └──(last stop)──▶ DONE
```

With `explore:=true` it starts in `IDLE`, every station tag after the first counts as a stop,
and it returns to `IDLE` when the `Explore` goal ends.

- **Steering:** `w = kp·offset + ka·angle`, using the left or the right track as the route says.
- **Junctions:** two tracks seen for 5 messages means a junction has started. One track for 20 messages means it is over.
- **Route planning** without `site_map` (`agv_control.mission_planner`): each leg takes the shorter way round the loop.
  - Counter-clockwise, entering or leaving a spur is `R,R` and passing one is `L,L`.
  - Clockwise, everything is mirrored.
- **Route planning** with `site_map` (`agv_control.route_planner`): shortest path on the taught junction graph.

| Parameter | Default | Meaning |
|---|---|---|
| `targets` | `['']` | Stations to visit in order, for example `[S1,S3]`. When set, it generates `route` and `stops`. |
| `site_map_file` | `''` | Taught map for `targets`; empty uses `station_loop.json` |
| `explore` | `false` | Serve the `~/explore` action instead of driving a mission |
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
| `SPURS` | START 4, S3 3, S4 5, S2 2, S1 1 | Station names and Navicodes, in counter-clockwise order |
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
├── models/agv_cad.sdf      CAD robot for Gazebo (default)
├── models/agv_box.sdf      provisional box robot (robot:=box)
├── meshes/cad/             CAD meshes for agv_cad.sdf (body decimated)
├── tools/decimate_dae.py   shrinks a Fusion COLLADA export (run with the project venv)
├── urdf/, meshes/*.STL     older CAD export (display only, not used by the launches)
├── rviz/agv_sim.rviz       RViz configuration
├── package.xml
└── setup.py
```

## Limits and next steps

- The CAD wheels are 130 mm; the measured real wheel was given as 131 mm. Check before hardware odometry.
- The CAD casters do not swivel in simulation, and the caster springs are provisional values.
- A Navicode is modelled as one 50 mm square. The real sensor decodes a pattern of several markers.
- The virtual MTS160 is geometric. It doesn't model magnetic field strength, edge effects or noise.
- The turn-around counts its angle from message timing, so an obstacle hold during a spin makes it think it has turned further than it did.
- Exploration does not finish if a junction is classified `unknown` or odometry drift splits one junction into two (> 0.3 m apart); neither happens in Gazebo.
- The lidar and docking with the trolley still need work.
