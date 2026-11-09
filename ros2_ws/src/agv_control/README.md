# agv_control

Shared ROS 2 behaviour for both simulation and the real AGV. This package has no
dependency on Gazebo, RViz, simulated pose, or `agv_sim`.

- `line_follower`: tape steering, junction selection, station stop/turn state machine,
  and the `~/explore` action server.
- `obstacle_stop`: stop-and-wait gate using four `sensor_msgs/Range` topics.
- `wheel_odometry`: `/wheel/odom` and the `odom → base_link` transform from the drive-wheel
  joint angles (`odometry.py`); wheel size and spacing come from `agv_description`. With
  `imu_topic` set, the heading comes from the gyro and the wheels give only the distance.
  The gyro bias is learnt whenever the wheels stand still (0.1 s without an encoder count),
  and the heading holds still then; until 1 s of still samples is in, the wheels give the
  heading. Trips and explored maps therefore use the gyro's heading.
- `tape_recorder`: logs junction, station and turn events with wheel odometry
  (`trip.py`); `~/save_trip` (`std_srvs/Trigger`) writes the trip JSON.
- `build_map` (`map_builder.py`): turns one or more trips into a junction graph.
- `route_planner.py`: shortest routes on that graph (`site_map_file`).
- `explorer.py`: chooses fork sides while exploring and builds the map on the way.
- `mission_planner.py`: L/R route planning from station codes and loop distances,
  used when no taught map is given.
- `junction.py`: fork/merge/unknown events from the MTS160 track stream.

Configuration in `config/`:

- `station_loop.json`: reference loop configuration for `mission_planner`, extracted
  from the simulated geometry (START, S3, S4, S2, S1). It is not a measured real-site map.
- `stations.json`: station name -> Navicode for `targets` on a taught map. Add a line
  when a new station tag is laid; no positions are needed.
- `site_map.json`: the taught map of the simulated layout, written by exploration.

## Gazebo use

`ros2 launch agv_sim gazebo.launch.py` starts the follower, `obstacle_stop`,
`wheel_odometry` and `tape_recorder` together with the simulated sensors; see
`../agv_sim/README.md` for its arguments.

The command path is `/cmd_vel_raw -> obstacle_stop -> /cmd_vel`. Publish manual
commands on `/cmd_vel_raw` too. `obstacle_stop` must be the sole `/cmd_vel`
publisher in this configuration. The legacy RViz launch runs the shared follower
directly against the kinematic simulator, without ultrasonic stopping as before.

The interfaces stay the same when real sensor and motor drivers replace the
simulators. On hardware use `use_sim_time: false` and measured route configuration.
The vendor MTS160 node and the simulated MTS160 must not publish the same topics
at the same time. Sensor behaviour is not part of this package.

## Exploring a layout

Launch with `explore:=true`; the follower waits at START (`IDLE`). Then:

```bash
ros2 action send_goal --feedback /line_follower/explore agv_interfaces/action/Explore \
  "{map_file: maps/site_map.json}"
```

How sides are chosen, at the first two-track message of each junction:

| Situation | Choice |
|---|---|
| Merge (second track appears ≥ 40 mm away) | keep the track nearest the centre |
| Fork with a branch not driven yet | take it, `R` first |
| Fork, both branches driven | first side of the shortest path to an undriven junction end |
| Nothing left undriven | shortest path home to START |

The robot turns around at every station tag except the START tag it drives over at
the beginning, and the goal succeeds when it is back at START with no undriven
junction ends. Feedback reports nodes found and junction ends still undriven. The
goal aborts if the tape is lost or no route to undriven tape exists; cancelling
stops the robot. The map is written relative to the launch folder and can be used
with `site_map:=` straight away.

Without exploration, record trips with hand-written `route` / `stops` and run
`ros2 run agv_control build_map trip1.json trip2.json -o site_map.json`; it lists
any junction ends that no trip drove.

## Route configuration

`station_loop.json` lists stations in cyclic counter-clockwise order. Each station
has a unique `name`, `navicode`, and `loop_position_m`; `loop_length_m` is the total
loop length. Planning assumes the existing station-spur topology: two junction
choices to enter/leave each spur, mirrored for clockwise travel.

Pass `route_map_file:=/absolute/path/to/site.json` to the follower to select a
different layout with that topology. Empty `route_map_file` loads the installed
reference configuration. `targets` produces the mission and stop-code sequence;
empty targets keep the existing explicit `route` / `stops` parameters. With
`site_map_file` set, `targets` are looked up in `stations.json` and planned on the
taught map instead.

## Deployment status

The folder separation is not hardware commissioning. The follower still estimates
spin angle from commanded speed and fixed message periods, and its track
subscription must be made compatible with the vendor driver's best-effort QoS
before connecting that driver. An ICM-20948 driver publishing `sensor_msgs/Imu`,
gyro scale calibration, odometry-based turns, calibrated sensor placement, motor
limits/watchdogs, and hardware validation remain separate work.

Build from `ros2_ws`, never from this package directory. The build target
`--packages-up-to agv_control` brings in `naviq_msgs` and `agv_interfaces` without
any simulation package.
