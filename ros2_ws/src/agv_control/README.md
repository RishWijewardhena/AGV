# agv_control

Shared ROS 2 behaviour for both simulation and the real AGV. This package has no
dependency on Gazebo, RViz, simulated pose, or `agv_sim`.

- `line_follower`: tape steering, junction selection, station stop/turn state machine.
- `obstacle_stop`: stop-and-wait gate using four `sensor_msgs/Range` topics.
- `mission_planner.py`: L/R route planning from station codes and loop distances.
- `config/station_loop.json`: reference START/S1/S2/S3 loop configuration, extracted
  from the existing simulated geometry. It is not a measured real-site map.

## Gazebo use

Start `ros2 launch agv_sim gazebo.launch.py` first; it supplies simulated sensors
and launches the shared obstacle-stop node. In another sourced terminal:

```bash
ros2 run agv_control line_follower --ros-args \
  -p speed:=0.1 -p use_sim_time:=true -r cmd_vel:=/cmd_vel_raw
```

The command path is `/cmd_vel_raw -> obstacle_stop -> /cmd_vel`. Publish manual
commands on `/cmd_vel_raw` too. `obstacle_stop` must be the sole `/cmd_vel`
publisher in this configuration. The legacy RViz launch runs the shared follower
directly against the kinematic simulator, without ultrasonic stopping as before.

The interfaces stay the same when real sensor and motor drivers replace the
simulators. On hardware use `use_sim_time: false` and measured route configuration.
The vendor MTS160 node and the simulated MTS160 must not publish the same topics
at the same time. Sensor behaviour is not part of this package.

## Route configuration

`station_loop.json` lists stations in cyclic counter-clockwise order. Each station
has a unique `name`, `navicode`, and `loop_position_m`; `loop_length_m` is the total
loop length. Planning assumes the existing station-spur topology: two junction
choices to enter/leave each spur, mirrored for clockwise travel.

Pass `route_map_file:=/absolute/path/to/site.json` to the follower to select a
different layout with that topology. Empty `route_map_file` loads the installed
reference configuration. `targets` produces the mission and stop-code sequence;
empty targets keep the existing explicit `route` / `stops` parameters.

## Deployment status

The folder separation is not hardware commissioning. The follower still estimates
spin angle from commanded speed and fixed message periods, and its track
subscription must be made compatible with the vendor driver's best-effort QoS
before connecting that driver. Odometry-based turns, calibrated sensor placement,
motor limits/watchdogs, and hardware validation remain separate work.

Build from `ros2_ws`, never from this package directory. The build target
`--packages-up-to agv_control` brings in `naviq_msgs` without any simulation package.
