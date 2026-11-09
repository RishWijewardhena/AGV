# agv_description

The AGV's robot description, shared by RViz, Gazebo and the real robot: URDF (xacro), CAD meshes
and the robot's geometry.

```
agv_description/
├── config/geometry.yaml     wheel size and spacing, sensor frames: the one place they are set
├── urdf/agv.urdf.xacro      links, joints, meshes, masses, sensor frames
├── urdf/agv.gazebo.xacro    Gazebo-only parts, included with sim:=true
├── meshes/                  CAD meshes (body reduced by tools/decimate_dae.py)
├── agv_description/geometry.py   geometry.yaml as node parameters for the launch files
├── launch/display.launch.py RViz with joint sliders, to check the model
├── rviz/display.rviz
└── tools/decimate_dae.py    shrinks a Fusion COLLADA export (run with the project venv)
```

Check the model without Gazebo:

```bash
ros2 launch agv_description display.launch.py
```

## Frames

`base_link` is on the floor under the drive axle, +x forward, +y left.

```
odom ─▶ base_link            wheel_odometry (Gazebo, hardware) or sim_node (kinematic sim)
        ├── body_link        mass, collision box, body and rail meshes
        ├── left_wheel, right_wheel                       continuous, from /joint_states
        ├── <corner>_caster_mount ─▶ _swivel ─▶ _wheel    spring, swivel, wheel joints
        ├── mts160_link      (0.30, 0, 0.038)
        ├── lidar_link       (0.312, 0, 0.317)
        ├── imu_link         (0, 0, 0.10), provisional
        └── ultrasonic_<corner>_link   (±0.38, ±0.18, 0.105), rear ones face backwards
```

`robot_state_publisher` publishes everything below `base_link`. The body's mass sits on
`body_link` because KDL cannot use an inertia on the root link.

## The model

Built from the Fusion export `AGV_Design_4` (`/home/rishmika/Documents/URDF4`). The visuals are the
CAD meshes; physics uses simple shapes.

| Part | Value |
|---|---|
| Mass | 30 kg in total, as weighed (Fusion's materials give 60 kg) |
| Body | 23.9 kg: the CAD centre of mass, CAD inertia scaled to the mass; collision box 0.70 × 0.46 × 0.29 m |
| Drive wheels | 131 mm hub motors (datasheet and measured; the CAD has 130 mm), 1.95 kg each, centres 0.3744 m apart; sphere collisions, both axes +y |
| Casters | four 2-inch swivel casters, stems at x = ±0.295 m, y = ±0.175 m, wheels trailing 23 mm; sprung (±5 mm) in Gazebo |
| MTS160 | in the CAD body: housing x = 0.285–0.320 m, bottom 38 mm above the floor |
| Ultrasonic | 4 `gpu_lidar` fans in Gazebo (±30° like the URM09, 0.02–5 m, 10 Hz), 6 mm outside the CAD sensor faces |
| Lidar | RPLidar head in the CAD; 360°, 0.15–6 m, 5.5 Hz in Gazebo |
| IMU | ICM-20948, not in the CAD yet. Gazebo at 100 Hz from datasheet DS-000189: gyro noise 0.015 °/s/√Hz and a random fixed offset (~1.7 °/s typical, ±5 °/s allowed); accelerometer 230 µg/√Hz, ±25 mg offset. Temperature drift is not simulated: Gazebo's dynamic-bias model wanders about ten times faster than the datasheet |
| Drive limits (Gazebo) | 0.5 m/s, 1 rad/s |

Each caster is a chain: the stem and top plate ride on the spring, the fork swivels freely, and
the wheel rolls; sideways friction on the trailing wheel turns the fork. The linear rails are
drawn fixed to the body (the CAD has them slide). The CAD's left wheel axis is reversed and its
casters sit 2 mm below the drive wheels; the model fixes the first and raises the caster wheels
by 2 mm for the second.

`agv.gazebo.xacro` holds what URDF cannot express: sensors, the `DiffDrive`,
`JointStatePublisher` (all joints) and `PosePublisher` plugins, wheel friction, and the caster
springs (`springStiffness`/`springReference`, converted to SDF `spring_stiffness`). Gazebo
merges the fixed-joint links into `base_link` and keeps their names as frames.

## Geometry

`config/geometry.yaml` is read by the xacro and, through `geometry.node_params()`, by the
launch files, which pass it to `wheel_odometry`, `obstacle_stop`, `gazebo_mts160` and
`motor_node`. Those nodes have no geometry defaults, so a missing value fails at start-up
instead of silently using a wrong one. Change a dimension only in `geometry.yaml`.

## Meshes

The CAD body mesh was 523 MB (6.9 M triangles, 4.7 M of them in hidden bearing balls).
`tools/decimate_dae.py` merges points on a 2 mm grid, part by part so materials stay, and copies
the lidar and linear actuators unchanged: 0.47 M triangles, 36 MB. After a new Fusion export:

```bash
cd "/media/rishmika/Shared Data/Idea8/AGV"
venv/bin/pip install numpy                              # once
venv/bin/python ros2_ws/src/agv_description/tools/decimate_dae.py \
    <export>/meshes/base_link/base_link.dae ros2_ws/src/agv_description/meshes/body.dae \
    --cell 0.002 --keep 'igs|Actuator|actuator'
```

`--keep` is a regex of CAD material names copied exactly (`igs` = the RPLidar parts). The
small meshes (wheels, casters, rails) are the export's own files.

## Open points

- The centre of mass comes from the CAD, whose masses are off by a factor of two. Weighing the drive axle alone would place it.
- The caster springs and the fork damping are provisional values. On the real robot the drive wheels have shocks instead; the model has none.
- No `ros2_control` block yet; the joint names are ready for one.
- The IMU position is a placeholder until it is in the CAD; only its yaw axis matters for the heading. The gyro's ±1.5 % scale tolerance is not simulated: calibrate it on the robot (for example ten spins on the spot against the tape).
