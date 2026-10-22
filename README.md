# AGV: magnetic-tape-guided trolley tug

An automated guided vehicle (AGV) that follows magnetic floor tape, drives into a trolley, locks onto it, tows it to another station and releases it.

The software targets **ROS 2 Jazzy** on **Ubuntu 24.04**, with a **Raspberry Pi 4B** on the robot. Shared control is separate from the kinematic/Gazebo simulators. A small motor wrapper in `agv_bringup` is available for commissioning; the I2C ultrasonic driver is not implemented yet.

## How it works

1. **Guidance.** A Naviq MTS160 magnetic sensor under the front of the robot measures where the tape is (in mm, at 200 Hz). A steering controller keeps the robot centred on the tape.
2. **Forks.** The sensor reports a left track and a right track. At a fork they separate, and the robot follows whichever one its route requires.
3. **Stations.** Coded "Navicode" markers next to the tape tell the robot which station it has reached.
4. **Coupling.** Four solenoid door locks, switched in two pairs by relays, hold the trolley while it is towed.
5. **Safety.** A lidar and four ultrasonic sensors detect obstacles. An emergency-stop button, a buzzer and LED status strips complete the safety system.
6. **Odometry.** The robot's position along its path comes from the wheel encoders, read from the motor driver.

## Repository layout

```
AGV/
├── README.md            this file
├── CLAUDE.md            detailed hardware and design notes (bus topology, sensor protocol, register maps)
├── docs/                datasheets, manuals and the bill of materials
├── 3D design/           SolidWorks assembly and STEP export of the chassis
└── ros2_ws/             ROS 2 workspace
    ├── agv.repos        third-party ROS packages, pinned to exact commits (see below)
    └── src/
        ├── agv_control/         shared follower, obstacle stop, mission planner and route config
        ├── agv_sim/             simulated sensors, kinematic/Gazebo worlds and simulation launch
        ├── agv_bringup/         hardware-startup scaffold (not a complete robot stack)
        └── naviq_mts160_ros2/   Naviq's MTS160 driver and naviq_msgs (fetched, not committed)
```

`ros2_ws/build/`, `install/` and `log/` are colcon build output and are not committed.

## Hardware

| Part | Model | Connection to the Pi |
|---|---|---|
| Drive motors | 2× ZLLG50ASM200 hub motors, 131 mm wheels, differential drive | — |
| Motor driver | ZLAC8015D V4 (dual channel) | RS485 (Modbus RTU, 115200 8N1) through a Waveshare USB-RS485 converter |
| Magnetic guide sensor | Naviq MTS160 (160 mm wide) | CAN (500 kbit/s) through a CANable USB-CAN adapter, as SocketCAN `can0` |
| Magnetic tape | Naviq MTRK, 25 or 50 mm, north-up | — |
| Lidar | RPLidar A1M8 | USB |
| Ultrasonic sensors | 4× DFRobot I2C ultrasonic (product 1832) | I2C |
| Trolley locks | 4× solenoid door locks via a 4-channel 5 V relay module | GPIO |
| Other | NeoPixel LED strips, 24 V buzzer, E-stop, local display | GPIO |
| Power | 24 V main supply; 24 V → 5 V buck converter for the Pi | — |

Datasheets and manuals for all of these are in [docs/](docs/). Pinouts, protocol details and register addresses are summarised in [CLAUDE.md](CLAUDE.md).

## Software

### `agv_control`: shared behaviour

The follower, obstacle-stop controller, and mission planner are shared between simulation and hardware. Route planning reads `agv_control/config/station_loop.json` rather than importing simulated tape geometry. See [agv_control/README.md](ros2_ws/src/agv_control/README.md) for interfaces and remaining hardware-compatibility work.

### `agv_sim`: simulation

`sim_node` provides the lightweight RViz simulation. Gazebo supplies physical motion with a virtual MTS160 and four ultrasonic approximations. Simulation launches start the appropriate shared nodes from `agv_control`. See [agv_sim/README.md](ros2_ws/src/agv_sim/README.md).

### `agv_bringup`: real-robot startup scaffold

Holds `robot.launch.py`, commissioning configuration, the copied `ZLAC8015D.py` library, and its small `/cmd_vel` wrapper. It starts only the obstacle-stop node by default; motors and the follower are disabled. Hardware commissioning is still required. See [agv_bringup/README.md](ros2_ws/src/agv_bringup/README.md).

### `naviq_mts160_ros2`: sensor driver (third party)

Naviq's official ROS 2 driver for the MTS160 ([GitHub](https://github.com/naviq-ch/naviq_mts160_ros2), Apache-2.0). It talks CANopen to the sensor and publishes `/mts160/track`, `/mts160/markers` and `/mts160/navicode`. It also provides the `naviq_msgs` message package that `agv_sim` depends on.

We use it unmodified. It is **not** copied into this repository. Instead, [ros2_ws/agv.repos](ros2_ws/agv.repos) records its URL and the exact commit we tested, and `vcs` (vcstool) downloads it. This keeps our history free of third-party code and makes the version we depend on explicit.

To update it to a newer upstream version, test the new commit, then change `version:` in `agv.repos`.

## Getting started

### 1. Install ROS 2 Jazzy and tools

Follow the [ROS 2 Jazzy install guide](https://docs.ros.org/en/jazzy/Installation.html), then:

```bash
sudo apt install python3-vcstool python3-colcon-common-extensions python3-rosdep
```

### 2. Clone and fetch dependencies

```bash
git clone <this-repo-url> AGV
cd AGV/ros2_ws
vcs import src < agv.repos                                  # fetches naviq_mts160_ros2
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y             # installs python-can etc.
```

### 3. Build

Always build from `ros2_ws`, never from inside `src`. Quote the path if it contains spaces.

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to agv_sim
source install/setup.bash
```

- `--packages-up-to agv_sim` builds simulation and its dependencies, including `agv_control` and `naviq_msgs`.
- `--packages-up-to agv_bringup` builds the hardware-startup scaffold and shared control without `agv_sim`.
- With `--symlink-install`, edits to Python files take effect when you restart the node. A change to `setup.py`, such as adding a new node, still needs a rebuild.

### 4. Run the simulation

With the workspace sourced, run either simulation:

```bash
ros2 launch agv_sim sim.launch.py targets:="[S1,S3]"
# Or, in a separate run:
ros2 launch agv_sim gazebo.launch.py
```

The Gazebo launch includes obstacle stopping. Start its shared follower in another sourced terminal:

```bash
ros2 run agv_control line_follower --ros-args -p speed:=0.1 \
  -p use_sim_time:=true -r cmd_vel:=/cmd_vel_raw
```

The Gazebo world currently has a straight tape strip; station missions still use the kinematic simulation.

## Setting up the real MTS160 sensor

The Naviq driver expects settings that differ from the sensor's factory defaults. Do this once, over the sensor's USB-C port, before connecting it over CAN:

1. With the sensor away from any tape, run `!ZERO` (zero-level calibration). The LED stays purple until this is done.
2. Use `!CNCF` to set CAN to **500 kbit/s**, **node ID 10**, **auto-run** on and **TPDO1** enabled.
3. Run `!SAVE`.

Then bring up the CAN link on the Pi and check that frames arrive:

```bash
sudo apt install can-utils
sudo ip link set can0 up type can bitrate 500000
candump can0
```

The bus needs 120 Ω termination at both ends. Check the CANable's termination jumper and whether the sensor terminates its own end.

If the CANable runs slcan firmware rather than candleLight, it appears as `/dev/ttyACM0`. In that case create `can0` first with `sudo slcand -o -s6 /dev/ttyACM0 can0`.

## Conventions

- **Sensor sign.** The MTS160 reports positions with left negative and right positive. ROS uses positive to the left, so flip the sign when turning a sensor position into a steering error.
- **One owner per port.** Each serial or CAN device is opened by exactly one node: one node for the motor driver and one for the guide sensor.
- **Stable device names.** USB serial adapters can change numbers (`/dev/ttyUSB0` ↔ `ttyUSB1`) between boots. Use udev rules to give them fixed names such as `/dev/zlac`.
- **Real dimensions only.** Wheel radius, gear ratio and track width for odometry must come from the motor datasheet and the chassis, not from library defaults. The ZLAC8015D Python library's defaults are for a different wheel.

## Status and next steps

- [x] Kinematic tape-following simulation with forks
- [x] Naviq driver pinned with vcstool
- [x] Simulated sensor publishes `naviq_msgs` like the real MTS160
- [x] Station markers, routes and missions in the kinematic simulation
- [x] Separate shared control, simulation, and hardware-startup packages
- [ ] Station missions in Gazebo (currently a straight-strip world)
- [ ] Bring up the MTS160 over CAN on the Pi
- [x] Small ZLAC8015D motor wrapper in `agv_bringup`, offline tested and disabled by default
- [ ] Motor hardware commissioning and encoder odometry
- [x] Import the CAD URDF and STL meshes into `agv_sim`
- [ ] Measure wheel track and validate the imported URDF for Gazebo physics
- [ ] Trolley lock control, safety sensors and status LEDs
