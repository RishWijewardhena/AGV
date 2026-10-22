# agv_bringup

Real-robot launch/configuration and a small ZLAC8015D motor wrapper. There are
still only three project packages: `agv_sim`, `agv_control`, and `agv_bringup`.
This package does not depend on Gazebo or RViz.

## Motor files

- `agv_bringup/ZLAC8015D.py`: copied from
  `test/ZLAC8015D_python/zlac8015d/ZLAC8015D.py`; the bench-test source is unchanged.
- `agv_bringup/motor_node.py`: receives `/cmd_vel` (`geometry_msgs/Twist`), converts
  linear.x/angular.z to direct-drive wheel RPM and calls the existing Controller.
- `config/robot.yaml`: serial port, wheel dimensions, motor directions and limits.

The copied library retains the original functions. Local changes are PyModbus
3.6.x compatibility (`slave=`), bounded reads, checked write errors, detection of
any nonzero fault word, and a read-back-verified 500 ms communication watchdog
(V4 RS485 manual v1.06, register 0x2000). Its old circumference/counts defaults
are uncalibrated; this node does not use them or publish odometry. The imported
bench library has no explicit license file; verify its licensing before redistribution.

## Build and safe startup

Run from the workspace, never from `src`:

```bash
cd "/media/rishmika/Shared Data/Idea8/AGV/ros2_ws"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to agv_bringup
source install/setup.bash
ros2 run agv_bringup motor_node
```

The last command is **disconnected by default** and opens no motor port.
`ros2 launch agv_bringup robot.launch.py` still starts only obstacle stopping by
default; both motor hardware and the follower are opt-in.

For the real serial link, install the Ubuntu 24.04 dependencies:

```bash
sudo apt install python3-pymodbus python3-serial python3-numpy
```

Supported PyModbus API: `>=3.6,<3.7`; the real connection explicitly rejects other
versions. Offline tests inject a fake client and need no PyModbus installation.

## Before enabling real motors

1. Lift and secure the drive wheels and have an independent hardware emergency
   stop available. This ROS node is not a safety-rated stop.
2. Set a stable motor port, measured drive-wheel centre spacing (metres), and
   verified left/right directions (+1/-1) in `config/robot.yaml`. Zero placeholders
   intentionally prevent the port from opening. Do not reuse Gazebo's provisional spacing.
3. Keep the initial `max_rpm: 10.0` limit. The 131 mm wheels are assumed direct-drive.
4. Close bench scripts: this node must be the only owner of the motor port. Do
   not run Gazebo command publishers in the real robot's ROS domain.

Only after these checks, `ros2 launch agv_bringup robot.launch.py enable_motors:=true`
opens/configures the driver, enables both channels at zero RPM and waits for a
fresh `/cmd_vel`. Use `/cmd_vel_raw` for normal teleoperation so obstacle stopping
is not bypassed. Real ultrasonic/MTS160 drivers are not launched here yet; absent
range readings cause the obstacle-stop node to keep commanding zero.

Commands time out to zero after 0.3 s. Invalid commands, driver faults, serial
errors or a polling stall stop/disable and close the link; fix the cause and
restart. Shutdown attempts zero RPM and disable independently. No automatic
fault clearing or reconnection is performed. Configuration changes require restart.

## Offline tests

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest -q src/agv_bringup/test
```

Tests exercise the copied Controller through a fake register client and the
real command/stop callbacks without ROS discovery or physical devices. Hardware
behaviour, braking distances and encoder odometry still require commissioning.
The follower also needs the compatibility/control work listed in
`../agv_control/README.md` before real station missions.
