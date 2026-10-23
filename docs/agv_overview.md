# AGV project overview

## Purpose

This AGV is a magnetic-tape-guided trolley tug. It travels to a trolley,
locks onto it, tows it to a requested station, releases it, and can return
to START or continue to another task.

## Intended task

1. Follow floor tape and choose the correct branch for the requested station.
2. Recognize stations using coded Navicode markers beside the tape.
3. Stop and wait when an obstacle blocks the path; resume when it clears.
4. Drive into a trolley and engage four solenoid locks.
5. Tow the trolley to its destination, stop, and release the locks.
6. Track travel using wheel encoders read from the motor driver.

## Main components

- **Chassis:** approximately 60 cm long, 46 cm wide, and 28 cm high, with
  two drive wheels near the middle and four corner caster wheels.
- **Drive:** two 131 mm ZLLG50ASM200 hub wheels and one ZLAC8015D V4 dual
  motor driver. The driver connects to the Raspberry Pi through USB–RS485.
- **Guide:** Naviq MTS160 magnetic sensor, intended to sit about 28 mm above
  the floor. It reads tape position, branches, and Navicode markers. Its
  planned Pi connection is CAN through a CANable adapter.
- **Track:** Naviq magnetic tape, 25 mm or 50 mm wide, with opposite-polarity
  markers. The current Gazebo test strip is 25 mm wide and 1.3 mm thick.
- **Obstacle sensing:** four DFRobot I2C ultrasonic sensors at the front and
  rear corners. An RPLidar A1M8 is planned for additional object detection.
- **Trolley coupling:** four solenoid door locks controlled in two pairs by
  a relay module.
- **Control and power:** Raspberry Pi 4B running Ubuntu 24.04 and ROS 2 Jazzy;
  24 V main supply with a 5 V converter for the Pi; emergency stop, buzzer,
  NeoPixel status lights, and a local display.

## Software and current progress

- `agv_control` contains shared route planning, tape following, and obstacle
  stopping.
- `agv_sim` contains an RViz simulation with station routes and a Gazebo
  simulation with a straight tape, virtual MTS160, and four virtual distance
  readings. Gazebo currently uses a simplified robot model.
- `agv_bringup` contains a small motor-control wrapper and startup files.
  Motor activation is disabled by default and has not been commissioned on
  the real robot.
- The SolidWorks URDF and STL meshes have been imported and displayed, but
  they have not replaced the simplified robot in the Gazebo world.

## Remaining work

- Measure the drive-wheel spacing and validate the CAD model for Gazebo physics.
- Add the full station route and trolley interaction to Gazebo.
- Commission the real MTS160 and motor driver; derive odometry from wheel
  encoders. The real route map must be measured rather than copied from the
  RViz example.
- Implement the real I2C ultrasonic node, trolley locks, and status devices.
