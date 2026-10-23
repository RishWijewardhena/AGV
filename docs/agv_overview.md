# AGV functions and station navigation

## What the AGV does

- Follows magnetic tape to travel between stations.
- Chooses the correct branch at each fork for the requested destination.
- Recognizes a station from its coded magnetic marker and stops there.
- Stops and waits if an obstacle blocks the path, then resumes when clear.
- Is intended to lock onto a trolley, tow it, and release it at another station.

## Sensors used

- **Naviq MTS160 magnetic sensor:** reports tape position and angle, detects
  multiple tracks at a fork, and reads Navicode station markers. The line
  follower uses these readings to steer and choose a branch.
- **Four ultrasonic sensors:** measure distances at the front and rear corners.
  They provide the readings used to stop and wait for obstacles.
- **Wheel encoders in the motor system:** planned source of travel distance and
  odometry for the real AGV.
- **RPLidar A1M8:** planned for additional object detection. It is not the
  source of the magnetic route map.

## How it travels from station to station

1. A mission lists the stations to visit, such as START → S1 → S3 → START.
2. The route planner uses a station map to choose the shorter direction around
   the loop and prepares a left/right choice for each junction.
3. The MTS160 keeps the robot centred on the tape. At a fork, the follower uses
   the planned left or right track and advances to the next junction choice
   after the fork has passed.
4. A Navicode marker confirms arrival at the expected station. The current
   follower stops, waits, turns around, and continues toward the next target.
5. The obstacle stop gate holds the drive command at zero if any distance
   reading indicates a nearby obstacle or a sensor reading is unavailable. It
   allows motion again only after all four readings show a clear path for a
   short period.

## What “without conflicts” means today

The planned junction choices and station codes help one AGV follow the right
route and avoid mistaking one station for another. Obstacle readings make it
stop and wait rather than drive into a blockage. The current stop gate does
not identify which direction an obstacle is in; any of the four sensors can
hold motion.

Coordination between **multiple AGVs** is not implemented. If several robots
will share the tape, they will need a separate system to reserve stations or
track sections and prevent head-on or intersection conflicts.

## Current state

The RViz simulation demonstrates the station loop and station missions. Gazebo
currently demonstrates a straight tape, virtual MTS160 and ultrasonic readings,
and obstacle stopping. Real sensor commissioning, encoder odometry, trolley
locking, and full station missions in Gazebo remain future work.
