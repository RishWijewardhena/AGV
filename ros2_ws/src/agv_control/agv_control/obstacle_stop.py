"""Shared stop-and-wait controller for real or simulated Range readings."""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan

NAMES =('front_left', 'front_right', 'rear_left', 'rear_right')
SOURCES=NAMES+('lidar_front',)
MAX_AGE={'lidar_front': 0.5}
LIDAR_X=0.22 # lidar ahead of base_link
BUMPER_X=0.30 #base link to front face
HALF_WIDTH=0.28 # half the width of the robot

def front_gap(ranges,angle_min,angle_increment,range_min,range_max):
    """
    Disatnce from the bumper to the closest scan point in the corridor
    """

    gap=math.inf

    for i,r in enumerate(ranges):

        #skip inf/NAN and out of range returns
        if math.isinf(r) or math.isnan(r) or r<range_min or r>range_max:
            continue

        a=angle_min+i*angle_increment
        #convert the(r,a) into base_link x
        x=r*math.cos(a)+LIDAR_X
        y=r*math.sin(a)

        #keep the only points ahead of the bumper and inside |y| <=HALF_WIDTH
        if x > BUMPER_X and abs(y) <= HALF_WIDTH:
            gap = min(gap, x - BUMPER_X)

    return gap

class StopGate:
    """ decide whether the ultrasonic sensor permits the movements"""

    def __init__(self):
        self.readings ={}
        self.blocked =True
        self.clear_since=None


    def update(self, name, distance,stamp):
        self.readings[name] = (distance,stamp)

        if distance is None or distance <=0.4:
            self.blocked = True
            self.clear_since = None


    def is_blocked(self, now):
        healthy = all(
            name in self.readings
            and self.readings[name][0] is not None
            and 0 <= now - self.readings[name][1] <= MAX_AGE.get(name, 0.30)
            for name in SOURCES
        )

        if not healthy or any(
            self.readings[name][0] <= 0.40 for name in SOURCES
        ):
            self.blocked = True
            self.clear_since = None
            return True

        if self.blocked:
            if all(self.readings[name][0] >= 0.55 for name in SOURCES):
                if self.clear_since is None or now < self.clear_since:
                    self.clear_since = now

                if now - self.clear_since >= 0.50:
                    self.blocked = False
            else:
                self.clear_since = None

        return self.blocked


class ObstacleStop(Node):
    ''' ROS2 node that subscribes to the ultrasonic sensor readings and publishes a stop command if an obstacle is detected. '''

    def __init__(self):
        super().__init__('obstacle_stop')

        self.gate = StopGate()
        self.command = None
        self.command_time = None
        self.previous_blocked = None

        self.publisher =self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist , '/cmd_vel_raw',self.on_command,10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.range_subscriptions =[]

        for name in NAMES:
            subscription = self.create_subscription(Range, f'/ultrasonic/{name}',lambda message, sensor=name: self.on_range(message ,sensor), qos_profile_sensor_data)
            
            self.range_subscriptions.append(subscription)

        self.create_timer(0.02,self.tick) # 50Hz


    def seconds(self):
        """ Return the current time in seconds. """
        return self.get_clock().now().nanoseconds * 1e-9

    def on_command(self, message):

        if (math.isfinite(message.linear.x) and math.isfinite(message.angular.z)):
            self.command = message
            self.command_time = self.seconds()

        else:
            self.command = None
            self.command_time = None

    def on_scan(self, message):
        gap= front_gap(message.ranges,message.angle_min,message.angle_increment,message.range_min,message.range_max)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.gate.update('lidar_front', gap, stamp)


    def on_range(self, message, name):
            distance = float(message.range)

            limits_valid = (
                math.isfinite(message.min_range)
                and math.isfinite(message.max_range)
                and 0 <= message.min_range < message.max_range
            )

            if not limits_valid or math.isnan(distance):
                distance = None
            elif math.isfinite(distance):
                if not message.min_range <= distance <= message.max_range:
                    distance = None

            # +inf means no return; -inf means too close and causes a stop.
            stamp = (
                message.header.stamp.sec
                + message.header.stamp.nanosec * 1e-9
            )
            self.gate.update(name, distance, stamp)

    def tick(self):
        now = self.seconds()
        blocked = self.gate.is_blocked(now)

        command_fresh = (
            self.command is not None
            and self.command_time is not None
            and 0 <= now - self.command_time <= 0.30
        )

        output = Twist()

        if not blocked and command_fresh:
            output = self.command

        self.publisher.publish(output)

        if blocked != self.previous_blocked:
            if blocked:
                self.get_logger().info(
                    'HOLD: obstacle or unavailable sensor data'
                )
            else:
                self.get_logger().info('CLEAR: motion permitted')

            self.previous_blocked = blocked


def main():
    rclpy.init()
    node = ObstacleStop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.publisher.publish(Twist())
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__== "__main__":
    main()
