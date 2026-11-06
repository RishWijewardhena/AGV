import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Range


# Sensor corners; their frames ultrasonic_<corner>_link come from the robot description.
CORNERS = ('front_left', 'front_right', 'rear_left', 'rear_right')


def nearest_distance(scan):
    if not scan.ranges:
        return float('nan')

    # Invalid input remains invalid, rather than reporting a clear path.
    if any(math.isnan(value) for value in scan.ranges):
        return float('nan')

    distance = min(scan.ranges)

    if distance < scan.range_min:
        return float('-inf')  # Too close to measure.

    if distance > scan.range_max:
        return float('inf')   # No return within the sensor range.

    return float(distance)


class UltrasonicSim(Node):
    def __init__(self):
        super().__init__('ultrasonic_sim')

        self.publishers_by_name = {}
        self.subscriptions_by_name = {}

        for name in CORNERS:
            self.publishers_by_name[name] = self.create_publisher(
                Range,
                f'/ultrasonic/{name}',
                10,
            )

            self.subscriptions_by_name[name] = self.create_subscription(
                LaserScan,
                f'/ultrasonic/{name}/scan',
                lambda message, sensor=name: self.on_scan(message, sensor),
                qos_profile_sensor_data,
            )

    def on_scan(self, scan, name):
        message = Range()
        message.header.stamp = scan.header.stamp
        message.header.frame_id = f'ultrasonic_{name}_link'

        # This output represents our simulated ultrasonic approximation.
        message.radiation_type = Range.ULTRASOUND
        message.field_of_view = float(scan.angle_max - scan.angle_min)
        message.min_range = float(scan.range_min)
        message.max_range = float(scan.range_max)
        message.range = nearest_distance(scan)

        self.publishers_by_name[name].publish(message)


def main():
    rclpy.init()
    node = UltrasonicSim()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
