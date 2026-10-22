import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, Range
from tf2_ros import StaticTransformBroadcaster


MOUNTS = {
    'front_left':  (0.31,  0.20, 0.18, 0.0),
    'front_right': (0.31, -0.20, 0.18, 0.0),
    'rear_left':   (-0.31,  0.20, 0.18, math.pi),
    'rear_right':  (-0.31, -0.20, 0.18, math.pi),
}


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
        self.static_tf = StaticTransformBroadcaster(self)

        transforms = []

        for name, (x, y, z, yaw) in MOUNTS.items():
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

            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'base_link'
            transform.child_frame_id = f'ultrasonic_{name}_link'

            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z

            transform.transform.rotation.z = math.sin(yaw / 2)
            transform.transform.rotation.w = math.cos(yaw / 2)

            transforms.append(transform)

        self.static_tf.sendTransform(transforms)

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