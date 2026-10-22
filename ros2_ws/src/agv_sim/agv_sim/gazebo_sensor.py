"""Publish virtual MTS160 readings from Gazebo ground truth, never wheel odometry."""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from naviq_msgs.msg import TrackDetection

from agv_sim.gazebo_sensor_geometry import StraightTapeSensor


class GazeboSensor(Node):
    def __init__(self):
        super().__init__('gazebo_mts160')
        world_file = self.declare_parameter('world_file', '').value
        self.sensor = StraightTapeSensor(world_file)
        self.pose = None
        self.create_subscription(TFMessage, '/gazebo/ground_truth', self.on_pose, 10)
        self.publisher = self.create_publisher(TrackDetection, '/mts160/track', 10)
        self.create_timer(0.01, self.publish_reading)

        # Sensing frame is on the bottom face, not the CAD housing centre.
        self.static_tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'mts160_link'
        transform.transform.translation.x = self.sensor.sensor_x
        transform.transform.translation.y = self.sensor.sensor_y
        transform.transform.translation.z = self.sensor.sensor_bottom
        transform.transform.rotation.w = 1.0
        self.static_tf.sendTransform(transform)
        self.get_logger().info('Virtual MTS160: straight tape, Gazebo ground truth, 100 Hz')

    def on_pose(self, message):
        for transform in message.transforms:
            if transform.child_frame_id == 'agv':
                self.pose = transform
                return

    def publish_reading(self):
        now = self.get_clock().now()
        message = TrackDetection()
        message.header.stamp = now.to_msg()
        message.header.frame_id = 'mts160_link'
        reading = None
        if self.pose is not None:
            age = (now-Time.from_msg(self.pose.header.stamp)).nanoseconds*1e-9
            if 0 <= age <= 0.1:
                p = self.pose.transform.translation
                q = self.pose.transform.rotation
                reading = self.sensor.read(p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if reading is not None:
            lp, la, rp, ra = reading
            message.left.position_mm, message.left.angle_deg = float(lp), float(la)
            message.right.position_mm, message.right.angle_deg = float(rp), float(ra)
            message.tape_detected = True
            message.single_track = (lp == rp and la == ra)
            message.strength = TrackDetection.STRENGTH_STRONG
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = GazeboSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
