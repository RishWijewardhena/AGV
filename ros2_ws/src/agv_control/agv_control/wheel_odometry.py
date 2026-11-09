""" publish wheel odometry from the drive wheel joint angles"""

import rclpy
import math
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

from agv_control.odometry import GyroYaw, WheelOdometry

class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry_node')
        self.declare_parameter('wheel_diameter', Parameter.Type.DOUBLE)  # from agv_description geometry
        self.declare_parameter('track_width', Parameter.Type.DOUBLE)
        self.declare_parameter('counts_per_rev', 4096)
        self.declare_parameter('left_joint','left_wheel_joint')
        self.declare_parameter('right_joint','right_wheel_joint')
        self.declare_parameter('publish_tf', True)   # odom -> base_link
        self.declare_parameter('imu_topic', '')      # gyro for the heading; empty = wheels only

        p=lambda name: self.get_parameter(name).value
        self.cpr = p('counts_per_rev')
        self.left_joint,self.right_joint = p('left_joint'),p('right_joint')
        self.odom = WheelOdometry(p('wheel_diameter'), p('track_width'), self.cpr)

        self.pub =self.create_publisher(Odometry, 'wheel/odom', 10)
        self.tf = TransformBroadcaster(self) if p('publish_tf') else None
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)

        self.gyro = None
        self.counts = None           # previous wheel counts
        self.unmoved = 0             # wheel updates in a row without a count change
        if p('imu_topic'):
            self.gyro = GyroYaw()
            self.create_subscription(Imu, p('imu_topic'), self.on_imu, qos_profile_sensor_data)

    def on_imu(self, msg):
        stamp = msg.header.stamp
        self.gyro.add(msg.angular_velocity.z, stamp.sec + stamp.nanosec * 1e-9)


    def to_counts(self, angle):
        """Wheel angle (rad) to whole encoder counts, like the driver reports."""
        return round(angle / (2 * math.pi) * self.cpr)

    def on_joints(self, msg):
        try:
            left = msg.position[msg.name.index(self.left_joint)]
            right = msg.position[msg.name.index(self.right_joint)]
        except (ValueError, IndexError):
            return                    # this message doesn't carry both wheels

        counts = (self.to_counts(left), self.to_counts(right))
        dth = None
        if self.gyro:
            self.unmoved = self.unmoved + 1 if counts == self.counts else 0
            turned = self.gyro.take(still=self.unmoved >= 5)   # 0.1 s without a count at 50 Hz
            if self.gyro.ready:
                dth = turned
        self.counts = counts
        self.odom.update(*counts, dth)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x = self.odom.x
        out.pose.pose.position.y = self.odom.y
        out.pose.pose.orientation.z = math.sin(self.odom.th / 2)
        out.pose.pose.orientation.w = math.cos(self.odom.th / 2)
        self.pub.publish(out)

        if self.tf:
            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = out.child_frame_id
            t.transform.translation.x = self.odom.x
            t.transform.translation.y = self.odom.y
            t.transform.rotation = out.pose.pose.orientation
            self.tf.sendTransform(t)

def main():
    rclpy.init()
    print("Starting wheel odometry node")
    rclpy.spin(WheelOdometryNode())
    print("Wheel odometry node shutting down")
    rclpy.shutdown()
