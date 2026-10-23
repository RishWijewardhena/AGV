"""Virtual MTS160 on the station loop, driven by the robot's true pose in Gazebo.

Same sensor model as sim_node.py; only the pose source differs.
The mounting position is a parameter, so any robot model can be used.
"""

import math

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from naviq_msgs.msg import TrackDetection, Navicode

from agv_sim.tape_map import station_loop, to_segments, read_mts160, marker_under_sensor


class GazeboMTS160(Node):
    def __init__(self):
        super().__init__('gazebo_mts160')
        self.declare_parameter('model_name', 'agv')
        self.declare_parameter('mts160_x', 0.25)        # m ahead of the drive axle
        self.model_name = self.get_parameter('model_name').value
        self.sensor_x = self.get_parameter('mts160_x').value

        self.lines, self.stations = station_loop()
        self.A, self.B = to_segments(self.lines)
        self.pose = None                 # (x, y, yaw) of the robot in the world

        self.nav_code = 0
        self.nav_counter = 0
        self.nav_new = False
        self.over_tag = None

        self.create_subscription(TFMessage, '/gazebo/ground_truth', self.on_pose, 10)
        self.track_pub = self.create_publisher(TrackDetection, '/mts160/track', 10)
        self.nav_pub = self.create_publisher(Navicode, '/mts160/navicode', 10)
        self.create_timer(0.01, self.read_sensor)        # 100 Hz, like TPDO1
        self.create_timer(0.1, self.publish_navicode)    # 10 Hz, like TPDO3

    def on_pose(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == self.model_name:
                q = t.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self.pose = (t.transform.translation.x, t.transform.translation.y, yaw)
                return

    def read_sensor(self):
        msg = TrackDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mts160_link'
        if self.pose is None:
            self.track_pub.publish(msg)
            return
        x, y, th = self.pose

        reading = read_mts160(x, y, th, self.A, self.B, self.sensor_x)
        if reading is not None:
            lp, la, rp, ra = reading
            msg.left.position_mm, msg.left.angle_deg = float(lp), float(la)
            msg.right.position_mm, msg.right.angle_deg = float(rp), float(ra)
            msg.strength = TrackDetection.STRENGTH_STRONG
            msg.tape_detected = True
            msg.single_track = (lp == rp and la == ra)
            diff = ra - la
            msg.fork = 10 <= diff <= 45
            msg.merge = -45 <= diff <= -10

        over = None
        for name, st in self.stations.items():
            side = marker_under_sensor(x, y, th, *st['marker'], self.sensor_x)
            if side == 'left':
                msg.left_marker = True
            elif side == 'right':
                msg.right_marker = True
            if side:
                over = name
        if self.over_tag and over is None:          # just left a marker: "decode" it
            self.nav_code = self.stations[self.over_tag]['code']
            self.nav_counter = (self.nav_counter + 1) % 256
            self.nav_new = True
            self.get_logger().info(f'Navicode {self.nav_code} ({self.over_tag})')
        self.over_tag = over

        self.track_pub.publish(msg)

    def publish_navicode(self):
        msg = Navicode()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mts160_link'
        msg.code = self.nav_code
        msg.counter = self.nav_counter
        msg.is_new = self.nav_new
        self.nav_new = False
        self.nav_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(GazeboMTS160())
    rclpy.shutdown()
