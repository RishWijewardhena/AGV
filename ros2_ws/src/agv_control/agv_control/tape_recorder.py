"""Record a trip: junction, station and turn events with wheel odometry."""

import json
import math
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from naviq_msgs.msg import Navicode


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TapeRecorder(Node):

    def __init__(self):
        super().__init__('tape_recorder')
        self.declare_parameter('output', '../agv_maps/trip.json')
        self.pose = None             # latest (x, y, th) from wheel odometry
        self.distance = 0.0          # path length driven (m)
        self.log = []                # recorded events, in order
        self.saved = False
        self.create_subscription(Odometry, 'wheel/odom', self.on_odom, 10)
        self.create_subscription(String, 'junction_events', self.on_event, 10)
        self.create_subscription(Navicode, 'mts160/navicode', self.on_navicode, 10)

    def on_odom(self, msg):
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw(p.orientation))
        if self.pose is not None:    # add the step since the last message
            self.distance += math.hypot(pose[0] - self.pose[0], pose[1] - self.pose[1])
        self.pose = pose

    def record(self, entry):
        if self.pose is None:
            self.get_logger().warn(f'no odometry yet, skipping {entry}')
            return
        x, y, th = self.pose
        entry.update(distance=round(self.distance, 3),
                     x=round(x, 3), y=round(y, 3), th=round(th, 3))
        self.log.append(entry)
        self.get_logger().info(str(entry))

    def on_event(self, msg):
        kind, _, side = msg.data.partition(',')     # 'fork,R' -> 'fork', ',', 'R'
        if kind == 'done':
            self.save()
            return
        entry = {'event': kind}
        if side:
            entry['side'] = side
        self.record(entry)

    def on_navicode(self, msg):
        if msg.is_new:                              # only the first message for each tag
            self.record({'event': 'station', 'code': msg.code})

    def save(self):
        if self.saved:
            return
        path = os.path.expanduser(self.get_parameter('output').value)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as out:
            json.dump(self.log, out, indent=1)
        self.saved = True
        self.get_logger().info(f'saved {len(self.log)} events to {path}')


def main():
    rclpy.init()
    node = TapeRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()                                 # Ctrl+C still keeps the trip
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
