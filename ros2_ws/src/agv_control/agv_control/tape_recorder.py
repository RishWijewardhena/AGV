"""Record a trip: junction, station and turn events with wheel odometry."""

import json
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from naviq_msgs.msg import Navicode
from std_srvs.srv import Trigger

from agv_control.trip import Trip, parse_event, yaw


class TapeRecorder(Node):

    def __init__(self):
        super().__init__('tape_recorder')
        self.declare_parameter('output', '../agv_maps/trip.json')
        self.trip = Trip()
        self.saved = 0
        self.create_subscription(Odometry, 'wheel/odom', self.on_odom, 10)
        self.create_subscription(String, 'junction_events', self.on_event, 10)
        self.create_subscription(Navicode, 'mts160/navicode', self.on_navicode, 10)
        self.create_service(Trigger, '~/save_trip', self.on_save)

    def on_odom(self, msg):
        p = msg.pose.pose
        self.trip.move(p.position.x, p.position.y, yaw(p.orientation))

    def record(self, entry):
        if self.trip.record(entry) is None:
            self.get_logger().warn(f'no odometry yet, skipping {entry}')
        else:
            self.get_logger().info(str(entry))

    def on_event(self, msg):
        entry = parse_event(msg.data)
        if entry['event'] == 'done':
            self.save()
        else:
            self.record(entry)

    def on_navicode(self, msg):
        if msg.is_new:                              # only the first message for each tag
            self.record({'event': 'station', 'code': msg.code})

    def save(self):
        path = os.path.abspath(os.path.expanduser(self.get_parameter('output').value))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as out:
            json.dump(self.trip.log, out, indent=1)
        self.saved = len(self.trip.log)
        message = f'saved {len(self.trip.log)} events to {path}'
        self.get_logger().info(message)
        return message

    def on_save(self, request, response):
        try:
            response.message = self.save()
            response.success = True
        except OSError as error:            # bad path, no permission, disk full
            response.success = False
            response.message = str(error)
        return response


def main():
    rclpy.init()
    node = TapeRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.saved != len(node.trip.log):                                 # Ctrl+C still keeps the trip
            node.save()

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
