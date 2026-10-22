"""Test shared follower -> obstacle stop with synthetic sensors in an isolated ROS domain.

Run from a sourced ros2_ws: python3 ../scripts/check_shared_control.py
No Gazebo or hardware drivers are started.
"""

import os
import threading
import time


def main():
    os.environ['ROS_DOMAIN_ID'] = str(20 + os.getpid() % 180)
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Range
    from naviq_msgs.msg import TrackDetection
    from agv_control.line_follower import Follower
    from agv_control.obstacle_stop import NAMES, ObstacleStop

    rclpy.init(args=['--ros-args', '-r', 'line_follower:cmd_vel:=/cmd_vel_raw'])
    follower = Follower()
    follower.set_parameters([Parameter('speed', value=0.1)])
    gate = ObstacleStop()
    fixture = rclpy.create_node('shared_control_check')
    ranges = {name: fixture.create_publisher(Range, f'/ultrasonic/{name}', 10)
              for name in NAMES}
    track_pub = fixture.create_publisher(TrackDetection, '/mts160/track', 10)
    received = []
    fixture.create_subscription(Twist, '/cmd_vel', received.append, 10)
    executor = SingleThreadedExecutor()
    for node in (follower, gate, fixture):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def stage(seconds, obstacle=False, publish_ranges=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            stamp = fixture.get_clock().now().to_msg()
            if publish_ranges:
                for name, publisher in ranges.items():
                    message = Range()
                    message.header.stamp = stamp
                    message.min_range, message.max_range = 0.02, 5.0
                    message.range = 0.35 if obstacle and name == 'front_left' else float('inf')
                    publisher.publish(message)
            track = TrackDetection()
            track.header.stamp = stamp
            track.tape_detected = track.single_track = True
            track.strength = TrackDetection.STRENGTH_STRONG
            track_pub.publish(track)
            time.sleep(0.05)

    def check(moving, label, publish_ranges=True):
        def matches():
            recent = received[-5:]
            return len(recent) == 5 and all(
                (message.linear.x > 0.09 if moving else message.linear.x == 0)
                and message.angular.z == 0 for message in recent)
        deadline = time.monotonic() + 2
        while not matches() and time.monotonic() < deadline:
            stage(0.1, publish_ranges=publish_ranges)
        assert matches(), label
        print('PASS:', label, flush=True)

    try:
        stage(1.5)
        check(True, 'shared follower drives through the clear obstacle gate')
        stage(0.6, obstacle=True)
        check(False, 'front obstacle stops motion')
        stage(0.3)
        check(False, 'clearance delay prevents immediate restart')
        stage(0.7)
        check(True, 'motion resumes after sustained clearance')
        stage(0.6, publish_ranges=False)
        check(False, 'stale sensor data stops motion', publish_ranges=False)
    finally:
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        for node in (follower, gate, fixture):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
