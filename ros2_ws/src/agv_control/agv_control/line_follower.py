import math
import rclpy
import rclpy.parameter
from rclpy.node import Node
from geometry_msgs.msg import Twist
from naviq_msgs.msg import TrackDetection, Navicode
from agv_control.mission_planner import load_route_map, plan_mission
from agv_control.junction import JunctionDetector



class Follower(Node):

    def __init__(self):
        super().__init__('line_follower')
        self.declare_parameter('route', ['R'])   # one 'L' or 'R' per junction (fork or merge)
        self.declare_parameter('speed', 0.3)
        self.declare_parameter('kp', 8.0)        # rad/s per metre of tape offset
        self.declare_parameter('ka', 1.0)        # rad/s per radian of tape angle
        self.route_i = 0                         # which junction we are at
        self.junctions =JunctionDetector()
        self.w = 0.0
        self.lost = 0
        self.create_subscription(TrackDetection, 'mts160/track', self.on_track, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # naviq_msgs/Navicode publishing
        self.declare_parameter('stops', [2, 4])   # Navicode values to stop at, in order
        self.declare_parameter('dwell', 3.0)      # seconds to wait at a stop
        self.declare_parameter('spin_speed', 0.6) # rad/s while turning around
        self.state = 'FOLLOW'
        self.stop_i = 0                           # which entry of 'stops' is next

        self.declare_parameter('route_map_file', '')  # empty = installed station-loop configuration
        self.declare_parameter('targets', [''])      # e.g. [S1, S3]; empty = use route/stops as given
        targets = [t for t in self.get_parameter('targets').value if t]
        if targets:
            route_map = load_route_map(self.get_parameter('route_map_file').value or None)
            route, stops = plan_mission(targets, route_map)
            self.set_parameters([rclpy.parameter.Parameter('route', value=route),
                                 rclpy.parameter.Parameter('stops', value=stops)])
            self.get_logger().info(f'mission {targets}: route {"".join(route)}, stops {stops}')

        self.state_start = self.get_clock().now()
        self.turned = 0.0                         # radians turned in the current spin
        self.create_subscription(Navicode, 'mts160/navicode', self.on_navicode, 10)




    def on_navicode(self, msg):
        stops = self.get_parameter('stops').value
        if (msg.is_new and self.state == 'FOLLOW'
                and self.stop_i < len(stops) and msg.code == stops[self.stop_i]):
            self.get_logger().info(f'station {msg.code} reached, stopping')
            self.state = 'DWELL'
            self.state_start = self.get_clock().now()

    def on_track(self, msg):
        if self.state == 'FOLLOW':
            self.follow(msg)
            return

        cmd = Twist()                                  # all zero: stand still
        if self.state == 'DWELL':
            waited = (self.get_clock().now() - self.state_start).nanoseconds * 1e-9
            if waited >= self.get_parameter('dwell').value:
                self.stop_i += 1
                self.state = 'SPIN'
                self.turned = 0.0
                self.get_logger().info('turning around')

        elif self.state == 'SPIN':
            spin = self.get_parameter('spin_speed').value
            cmd.angular.z = spin
            self.turned += spin * 0.01                 # track messages arrive every 10 ms
            centred = (msg.tape_detected and abs(msg.left.position_mm) < 10
                       and abs(msg.left.angle_deg) < 10)
            if self.turned >= math.radians(150) and centred:
                cmd.angular.z = 0.0
                self.reset_junctions()
                if self.stop_i >= len(self.get_parameter('stops').value):
                    self.state = 'DONE'
                    self.get_logger().info('mission complete')
                else:
                    self.state = 'FOLLOW'
                    self.get_logger().info('back on the tape')

        self.pub.publish(cmd)

    def reset_junctions(self):
        self.junctions.reset()
        self.w = 0.0


    def side(self):
        route = self.get_parameter('route').value
        return route[min(self.route_i, len(route) - 1)]   # past the end: keep the last choice

    def count_junctions(self, msg):
        event =self.junctions.update(msg.single_track, msg.left.position_mm, msg.left.angle_deg, msg.right.position_mm, msg.right.angle_deg)

        if event == 'start':
            self.get_logger().info(f'junction {self.route_i + 1}: taking {self.side()} side')

        elif event is not None:
            self.get_logger().info(f'junction {self.route_i + 1}: {event[1]}, took {self.side()}')
            self.route_i += 1
            

    def follow(self, msg):
        cmd = Twist()
        speed = self.get_parameter('speed').value

        if not msg.tape_detected:
            self.lost += 1
            if self.lost < 50:                   # 0.5 s: keep the last steering
                cmd.linear.x = speed
                cmd.angular.z = self.w
            else:
                self.get_logger().info('tape lost, stopping', once=True)
            self.pub.publish(cmd)                # after that: zero, so it stops
            return
        self.lost = 0

        self.count_junctions(msg)
        track = msg.left if self.side() == 'L' else msg.right

        e = -track.position_mm / 1000.0          # MTS +right -> ROS +left, metres
        a = -math.radians(track.angle_deg)       # same sign flip for the angle
        w = self.get_parameter('kp').value * e + self.get_parameter('ka').value * a
        self.w = max(-1.5, min(1.5, w))

        cmd.linear.x = speed
        cmd.angular.z = self.w
        self.pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(Follower())
    rclpy.shutdown()
