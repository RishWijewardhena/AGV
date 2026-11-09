import json
import math
import os
import rclpy
import rclpy.parameter
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from naviq_msgs.msg import TrackDetection, Navicode
from agv_interfaces.action import Explore
from agv_control.mission_planner import load_route_map, plan_mission
from agv_control.junction import JunctionDetector
from std_msgs.msg import String
from agv_control import map_builder, route_planner
from agv_control.explorer import Explorer
from agv_control.trip import parse_event, yaw


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
        self.events= self.create_publisher(String, 'junction_events', 10)  # read by tape_recorder

        self.declare_parameter('stops', [2, 4])   # Navicode values to stop at, in order
        self.declare_parameter('dwell', 3.0)      # seconds to wait at a stop
        self.declare_parameter('spin_speed', 0.6) # rad/s while turning around
        self.declare_parameter('max_turn', 220.0) # deg: stop if the tape is not found by then
        self.state = 'FOLLOW'
        self.stop_i = 0                           # which entry of 'stops' is next
        self.stop_code = None                     # Navicode of the tag we last stopped at

        stations_file = os.path.join(get_package_share_directory('agv_control'), 'config', 'stations.json')
        with open(stations_file, encoding='utf-8') as f:
            self.stations = json.load(f)          # station name -> Navicode

        self.declare_parameter('explore', False)  # wait for an Explore goal instead of driving a route
        self.explorer = None                      # the exploration in progress, if any
        self.goal = None                          # its action goal
        self.explore_side = 'R'                   # side picked at the junction being entered
        if self.get_parameter('explore').value:
            self.state = 'IDLE'
            self.explore_server = ActionServer(
                self, Explore, '~/explore', self.on_explore, goal_callback=self.on_explore_goal,
                cancel_callback=lambda goal: CancelResponse.ACCEPT)

        self.declare_parameter('route_map_file', '')  # empty = installed station-loop configuration
        self.declare_parameter('site_map_file', '')   # taught map from build_map; empty = loop-only planner
        self.declare_parameter('targets', [''])      # e.g. [S1, S3]; empty = use route/stops as given
        targets = [t for t in self.get_parameter('targets').value if t]
        if targets and not self.get_parameter('explore').value:
            site_map_file = self.get_parameter('site_map_file').value
            if site_map_file:
                route, stops = route_planner.plan_mission(
                    route_planner.load_map(site_map_file), [self.stations[t] for t in targets],
                    home=self.stations['START'])
            else:
                route_map = load_route_map(self.get_parameter('route_map_file').value or None)
                route, stops = plan_mission(targets, route_map)
            self.set_parameters([rclpy.parameter.Parameter('route', value=route),
                                 rclpy.parameter.Parameter('stops', value=stops)])
            self.get_logger().info(f'mission {targets}: route {"".join(route)}, stops {stops}')

        self.state_start = self.get_clock().now()
        self.turned = 0.0                         # radians turned in the current spin
        self.heading = None                       # odometry yaw: from the gyro when the robot has one
        self.create_subscription(Odometry, 'wheel/odom', self.on_odom, 10)
        self.create_subscription(Navicode, 'mts160/navicode', self.on_navicode, 10)


    def announce(self, text):
        """Tell the recorder to what has happned, e.g. fork,R or turn,R"""
        msg = String()
        msg.data = text
        self.events.publish(msg)
        if self.explorer is not None and text != 'done':
            self.explorer.trip.record(parse_event(text))

    def on_odom(self, msg):
        p = msg.pose.pose
        heading = yaw(p.orientation)
        if self.state == 'SPIN' and self.heading is not None:
            step = heading - self.heading
            self.turned += abs(math.atan2(math.sin(step), math.cos(step)))
        self.heading = heading
        if self.explorer is not None:
            self.explorer.trip.move(p.position.x, p.position.y, heading)

    def on_explore_goal(self, request):
        if self.goal is not None or not request.map_file:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def on_explore(self, goal):
        """Explore from START until every junction end is driven, then save the map."""
        self.goal = goal
        self.explorer = Explorer(self.stations['START'])
        self.explore_done = Future()
        self.route_i = 0
        self.lost = 0
        self.reset_junctions()
        self.state = 'FOLLOW'
        self.get_logger().info('exploring')

        success, message = await self.explore_done
        nodes, edges = self.explorer.graph()
        if success:
            try:
                path = map_builder.write_map(goal.request.map_file, self.explorer.site_map())
                message = f'map saved to {path}'
            except OSError as error:            # bad path, no permission, disk full
                success, message = False, str(error)
        self.get_logger().info(f'exploration ended: {message}')
        if goal.is_cancel_requested:
            goal.canceled()
        elif success:
            goal.succeed()
        else:
            goal.abort()
        self.goal = self.explorer = None
        return Explore.Result(success=success, message=message, nodes=len(nodes), edges=len(edges))

    def finish_explore(self, success, message):
        self.state = 'IDLE'
        self.explore_done.set_result((success, message))

    def report(self):
        nodes, open_ports = self.explorer.progress()
        self.goal.publish_feedback(Explore.Feedback(nodes=nodes, open_ports=open_ports))

    def on_navicode(self, msg):
        if not msg.is_new:
            return
        if self.explorer is not None:
            stop = self.explorer.station(msg.code)
            self.report()
        else:
            stops = self.get_parameter('stops').value
            stop = self.stop_i < len(stops) and msg.code == stops[self.stop_i]
        if stop and self.state == 'FOLLOW':
            self.get_logger().info(f'station {msg.code} reached, stopping')
            self.stop_code = msg.code
            self.state = 'DWELL'
            self.state_start = self.get_clock().now()

    def on_track(self, msg):
        if self.goal is not None and self.goal.is_cancel_requested and self.state != 'IDLE':
            self.finish_explore(False, 'cancelled')

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
                self.announce('turn')

        elif self.state == 'SPIN':
            cmd.angular.z = self.get_parameter('spin_speed').value
            centred = (msg.tape_detected and abs(msg.left.position_mm) < 10
                       and abs(msg.left.angle_deg) < 10)
            if self.turned > math.radians(self.get_parameter('max_turn').value):
                cmd.angular.z = 0.0                    # the tape decides; past the limit, give up
                self.state = 'DONE'
                self.get_logger().error('no tape after turning around, stopping')
                if self.explorer is not None:
                    self.finish_explore(False, 'no tape after turning around')
            elif self.turned >= math.radians(150) and centred:
                cmd.angular.z = 0.0
                self.reset_junctions()
                if self.explorer is not None:
                    done = self.stop_code == self.explorer.home and self.explorer.finished()
                else:
                    done = self.stop_i >= len(self.get_parameter('stops').value)
                if done:
                    self.state = 'DONE'
                    self.get_logger().info('mission complete')
                    self.announce('done')
                    if self.explorer is not None:
                        self.finish_explore(True, 'explored')
                else:
                    self.state = 'FOLLOW'
                    self.get_logger().info('back on the tape')

        self.pub.publish(cmd)

    def reset_junctions(self):
        self.junctions.reset()
        self.w = 0.0


    def side(self):
        if self.explorer is not None:
            return self.explore_side
        route = self.get_parameter('route').value
        return route[min(self.route_i, len(route) - 1)]   # past the end: keep the last choice

    def count_junctions(self, msg):
        event =self.junctions.update(msg.single_track, msg.left.position_mm, msg.left.angle_deg, msg.right.position_mm, msg.right.angle_deg)
        if (self.explorer is not None and not msg.single_track
                and self.junctions.wide == 1 and not self.junctions.in_junction):
            self.explore_side = self.pick_side(msg)

        if event == 'start':
            self.get_logger().info(f'junction {self.route_i + 1}: taking {self.side()} side')
            self.announce(f'start,{self.side()}')

        elif event is not None:
            self.get_logger().info(f'junction {self.route_i + 1}: {event[1]}, took {self.side()}')
            self.announce(f'{event[1]},{self.side()}')
            self.route_i += 1
            if self.explorer is not None:
                self.report()

    def pick_side(self, msg):
        """Explore mode, first two-track message: a merge keeps our track, a fork asks the explorer."""
        if self.junctions.first_gap >= self.junctions.merge_gap:     # the other track joins from afar
            return 'L' if abs(msg.left.position_mm) < abs(msg.right.position_mm) else 'R'
        try:
            return self.explorer.choose()
        except ValueError as error:
            self.finish_explore(False, str(error))
            return self.explore_side


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
                if self.explorer is not None:
                    self.finish_explore(False, 'tape lost')
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
