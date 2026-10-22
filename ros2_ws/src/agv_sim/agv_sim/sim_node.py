import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist , TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile,DurabilityPolicy
import numpy as np
from std_msgs.msg import ColorRGBA
from agv_sim.tape_map import TAPE_WIDTH, MARKER_SIZE, START_POSE, station_loop, to_segments,read_mts160 ,marker_under_sensor
from naviq_msgs.msg import TrackDetection , Navicode




class Sim(Node):
	def __init__(self):
		super().__init__('sim')
		self.x=self.y=self.th=0.0 #pose
		self.v=self.w=0.0
		self.dt=0.01
		self.last_cmd=self.get_clock().now()
		self.cmd_timeout=0.5
		self.tape_map, self.stations = station_loop()
		self.x, self.y, self.th = START_POSE

		qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
		self.tape_pub=self.create_publisher(MarkerArray,'tape_map',qos)
		self.publish_tape_map()
		self.create_subscription(Twist,'cmd_vel', self.on_cmd,10)
		self.tf = TransformBroadcaster(self)
		self.create_timer(self.dt,self.step)
		self.A, self.B = to_segments(self.tape_map)
		self.sensor_x = 0.25                                  # m ahead of the axle
		self.track_pub = self.create_publisher(TrackDetection, 'mts160/track', 10)
		self.reading = None

		self.nav_pub = self.create_publisher(Navicode, 'mts160/navicode', 10)
		self.nav_code = 0
		self.nav_counter = 0
		self.nav_new = False
		self.over_tag = None          # name of the station whose marker is under the sensor
		self.create_timer(0.1, self.publish_navicode)   # 10 Hz, like TPDO3



		self.sensor_pub = self.create_publisher(Marker, 'sensor_markers', 10)
		self.create_timer(0.05, self.publish_sensor)   # 20 Hz for RViz



	def publish_tape_map(self):
		arr=MarkerArray()
		for i,line in enumerate(self.tape_map):
			m=Marker()
			m.header.frame_id='odom'
			m.ns,m.id='tape',i
			m.type,m.action=Marker.LINE_STRIP,Marker.ADD
			m.scale.x=TAPE_WIDTH
			m.color.r,m.color.g,m.color.b,m.color.a=1.0,0.8,0.0,1.0
			m.pose.orientation.w=1.0
			m.points =[Point(x=float(x),y=float(y),z=0.0) for x,y in line]
			arr.markers.append(m)

		for k, (name,st) in enumerate(self.stations.items()):
			mk = Marker()
			mk.header.frame_id = 'odom'
			mk.ns, mk.id = 'marker', k
			mk.type, mk.action = Marker.CUBE, Marker.ADD
			mk.pose.position.x, mk.pose.position.y = float(st['marker'][0]), float(st['marker'][1])
			mk.pose.orientation.w = 1.0
			mk.scale.x = mk.scale.y = MARKER_SIZE
			mk.scale.z = 0.01
			mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.1, 0.1, 1.0
			arr.markers.append(mk)
			label = Marker()
			label.header.frame_id = 'odom'
			label.ns, label.id = 'label', k
			label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
			label.pose.position.x = float(st['tag'][0])
			label.pose.position.y = float(st['tag'][1])
			label.pose.position.z = 0.4            # float above the floor
			label.pose.orientation.w = 1.0
			label.scale.z = 0.3                    # text height (m)
			label.color.r = label.color.g = label.color.b = label.color.a = 1.0
			label.text = f"{name} ({st['code']})"
			arr.markers.append(label)


		self.tape_pub.publish(arr)


	def on_cmd(self,m):
		self.v=m.linear.x
		self.w=m.angular.z
		self.last_cmd=self.get_clock().now()

	def step(self):

		age=(self.get_clock().now()-self.last_cmd).nanoseconds*1e-9
		if age > self.cmd_timeout:
			self.v=0.0
			self.w=0.0
		self.x+=self.v*math.cos(self.th)*self.dt
		self.y+=self.v*math.sin(self.th)*self.dt
		self.th+= self.w*self.dt

		t=TransformStamped()
		t.header.stamp=self.get_clock().now().to_msg()
		t.header.frame_id='odom'
		t.child_frame_id='base_link'
		t.transform.translation.x=self.x
		t.transform.translation.y=self.y
		t.transform.rotation.z=math.sin(self.th/2)
		t.transform.rotation.w=math.cos(self.th/2)
		self.tf.sendTransform(t)
		self.read_sensor()

	def read_sensor(self):

		self.reading=read_mts160(self.x,self.y,self.th,self.A,self.B,self.sensor_x)
		msg=TrackDetection()
		msg.header.stamp=self.get_clock().now().to_msg()
		msg.header.frame_id = 'mts160_link'
		if self.reading is not None:
			lp, la, rp, ra = self.reading
			msg.left.position_mm, msg.left.angle_deg = float(lp), float(la)
			msg.right.position_mm, msg.right.angle_deg = float(rp), float(ra)
			msg.strength = TrackDetection.STRENGTH_STRONG
			msg.tape_detected = True
			msg.single_track = (lp == rp and la == ra)

			diff = ra - la                       # advisory flags, as defined in the .msg file
			msg.fork = 10 <= diff <= 45
			msg.merge = -45 <= diff <= -10

		over = None
		for name, st in self.stations.items():
			side = marker_under_sensor(self.x, self.y, self.th, *st['marker'], self.sensor_x)
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
		self.nav_new = False                          # only the first message after a new code
		self.nav_pub.publish(msg)



	def publish_sensor(self):
		m = Marker()
		m.header.frame_id = 'base_link'
		m.header.stamp = self.get_clock().now().to_msg()
		m.ns, m.id = 'sensor', 0
		m.type, m.action = Marker.LINE_LIST, Marker.ADD
		m.scale.x = 0.006
		m.pose.orientation.w = 1.0
		m.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0)
		m.points = [Point(x=self.sensor_x, y=0.08, z=0.01), Point(x=self.sensor_x, y=-0.08, z=0.01)]
		self.sensor_pub.publish(m)

		d = Marker()
		d.header = m.header
		d.ns, d.id = 'sensor', 1
		d.type, d.action = Marker.POINTS, Marker.ADD
		d.scale.x = d.scale.y = 0.015
		d.pose.orientation.w = 1.0
		if self.reading is not None:
			lp, _, rp, _ = self.reading
			# MTS +right -> ROS +left: y = -mm / 1000
			d.points = [Point(x=self.sensor_x, y=-lp / 1000, z=0.02), Point(x=self.sensor_x, y=-rp / 1000, z=0.02)]
			d.colors = [ColorRGBA(r=0.2, g=0.4, b=1.0, a=1.0), ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)]
		self.sensor_pub.publish(d)




def main():
	rclpy.init()
	rclpy.spin(Sim())
	rclpy.shutdown()
