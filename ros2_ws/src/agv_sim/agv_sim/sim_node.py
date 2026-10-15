import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist , TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy	
from agv_sim.tape_map import y_fork
import numpy as np
from std_msgs.msg import UInt16,ColorRGBA
from agv_sim.tape_map import y_fork, to_segments,dist_to_tape


class Sim(Node):
	def __init__(self):
		super().__init__('sim')
		self.x=self.y=self.th=0.0 #pose
		self.v=self.w=0.0
		self.dt=0.01
		self.last_cmd=self.get_clock().now()
		self.cmd_timeout=0.5
		self.tape_map=y_fork()
		qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
		self.tape_pub=self.create_publisher(MarkerArray,'tape_map',qos)
		self.publish_tape_map()
		self.create_subscription(Twist,'cmd_vel', self.on_cmd,10)
		self.tf = TransformBroadcaster(self)
		self.create_timer(self.dt,self.step)
		self.A, self.B = to_segments(self.tape_map)
		self.sensor_x = 0.25                                  # m ahead of the axle
		self.cell_off = (7.5 - np.arange(16)) * 0.01          # lateral offsets, +left
		self.tape_half = 0.015
		self.cells_pub = self.create_publisher(UInt16, 'tape_cells', 10)
		self.cells_on = np.zeros(16, dtype=bool)
		self.sensor_pub = self.create_publisher(Marker, 'sensor_markers', 10)
		self.create_timer(0.05, self.publish_sensor)   # 20 Hz for RViz



	def publish_tape_map(self):
		arr=MarkerArray()
		for i,line in enumerate(self.tape_map):
			m=Marker()
			m.header.frame_id='odom'
			m.ns,m.id='tape',i
			m.type,m.action=Marker.LINE_STRIP,Marker.ADD
			m.scale.x=0.03 #30mm tape
			m.color.r,m.color.g,m.color.b,m.color.a=1.0,0.8,0.0,1.0
			m.pose.orientation.w=1.0
			m.points =[Point(x=x,y=y,z=0.0) for x,y in line]
			arr.markers.append(m)
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

		c,s=math.cos(self.th),math.sin(self.th)
		sx=self.x+self.sensor_x*c
		sy=self.y+self.sensor_x*s

		#left of the robot is(-sin.cos)
		pts=np.stack([sx-self.cell_off*s,sy+self.cell_off*c],axis=1)
		on=dist_to_tape(pts,self.A,self.B)<self.tape_half
		word=sum(1<<i for i in range(16) if on[i])
		msg=UInt16()
		msg.data=word
		self.cells_pub.publish(msg)
		self.cells_on=on

	def publish_sensor(self):
		m = Marker()
		m.header.frame_id = 'base_link'
		m.header.stamp = self.get_clock().now().to_msg()
		m.ns, m.id = 'sensor', 0
		m.type, m.action = Marker.POINTS, Marker.ADD
		m.scale.x = m.scale.y = 0.008
		m.pose.orientation.w = 1.0
		for i in range(16):
			m.points.append(Point(x=self.sensor_x, y=float(self.cell_off[i]), z=0.01))
			if self.cells_on[i]:
				m.colors.append(ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))   # on: green
			else:
				m.colors.append(ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0))   # off: grey
		self.sensor_pub.publish(m)



def main():
	rclpy.init()
	rclpy.spin(Sim())
	rclpy.shutdown()
