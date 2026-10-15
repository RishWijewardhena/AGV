import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16

class Follower(Node):
    
    def __init__(self):
        super().__init__('line_follower')
        self.declare_parameter('choice','center') #center, left, right
        self.declare_parameter('speed',0.3)
        self.declare_parameter('kp',8.0)
        self.declare_parameter('kd',0.1)
        self.prev_e=None
        self.w=0.0
        self.lost=0
        self.dt=0.01
        self.create_subscription(UInt16,'tape_cells',self.on_cells,10)
        self.pub=self.create_publisher(Twist,'cmd_vel',10)

    def on_cells(self,msg):
        on=[i for i in range(16) if msg.data & (1<<i)] # cells over the tape
        cmd=Twist()

        if not on:
            self.lost+=1
            if self.lost<50:
                cmd.linear.x=self.get_parameter('speed').value
                cmd.angular.z=self.w

            self.pub.publish(cmd)
            return
        self.lost=0

        off=lambda i: (7.5-i)*0.01 # lateral offset of cell i, +left
        choice=self.get_parameter('choice').value

        if choice=='left':
            e=off(min(on))-0.01

        elif choice=='right':
            e=off(max(on))+0.01

        else:
            e=sum(off(i) for i in on)/len(on)

        d=0.0  if self.prev_e is None else (e-self.prev_e)/self.dt
        self.prev_e=e
        w=self.get_parameter('kp').value*e + self.get_parameter('kd').value*d
        self.w=max(-1.5,min(1.5,w))
        cmd.linear.x=self.get_parameter('speed').value
        cmd.angular.z=self.w
        self.pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(Follower())
    rclpy.shutdown()

