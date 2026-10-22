"""Small /cmd_vel wrapper around the existing ZLAC8015D Controller."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


def wheel_rpm(linear, angular, radius, separation, left_sign, right_sign, maximum):
    """Direct-drive wheels: ROS m/s and rad/s to signed integer motor RPM."""
    if not all(math.isfinite(v) for v in (linear, angular)):
        raise ValueError('Velocity must be finite')
    maximum = math.floor(maximum)  # Commands are whole RPM; never round above the cap.
    factor = 60/(2*math.pi*radius)
    left = (linear-angular*separation/2)*factor
    right = (linear+angular*separation/2)*factor
    if not all(math.isfinite(v) for v in (left, right)):
        raise ValueError('Velocity conversion overflow')
    # Scale together to preserve turning curvature when limiting speed.
    scale = max(1.0, abs(left)/maximum, abs(right)/maximum)
    return round(left_sign*left/scale), round(right_sign*right/scale)


class MotorNode(Node):
    def __init__(self, *, controller_factory=None, **kwargs):
        super().__init__('motor_node', **kwargs)
        defaults = dict(enable_motors=False, port='/dev/zlac', node_id=1,
                        wheel_radius=0.0655, wheel_separation=0.0,
                        left_direction=0, right_direction=0,
                        max_rpm=10.0, command_timeout=0.3)
        self.config = {name: self.declare_parameter(
            name, value, ParameterDescriptor(read_only=True)).value
            for name, value in defaults.items()}
        self.motors = None
        self.command = None
        self.last_tick = time.monotonic()
        c = self.config
        if not c['enable_motors']:
            self.get_logger().info('DISCONNECTED: enable_motors=false; no motor port opened.')
            return
        for name in ('wheel_radius', 'wheel_separation', 'max_rpm', 'command_timeout'):
            if not math.isfinite(c[name]) or c[name] <= 0:
                raise ValueError(f'{name} must be positive and configured before enabling motors')
        if not 1 <= c['max_rpm'] <= 1000:
            raise ValueError('max_rpm must be 1..1000, within the motor rating')
        if c['left_direction'] not in (-1, 1) or c['right_direction'] not in (-1, 1):
            raise ValueError('Set commissioned left_direction/right_direction to +1 or -1')

        try:
            if controller_factory is None:
                from agv_bringup.ZLAC8015D import Controller
                controller_factory = Controller
            self.motors = controller_factory(port=c['port'], node_id=c['node_id'])
            self.motors.disable_motor()
            self.motors.set_rpm(0, 0)
            self.motors.set_communication_timeout(500)
            self.motors.set_accel_time(500, 500)
            self.motors.set_decel_time(500, 500)
            self.motors.set_mode(3)
            if self.motors.get_mode() != 3:
                raise IOError('Velocity-mode readback failed')
            self.check_faults()
            self.motors.enable_motor()
            self.last_tick = time.monotonic()
        except Exception as exc:
            self.fail(exc)
            return

        self.create_subscription(Twist, '/cmd_vel', self.receive_command, 1)
        # Always wall time, even if another launch sets use_sim_time by mistake.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.send_command, clock=self.wall_clock)
        self.get_logger().warn('Motors ENABLED at zero speed. Awaiting /cmd_vel.')

    def receive_command(self, msg):
        if self.motors is None:
            return
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(v) for v in values) or any(values[i] != 0 for i in (1, 2, 3, 4)):
            self.fail(ValueError('Expected finite planar linear.x / angular.z commands'))
            return
        self.command = (msg.linear.x, msg.angular.z, time.monotonic())

    def check_faults(self):
        faults = self.motors.get_fault_code()
        if any(code != 0 for flag, code in faults):
            raise IOError('Motor faults: ' + ', '.join(f'0x{code:04X}' for flag, code in faults))

    def send_command(self):
        if self.motors is None:
            return
        try:
            if time.monotonic()-self.last_tick >= 0.4:
                raise IOError('Motor polling stalled; restart required')
            self.check_faults()
            now = time.monotonic()
            if now-self.last_tick >= 0.4:
                raise IOError('Motor feedback exceeded communication-watchdog margin')
            rpm = (0, 0)
            c = self.config
            if self.command and now-self.command[2] < c['command_timeout']:
                rpm = wheel_rpm(*self.command[:2], c['wheel_radius'], c['wheel_separation'],
                                c['left_direction'], c['right_direction'], c['max_rpm'])
            self.motors.set_rpm(*rpm)
            self.last_tick = time.monotonic()
        except Exception as exc:
            self.fail(exc)

    def stop(self):
        motors, self.motors = self.motors, None
        self.command = None
        if motors is None:
            return
        # Attempt disable even when writing zero fails. Never auto-reconnect.
        for action in (lambda: motors.set_rpm(0, 0), motors.disable_motor, motors.client.close):
            try:
                action()
            except Exception as exc:
                self.get_logger().error(f'Stop/close unconfirmed: {exc}')

    def fail(self, error):
        self.get_logger().error(f'MOTOR STOP: {error}. Correct the cause and restart the node.')
        self.stop()

    def destroy_node(self):
        self.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
