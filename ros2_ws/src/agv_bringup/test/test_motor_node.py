"""Offline callback tests: no ROS discovery, serial ports, or physical motors."""
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import Twist

from agv_bringup.motor_node import MotorNode, wheel_rpm


def test_forward_reverse_spin_and_proportional_rpm_limit():
    factor = 60/(2*math.pi*.0655)
    assert wheel_rpm(.01, 0, .0655, .4, 1, -1, 10) == (round(.01*factor), -round(.01*factor))
    assert wheel_rpm(-.01, 0, .0655, .4, 1, -1, 10) == (-round(.01*factor), round(.01*factor))
    left, right = wheel_rpm(0, .1, .0655, .4, 1, -1, 10)
    assert left < 0 and right < 0
    assert wheel_rpm(1, 1, .0655, .4, 1, -1, 10) == (7, -10)
    assert wheel_rpm(1, 0, .0655, .4, 1, -1, 10.8) == (10, -10)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_commands_rejected(value):
    with pytest.raises(ValueError):
        wheel_rpm(value, 0, .0655, .4, 1, -1, 10)


def make_node(monkeypatch):
    # Exercise the real callbacks, but do not construct a ROS participant.
    node = object.__new__(MotorNode)
    node.motors = Mock()
    node.motors.get_fault_code.return_value = ((False, 0), (False, 0))
    node.command = None
    node.last_tick = 10.0
    node.config = dict(wheel_radius=.0655, wheel_separation=.4,
                       left_direction=1, right_direction=-1, max_rpm=10.0,
                       command_timeout=.3)
    node.get_logger = lambda: Mock()
    monkeypatch.setattr('agv_bringup.motor_node.time.monotonic', lambda: 10.05)
    return node


def test_fresh_command_drives_and_stale_command_becomes_zero(monkeypatch):
    node = make_node(monkeypatch)
    motors = node.motors
    command = Twist()
    command.linear.x = .01
    node.receive_command(command)
    node.send_command()
    assert motors.set_rpm.call_args.args == (1, -1)
    monkeypatch.setattr('agv_bringup.motor_node.time.monotonic', lambda: 10.36)
    node.send_command()
    assert motors.set_rpm.call_args.args == (0, 0)


@pytest.mark.parametrize('failure', ['fault', 'read', 'write', 'nan', 'stall'])
def test_errors_stop_close_and_do_not_resume(monkeypatch, failure):
    node = make_node(monkeypatch)
    motors = node.motors
    if failure == 'fault':
        motors.get_fault_code.return_value = ((True, 0x0820), (False, 0))
    elif failure == 'read':
        motors.get_fault_code.side_effect = IOError('Lost connection')
    elif failure == 'write':
        motors.set_rpm.side_effect = IOError('Lost connection')
    elif failure == 'nan':
        command = Twist()
        command.linear.x = float('nan')
        node.receive_command(command)
    else:
        node.last_tick = 9.0
    node.send_command()
    assert node.motors is None
    assert motors.disable_motor.called
    assert motors.client.close.called
    count = motors.set_rpm.call_count
    node.receive_command(Twist())
    node.send_command()
    assert motors.set_rpm.call_count == count


def test_defaults_never_construct_controller(monkeypatch):
    from rclpy.node import Node
    monkeypatch.setattr(Node, '__init__', lambda self, *a, **kw: None)
    monkeypatch.setattr(Node, 'declare_parameter',
                        lambda self, name, value, *a, **kw: SimpleNamespace(value=value))
    monkeypatch.setattr(Node, 'get_logger', lambda self: Mock())
    factory = Mock()
    node = MotorNode(controller_factory=factory)
    assert node.motors is None
    factory.assert_not_called()
