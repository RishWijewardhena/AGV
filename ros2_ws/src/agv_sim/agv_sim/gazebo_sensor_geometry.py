"""Geometric MTS160 model for the straight tape in agv_world.sdf (no ROS)."""

import math
import xml.etree.ElementTree as ET

from agv_sim.tape_map import read_mts160, to_segments


def flat_pose(element):
    """Read the unscoped planar SDF poses used by this first world."""
    pose = element.find('pose')
    if pose is not None and pose.get('relative_to'):
        raise ValueError('The straight-strip sensor does not resolve relative_to frames')
    values = [float(v) for v in element.findtext('pose', '0 0 0 0 0 0').split()]
    x, y, z, roll, pitch, yaw = values
    if abs(roll) > 1e-9 or abs(pitch) > 1e-9:
        raise ValueError('World tape and sensor mount must use planar SDF poses')
    return x, y, z, yaw


def compose(a, b):
    x, y, z, yaw = a
    bx, by, bz, byaw = b
    c, s = math.cos(yaw), math.sin(yaw)
    return x+c*bx-s*by, y+s*bx+c*by, z+bz, yaw+byaw


class StraightTapeSensor:
    def __init__(self, world_file):
        world = ET.parse(world_file).getroot().find('world')
        tape = world.find("model[@name='tape']")
        link = tape.find('link')
        visual = link.find('visual')
        length, _, thickness = map(float, visual.findtext('geometry/box/size').split())
        tx, ty, tz, yaw = compose(compose(flat_pose(tape), flat_pose(link)), flat_pose(visual))
        dx, dy = length/2*math.cos(yaw), length/2*math.sin(yaw)
        self.a, self.b = to_segments([[(tx-dx, ty-dy), (tx+dx, ty+dy)]])
        self.tape_top = tz + thickness/2

        sensor = world.find("model[@name='agv']/link[@name='mts160_link']")
        housing = sensor.find('visual')
        self.sensor_x, self.sensor_y, sz, syaw = compose(flat_pose(sensor), flat_pose(housing))
        _, width, height = map(float, housing.findtext('geometry/box/size').split())
        if abs(syaw) > 1e-9:
            raise ValueError('The MTS160 bar must be mounted perpendicular to travel')
        self.half_width = width/2
        self.sensor_bottom = sz - height/2

    def read(self, x, y, z, qx, qy, qz, qw):
        """Read at a Gazebo world pose, including sensor-floor clearance.

        This is geometric detection, not a magnetic-field simulation. The simple
        height gate uses 10–50 mm above the tape and rejects tilt over 15 degrees.
        Tape-edge field strength, noise, and marker decoding are not modelled.
        """
        norm = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
        if norm < 1e-9:
            return None
        qx, qy, qz, qw = (v/norm for v in (qx, qy, qz, qw))
        r00, r01, r02 = 1-2*(qy*qy+qz*qz), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)
        r10, r11, r12 = 2*(qx*qy+qw*qz), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qw*qx)
        r20, r21, r22 = 2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx*qx+qy*qy)
        sx, sy, sz = self.sensor_x, self.sensor_y, self.sensor_bottom
        clearance = z+r20*sx+r21*sy+r22*sz-self.tape_top
        if not 0.01 <= clearance <= 0.05 or r22 < math.cos(math.radians(15)):
            return None
        return read_mts160(x+r00*sx+r01*sy+r02*sz,
                           y+r10*sx+r11*sy+r12*sz, math.atan2(r10, r00),
                           self.a, self.b, sensor_x=0, half_width=self.half_width)
