"""Build the robot description and check it against geometry.yaml."""

import os
import xml.etree.ElementTree as ET

import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

from agv_description.geometry import load

SHARE = get_package_share_directory('agv_description')


def robot(sim):
    doc = xacro.process_file(os.path.join(SHARE, 'urdf', 'agv.urdf.xacro'),
                             mappings={'sim': str(sim).lower()})
    return ET.fromstring(doc.toxml())


def joint(root, name):
    origin = root.find(f"joint[@name='{name}']/origin")
    xyz = [float(v) for v in origin.get('xyz').split()]
    yaw = float(origin.get('rpy', '0 0 0').split()[2])
    return xyz + [yaw]


@pytest.mark.parametrize('sim', [False, True])
def test_weighs_30_kg(sim):
    masses = [float(m.get('value')) for m in robot(sim).iter('mass')]
    assert sum(masses) == pytest.approx(30.0)


def test_gazebo_parts_only_with_sim():
    assert robot(False).find('gazebo') is None
    assert len(robot(True).findall('gazebo')) > 0


def test_meshes_exist():
    for mesh in robot(False).iter('mesh'):
        path = mesh.get('filename').replace('package://agv_description', SHARE)
        assert os.path.isfile(path), path


def test_frames_match_geometry():
    root, g = robot(False), load()
    assert joint(root, 'mts160_link_joint') == pytest.approx(g['mts160'])
    assert joint(root, 'lidar_link_joint') == pytest.approx(g['lidar'])
    assert joint(root, 'imu_link_joint') == pytest.approx(g['imu'])
    for corner, pose in g['ultrasonic'].items():
        assert joint(root, f'ultrasonic_{corner}_link_joint') == pytest.approx(pose)
    left, right = joint(root, 'left_wheel_joint'), joint(root, 'right_wheel_joint')
    assert left[1] - right[1] == pytest.approx(g['wheel_separation'])
    assert left[2] == pytest.approx(g['wheel_radius'])
