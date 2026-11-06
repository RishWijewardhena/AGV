"""Robot geometry from config/geometry.yaml, as node parameters for the launch files."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory


def load():
    """The geometry dictionary from the installed geometry.yaml."""
    path = os.path.join(get_package_share_directory('agv_description'), 'config', 'geometry.yaml')
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def node_params(geometry):
    """Parameters for each node that needs the robot's geometry, by node name."""
    return {
        'wheel_odometry': {'wheel_diameter': 2 * geometry['wheel_radius'],
                           'track_width': geometry['wheel_separation']},
        'gazebo_mts160': {'mts160_x': geometry['mts160'][0]},
        'obstacle_stop': {'lidar_x': geometry['lidar'][0], 'bumper_x': geometry['bumper_x']},
    }
