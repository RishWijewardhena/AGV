"""Station-loop world in Gazebo, with a robot model spawned at START.

robot:=cad (default) spawns the Fusion model AGV_Design_4; robot:=box the provisional box robot.
"""

import os
import tempfile

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


from agv_sim.tape_map import START_POSE
from agv_sim.world_builder import write_world


# Robot model and the geometry its nodes need. The CAD values are the nodes' defaults (real robot).
ROBOTS = {
    'cad': {'model': 'agv_cad.sdf', 'lidar': (0.312, 0.317), 'params': {}},
    'box': {'model': 'agv_box.sdf', 'lidar': (0.22, 0.3075), 'params': {
        'wheel_odometry': {'wheel_diameter': 0.131, 'track_width': 0.40},
        'gazebo_mts160': {'mts160_x': 0.25},
        'obstacle_stop': {'lidar_x': 0.22, 'bumper_x': 0.30},
        'ultrasonic_sim': {'front_left': [0.31, 0.20, 0.18, 0.0], 'front_right': [0.31, -0.20, 0.18, 0.0],
                           'rear_left': [-0.31, 0.20, 0.18, 3.14159], 'rear_right': [-0.31, -0.20, 0.18, 3.14159]},
    }},
}


def start_gazebo(context):
    world = write_world(os.path.join(tempfile.gettempdir(), 'agv_station_loop.sdf'))
    args = ['gz', 'sim', '-r', world]
    if LaunchConfiguration('headless').perform(context) == 'true':
        args.insert(2, '-s')
    return [ExecuteProcess(cmd=args, output='screen')]


def robot_nodes(context):
    """Spawn the chosen robot and start the nodes that depend on its geometry."""
    robot = ROBOTS[LaunchConfiguration('robot').perform(context)]
    x, y, yaw = START_POSE
    model = os.path.join(get_package_share_directory('agv_sim'), 'models', robot['model'])

    def params(name):
        return [{'use_sim_time': True, **robot['params'].get(name, {})}]

    lidar_x, lidar_z = robot['lidar']
    return [
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'agv_world', '-file', model, '-name', 'agv',
                        '-x', str(x), '-y', str(y), '-z', '0.002', '-Y', str(yaw)]),
        Node(package='agv_sim', executable='ultrasonic_sim', output='screen', parameters=params('ultrasonic_sim')),
        Node(package='agv_control', executable='obstacle_stop', output='screen', parameters=params('obstacle_stop')),
        Node(package='agv_control', executable='wheel_odometry', name='wheel_odometry', output='screen',
             parameters=params('wheel_odometry')),
        Node(package='agv_sim', executable='gazebo_mts160', name='gazebo_mts160', output='screen',
             parameters=params('gazebo_mts160')),
        Node(package='tf2_ros', executable='static_transform_publisher', name='lidar_tf',
             arguments=['--x', str(lidar_x), '--z', str(lidar_z),
                        '--frame-id', 'base_link', '--child-frame-id', 'lidar_link'],
             parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    targets = LaunchConfiguration('targets')
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run gzserver in headless mode'),
        DeclareLaunchArgument('robot', default_value='cad', choices=list(ROBOTS),
                              description='robot model: cad (AGV_Design_4) or box'),

        DeclareLaunchArgument('targets', default_value='[S2]',
                              description='stations to visit in order, e.g. [S1,S3]'),

        DeclareLaunchArgument('trip_file', default_value='maps/trips/trip.json',
                              description='Where tape_recorder saves the trip (relative to the launch folder)'),

        DeclareLaunchArgument('route', default_value='[R]'),
        DeclareLaunchArgument('stops', default_value='[2, 4]'),

        DeclareLaunchArgument('site_map', default_value='',
                              description='taught map from build_map; empty = loop-only planner'),

        DeclareLaunchArgument('explore', default_value='false',
                              description='follower waits for an Explore action goal instead of a mission'),

        # model://agv_sim/meshes/... in the robot models resolves against the install share folder
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.dirname(get_package_share_directory('agv_sim'))),

        OpaqueFunction(function=start_gazebo),
        OpaqueFunction(function=robot_nodes),

        Node(package='agv_control', executable='line_follower', name='line_follower', output='screen',
             parameters=[{'use_sim_time': True,
                          'site_map_file': ParameterValue(LaunchConfiguration('site_map'), value_type=str),
                          'explore': ParameterValue(LaunchConfiguration('explore'), value_type=bool)}],
             ros_arguments=['-p', ['targets:=', targets],
                            '-p', ['route:=', LaunchConfiguration('route')],
                            '-p', ['stops:=', LaunchConfiguration('stops')]],
             remappings=[('cmd_vel', '/cmd_vel_raw')]),

        Node(package='agv_control', executable='tape_recorder', name='tape_recorder', output='screen',
             parameters=[{'use_sim_time': True, 'output': LaunchConfiguration('trip_file')}]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gazebo_bridge',
             output='screen',
             arguments=[
                 '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                 '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                 '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                 '/model/agv/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                 '/ultrasonic/front_left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/ultrasonic/front_right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/ultrasonic/rear_left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/ultrasonic/rear_right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

             ],
             remappings=[('/model/agv/pose', '/gazebo/ground_truth')],
             parameters=[{'use_sim_time': True}]),
    ])
