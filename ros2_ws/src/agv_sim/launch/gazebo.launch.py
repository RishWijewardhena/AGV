"""Station-loop world in Gazebo, with the AGV from agv_description spawned at START."""

import os
import tempfile

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from agv_description.geometry import load, node_params
from agv_sim.tape_map import START_POSE
from agv_sim.world_builder import write_world


def start_gazebo(context):
    world = write_world(os.path.join(tempfile.gettempdir(), 'agv_station_loop.sdf'))
    args = ['gz', 'sim', '-r', world]
    if LaunchConfiguration('headless').perform(context) == 'true':
        args.insert(2, '-s')
    return [ExecuteProcess(cmd=args, output='screen')]


def robot_nodes():
    """Publish the robot description, spawn it, and start the nodes that need its geometry."""
    xacro_file = PathJoinSubstitution([FindPackageShare('agv_description'), 'urdf', 'agv.urdf.xacro'])
    description = ParameterValue(Command(['xacro "', xacro_file, '" sim:=true']), value_type=str)
    geometry = node_params(load())
    geometry['wheel_odometry']['imu_topic'] = '/imu'     # heading from the simulated ICM-20948
    x, y, yaw = START_POSE

    def params(name):
        return [{'use_sim_time': True, **geometry.get(name, {})}]

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': True, 'robot_description': description}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'agv_world', '-topic', 'robot_description', '-name', 'agv',
                        '-x', str(x), '-y', str(y), '-z', '0.002', '-Y', str(yaw)]),
        Node(package='agv_sim', executable='ultrasonic_sim', output='screen', parameters=params('ultrasonic_sim')),
        Node(package='agv_control', executable='obstacle_stop', output='screen', parameters=params('obstacle_stop')),
        Node(package='agv_control', executable='wheel_odometry', name='wheel_odometry', output='screen',
             parameters=params('wheel_odometry')),
        Node(package='agv_sim', executable='gazebo_mts160', name='gazebo_mts160', output='screen',
             parameters=params('gazebo_mts160')),
    ]


def generate_launch_description():
    targets = LaunchConfiguration('targets')
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run gzserver in headless mode'),

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

        # The description's package:// meshes reach Gazebo as model://agv_description/meshes/...
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.dirname(get_package_share_directory('agv_description'))),

        OpaqueFunction(function=start_gazebo),
        *robot_nodes(),

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
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                 '/model/agv/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                 '/ultrasonic/front_left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/ultrasonic/front_right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/ultrasonic/rear_left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/ultrasonic/rear_right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                 '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

             ],
             remappings=[('/model/agv/pose', '/gazebo/ground_truth')],
             parameters=[{'use_sim_time': True}]),
    ])
