"""Station-loop world in Gazebo, with a robot model spawned at START.

Swap the robot without touching the world:
    ros2 launch agv_sim gazebo_loop.launch.py robot_file:=/path/to/robot.urdf
"""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from agv_sim.tape_map import START_POSE
from agv_sim.world_builder import write_world


def start_gazebo(context):
    world = write_world(os.path.join(tempfile.gettempdir(), 'agv_station_loop.sdf'))
    args = ['gz', 'sim', '-r', world]
    if LaunchConfiguration('headless').perform(context) == 'true':
        args.insert(2, '-s')
    return [ExecuteProcess(cmd=args, output='screen')]


def generate_launch_description():
    x, y, yaw = START_POSE
    robot_file = LaunchConfiguration('robot_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_file',
            default_value=PathJoinSubstitution([FindPackageShare('agv_sim'), 'models', 'agv_box.sdf']),
            description='robot model to spawn (SDF or URDF)'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true: no Gazebo window'),

        OpaqueFunction(function=start_gazebo),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'agv_world', '-file', robot_file, '-name', 'agv',
                        '-x', str(x), '-y', str(y), '-z', '0.002', '-Y', str(yaw)]),

        Node(package='agv_sim', executable='ultrasonic_sim', output='screen',
             parameters=[{'use_sim_time': True}]),

        Node(package='agv_control', executable='obstacle_stop', output='screen',
             parameters=[{'use_sim_time': True}]),
                

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
             ],
             remappings=[('/model/agv/pose', '/gazebo/ground_truth')],
             parameters=[{'use_sim_time': True}]),
    ])
