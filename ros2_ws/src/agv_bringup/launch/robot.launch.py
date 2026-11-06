"""Real-robot startup; motor hardware is explicitly opt-in."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from agv_description.geometry import load, node_params


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('agv_bringup'), 'config', 'robot.yaml'])
    xacro_file = PathJoinSubstitution([FindPackageShare('agv_description'), 'urdf', 'agv.urdf.xacro'])
    description = ParameterValue(Command(['xacro "', xacro_file, '"']), value_type=str)
    geometry = node_params(load())
    return LaunchDescription([
        DeclareLaunchArgument('enable_motors', default_value='false',
                              description='Open the motor port; requires commissioned robot.yaml'),
        DeclareLaunchArgument('enable_follower', default_value='false',
                              description='Enable only after hardware commissioning'),
        DeclareLaunchArgument('targets', default_value='[]',
                              description='Station targets from the configured real-site route map'),
        LogInfo(msg='Motors and follower are disabled by default. Real I2C ultrasonic and '
                    'MTS160 drivers are not launched here yet.'),
        Node(package='agv_bringup', executable='motor_node', output='screen',
             condition=IfCondition(LaunchConfiguration('enable_motors')),
             parameters=[config, geometry['motor_node'], {'enable_motors': True}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description}]),
        Node(package='agv_control', executable='obstacle_stop', output='screen',
             parameters=[config, geometry['obstacle_stop']]),
        Node(package='agv_control', executable='line_follower', output='screen',
             condition=IfCondition(LaunchConfiguration('enable_follower')),
             parameters=[config], remappings=[('cmd_vel', '/cmd_vel_raw')],
             ros_arguments=['-p', ['targets:=', LaunchConfiguration('targets')]]),
    ])
