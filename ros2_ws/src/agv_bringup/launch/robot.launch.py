"""Real-robot startup; motor hardware is explicitly opt-in."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('agv_bringup'), 'config', 'robot.yaml'])
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
             parameters=[config, {'enable_motors': True}]),
        Node(package='agv_control', executable='obstacle_stop', output='screen',
             parameters=[config]),
        Node(package='agv_control', executable='line_follower', output='screen',
             condition=IfCondition(LaunchConfiguration('enable_follower')),
             parameters=[config], remappings=[('cmd_vel', '/cmd_vel_raw')],
             ros_arguments=['-p', ['targets:=', LaunchConfiguration('targets')]]),
    ])
