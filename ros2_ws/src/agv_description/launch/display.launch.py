"""Show the AGV model in RViz with sliders for its joints, to check the description without Gazebo."""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('agv_description')
    xacro_file = PathJoinSubstitution([share, 'urdf', 'agv.urdf.xacro'])
    description = ParameterValue(Command(['xacro "', xacro_file, '"']), value_type=str)
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'display.rviz'])]),
    ])
