from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration ,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    targets = LaunchConfiguration('targets')
    rviz_config = PathJoinSubstitution([FindPackageShare('agv_sim'), 'rviz', 'agv_sim.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('targets', default_value='[S2]',
                              description='stations to visit in order, e.g. [S1,S3]'),

        Node(package='agv_sim', executable='sim_node', name='sim', output='screen'),

        Node(package='agv_control', executable='line_follower', name='line_follower', output='screen',
             ros_arguments=['-p', ['targets:=', targets]]),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config]),
    ])
