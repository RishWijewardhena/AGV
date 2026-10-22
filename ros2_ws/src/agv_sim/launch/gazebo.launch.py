"""Manual driving with virtual MTS160 readings over the straight tape."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare('agv_sim'), 'worlds', 'agv_world.sdf',
    ])
    gazebo = ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen')
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='gazebo_bridge', output='screen',
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
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(
            target_action=gazebo,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))],
        )),
        gazebo,
        bridge,
        Node(package='agv_sim', executable='gazebo_sensor', output='screen',
             parameters=[{'use_sim_time': True, 'world_file': world}]),

        Node(package='agv_sim', executable='ultrasonic_sim', output='screen',
             parameters=[{'use_sim_time': True}]),

        Node(package='agv_control', executable='obstacle_stop', output='screen',
             parameters=[{'use_sim_time': True}]),
    ])
