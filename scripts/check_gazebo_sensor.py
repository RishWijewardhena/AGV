"""Exercise Gazebo -> ROS bridge -> virtual MTS160 in an isolated headless world.

Run from the workspace after sourcing ROS and install/setup.bash:
python3 ../scripts/check_gazebo_sensor.py
"""

import os
from pathlib import Path
import signal
import subprocess
import time
import uuid


def main():
    world = Path(__file__).resolve().parents[1] / 'ros2_ws/src/agv_sim/worlds/agv_world.sdf'
    os.environ['GZ_PARTITION'] = 'agv-sensor-check-' + uuid.uuid4().hex
    os.environ['ROS_DOMAIN_ID'] = str(20 + os.getpid() % 180)
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    import rclpy
    from naviq_msgs.msg import TrackDetection
    from tf2_msgs.msg import TFMessage

    processes = []

    def start(command):
        processes.append(subprocess.Popen(command, start_new_session=True,
                                          stdout=subprocess.DEVNULL))

    rclpy.init()
    node = rclpy.create_node('sensor_integration_check')
    received = []
    poses = []
    node.create_subscription(TrackDetection, '/mts160/track', received.append, 10)
    node.create_subscription(TFMessage, '/gazebo/ground_truth', poses.append, 10)

    def wait_for(predicate, timeout=15):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('A simulation process exited')
        raise AssertionError(f'Timeout: readings={received[-1:]}, ground_truth={poses[-1:]}')

    try:
        start(['gz', 'sim', '-s', '-r', str(world)])
        start(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
               '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
               '/model/agv/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
               '--ros-args', '-r', '/model/agv/pose:=/gazebo/ground_truth'])
        start(['ros2', 'run', 'agv_sim', 'gazebo_sensor', '--ros-args',
               '-p', 'use_sim_time:=true', '-p', f'world_file:={world}'])
        wait_for(lambda: received and received[-1].tape_detected)
        for x, y, expected in [(0, 0, 0), (0, 0.04, 40), (0, -0.04, -40),
                               (0, 0.12, None), (3.2, 0, None)]:
            received.clear()
            poses.clear()
            result = subprocess.run([
                'gz', 'service', '-s', '/world/agv_world/set_pose',
                '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000', '--req',
                f'name: "agv", position: {{x: {x}, y: {y}, z: 0.002}}, orientation: {{w: 1}}',
            ], capture_output=True, text=True, timeout=8, check=True)
            assert 'true' in result.stdout, result.stdout + result.stderr

            def correct():
                # Check the physical pose and several fresh readings after teleport.
                transforms = poses[-1].transforms if poses else []
                agv = next((t for t in transforms if t.child_frame_id == 'agv'), None)
                if agv is None or len(received) < 5:
                    return False
                p = agv.transform.translation
                if abs(p.x-x) > 0.005 or abs(p.y-y) > 0.005:
                    return False
                for reading in received[-5:]:
                    if expected is None:
                        if reading.tape_detected:
                            return False
                    elif (not reading.tape_detected or not reading.single_track
                          or abs(reading.left.position_mm-expected) > 2
                          or abs(reading.right.position_mm-expected) > 2):
                        return False
                return True

            wait_for(correct)
            print(f'PASS: world pose ({x}, {y}) -> offset {expected} mm', flush=True)
        print('PASS: actual Gazebo pose drives ROS magnetic readings', flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
