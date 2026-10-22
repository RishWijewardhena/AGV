"""Headless physical-turn check, isolated from any running user simulation."""

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time
import uuid

from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('world', nargs='?', type=Path, default=Path(__file__).resolve().parents[1]
                        / 'ros2_ws/src/agv_sim/worlds/agv_world.sdf')
    parser.add_argument('--angular-speed', type=float, default=0.4)
    args = parser.parse_args()
    env = dict(os.environ, GZ_PARTITION='agv-turn-check-' + uuid.uuid4().hex)
    os.environ['GZ_PARTITION'] = env['GZ_PARTITION']
    node = Node()
    publisher = node.advertise('/cmd_vel', Twist)

    def gz(*arguments):
        result = subprocess.run(['gz', *arguments], env=env, text=True,
                                capture_output=True, timeout=10, check=True)
        return result.stdout

    def sample(topic):
        return json.loads(gz('topic', '-e', '-t', topic, '-n', '1', '--json-output'))

    def stamp(message):
        value = message['header']['stamp']
        return float(value.get('sec', 0)) + float(value.get('nsec', 0)) * 1e-9

    def heading(pose):
        q = pose.get('orientation', {})
        x, y, z, w = (float(q.get(k, default)) for k, default in
                      [('x', 0), ('y', 0), ('z', 0), ('w', 1)])
        return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))

    def physical_pose():
        message = sample('/world/agv_world/pose/info')
        pose = next(p for p in message['pose'] if p.get('name') == 'agv')
        return stamp(message), pose

    server = subprocess.Popen(['gz', 'sim', '-s', '-r', str(args.world.resolve())],
                              env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                              text=True)
    try:
        time.sleep(2)
        if server.poll() is not None:
            raise RuntimeError(server.stderr.read())
        t0, p0 = physical_pose()
        odom0 = sample('/odom')
        command = Twist()
        command.angular.z = args.angular_speed
        for _ in range(60):
            publisher.publish(command)
            time.sleep(0.05)
        t1, p1 = physical_pose()
        odom = sample('/odom')
        publisher.publish(Twist())
        delta = math.atan2(math.sin(heading(p1)-heading(p0)),
                           math.cos(heading(p1)-heading(p0)))
        duration = t1 - t0
        # Compare with wheel rotation rather than wall time or command timing.
        expected = math.atan2(math.sin(heading(odom['pose'])-heading(odom0['pose'])),
                              math.cos(heading(odom['pose'])-heading(odom0['pose'])))
        a, b = p0.get('position', {}), p1.get('position', {})
        drift = math.hypot(float(b.get('x', 0))-float(a.get('x', 0)),
                           float(b.get('y', 0))-float(a.get('y', 0)))
        print(json.dumps({'simulation_seconds': duration, 'expected_yaw_rad': expected,
                          'actual_yaw_rad': delta, 'actual_over_expected': delta/expected,
                          'translation_m': drift, 'start_pose': p0, 'end_pose': p1,
                          'wheel_odometry_yaw_rad': heading(odom['pose'])}, indent=2))
        assert abs(expected) > 0.2, 'Insufficient rotation'
        assert 0.75 < delta/expected < 1.25, 'Chassis turning does not match the command'
        assert drift < 0.05, 'Excessive translation during in-place turn'
        print('PASS: physical heading follows the turning command')
    finally:
        server.terminate()
        try:
            _, errors = server.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            server.kill()
            _, errors = server.communicate()
        if errors:
            print(errors[-2000:])


if __name__ == '__main__':
    main()
