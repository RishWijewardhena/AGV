"""Trip log shared by tape_recorder and the explorer: events stamped with wheel odometry (no ROS)."""

import math


def yaw(q):
    """Heading (rad) of a quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def parse_event(text):
    """'fork,R' -> {'event': 'fork', 'side': 'R'};  'turn' -> {'event': 'turn'}."""
    kind, _, side = text.partition(',')
    entry = {'event': kind}
    if side:
        entry['side'] = side
    return entry


class Trip:
    """Latest pose, path length driven and the events recorded so far."""

    def __init__(self):
        self.pose = None             # latest (x, y, th) from wheel odometry
        self.distance = 0.0          # path length driven (m)
        self.log = []                # recorded events, in order

    def move(self, x, y, th):
        if self.pose is not None:    # add the step since the last pose
            self.distance += math.hypot(x - self.pose[0], y - self.pose[1])
        self.pose = (x, y, th)

    def record(self, entry):
        """Stamp the entry with the current pose and keep it; None before any odometry."""
        if self.pose is None:
            return None
        x, y, th = self.pose
        entry.update(distance=round(self.distance, 3),
                     x=round(x, 3), y=round(y, 3), th=round(th, 3))
        self.log.append(entry)
        return entry
