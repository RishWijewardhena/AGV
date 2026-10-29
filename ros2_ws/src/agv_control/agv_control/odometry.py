"""Differential-drive odometry from wheel encoder counts (no ROS)."""

import math


def count_delta(new, old):
    """Difference between two signed 32-bit counter values, across a wrap."""
    return (new - old + 2**31) % 2**32 - 2**31


class WheelOdometry:
    """Track the robot pose from the driver's absolute left and right counts."""

    def __init__(self, wheel_diameter=0.131, track_width=0.40, counts_per_rev=4096):
        self.metres_per_count = math.pi * wheel_diameter / counts_per_rev
        self.track_width = track_width
        self.x = self.y = self.th = 0.0
        self.distance = 0.0          # total path length driven (m), always growing
        self.last = None             # previous (left, right) counts

    def update(self, left, right):
        """Take the driver's absolute counts and advance the pose."""
        if self.last is None:        # first reading: nothing to compare yet
            self.last = (left, right)
            return

        d_left = count_delta(left, self.last[0]) * self.metres_per_count
        d_right = count_delta(right, self.last[1]) * self.metres_per_count
        self.last = (left, right)

        ds = (d_left + d_right) / 2                    # forward motion of base_link
        dth = (d_right - d_left) / self.track_width    # + = turn left (ROS convention)

        # use the heading halfway through the step: accurate on curves
        mid = self.th + dth / 2
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))  # keep in -pi..pi
        self.distance += abs(ds)
