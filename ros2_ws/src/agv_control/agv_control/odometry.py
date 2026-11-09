"""Differential-drive odometry from wheel encoder counts (no ROS)."""

import math


def count_delta(new, old):
    """Difference between two signed 32-bit counter values, across a wrap."""
    return (new - old + 2**31) % 2**32 - 2**31


class WheelOdometry:
    """Track the robot pose from the driver's absolute left and right counts."""

    def __init__(self, wheel_diameter, track_width, counts_per_rev):
        self.metres_per_count = math.pi * wheel_diameter / counts_per_rev
        self.track_width = track_width
        self.x = self.y = self.th = 0.0
        self.distance = 0.0          # total path length driven (m), always growing
        self.last = None             # previous (left, right) counts

    def update(self, left, right, dth=None):
        """Take the driver's absolute counts and advance the pose.

        ``dth`` is the heading change since the last update from a gyro; without it the
        heading comes from the difference between the wheels.
        """
        if self.last is None:        # first reading: nothing to compare yet
            self.last = (left, right)
            return

        d_left = count_delta(left, self.last[0]) * self.metres_per_count
        d_right = count_delta(right, self.last[1]) * self.metres_per_count
        self.last = (left, right)

        ds = (d_left + d_right) / 2                    # forward motion of base_link
        if dth is None:
            dth = (d_right - d_left) / self.track_width    # + = turn left (ROS convention)

        # use the heading halfway through the step: accurate on curves
        mid = self.th + dth / 2
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))  # keep in -pi..pi
        self.distance += abs(ds)


class GyroYaw:
    """Heading change from a z-axis gyro, with its bias learnt while the wheels stand still.

    Each take() covers the gyro samples since the previous one; the caller says whether the
    wheels stood still in that time. Still intervals teach the bias (a plain mean for the first
    ``tau`` seconds, then a running average over ``tau``) and turn the robot by nothing.
    ``ready`` once ``settle`` seconds of still samples are in.
    """

    def __init__(self, settle=1.0, tau=5.0):
        self.settle, self.tau = settle, tau
        self.bias = 0.0
        self.learnt = 0.0            # seconds of still samples so far
        self.integral = 0.0          # raw rate integrated since the last take() (rad)
        self.span = 0.0              # time covered by that integral (s)
        self.last = None             # previous (time, rate)

    @property
    def ready(self):
        return self.learnt >= self.settle

    def add(self, rate, t):
        """One gyro sample: z rate (rad/s) at time t (s)."""
        if self.last is not None:
            dt = t - self.last[0]
            if 0 < dt < 0.1:         # skip gaps and clock jumps
                self.integral += (self.last[1] + rate) / 2 * dt
                self.span += dt
        self.last = (t, rate)

    def take(self, still):
        """Heading change (rad) since the last call; ``still``: the wheels did not move."""
        integral, span = self.integral, self.span
        self.integral = self.span = 0.0
        if not still:
            return integral - self.bias * span
        if span > 0:
            self.learnt += span
            self.bias += (integral / span - self.bias) * span / min(self.learnt, self.tau)
        return 0.0
