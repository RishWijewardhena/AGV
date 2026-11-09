"""Check the odometry maths against hand-computed poses."""

import math
import unittest

from agv_control.odometry import GyroYaw, WheelOdometry, count_delta

ROBOT = (0.131, 0.40, 4096)   # wheel diameter, track width, counts per rev used by the expected poses


def driven(*readings):
    """Odometry after the first reading (0, 0) and then each (left, right)."""
    odom = WheelOdometry(*ROBOT)
    odom.update(0, 0)
    for left, right in readings:
        odom.update(left, right)
    return odom


class OdometryTests(unittest.TestCase):

    def test_straight_one_revolution(self):
        odom = driven((4096, 4096))
        self.assertAlmostEqual(odom.x, 0.41155, places=4)
        self.assertAlmostEqual(odom.y, 0.0, places=4)
        self.assertAlmostEqual(odom.th, 0.0, places=4)
        self.assertAlmostEqual(odom.distance, 0.41155, places=4)

    def test_spin_in_place(self):
        odom = driven((-1000, 1000))
        self.assertAlmostEqual(odom.x, 0.0, places=4)
        self.assertAlmostEqual(odom.y, 0.0, places=4)
        self.assertAlmostEqual(odom.th, 0.50238, places=4)
        self.assertAlmostEqual(odom.distance, 0.0, places=4)

    def test_one_wheel_single_step(self):
        odom = driven((0, 4096))
        self.assertAlmostEqual(odom.x, 0.17914, places=4)
        self.assertAlmostEqual(odom.y, 0.10125, places=4)
        self.assertAlmostEqual(odom.th, 1.02887, places=4)

    def test_one_wheel_small_steps_match_the_arc(self):
        odom = driven(*[(0, 4096 * i / 100) for i in range(1, 101)])
        self.assertAlmostEqual(odom.x, 0.2 * math.sin(1.02887), places=4)
        self.assertAlmostEqual(odom.y, 0.2 * (1 - math.cos(1.02887)), places=4)
        self.assertAlmostEqual(odom.th, 1.02887, places=4)

    def test_first_reading_does_not_move(self):
        odom = WheelOdometry(*ROBOT)
        odom.update(123456, -98765)
        self.assertEqual((odom.x, odom.y, odom.th, odom.distance), (0.0, 0.0, 0.0, 0.0))

    def test_counter_wrap(self):
        self.assertEqual(count_delta(-2147483600, 2147483600), 96)
        self.assertEqual(count_delta(2147483600, -2147483600), -96)


def gyro_run(gyro, rate, seconds, still, t0=0.0, hz=100):
    """Feed a constant rate for `seconds`, taking every 2 samples (50 Hz wheels); summed turn."""
    turned, t = 0.0, t0
    for i in range(int(seconds * hz)):
        t = t0 + i / hz
        gyro.add(rate, t)
        if i % 2:
            turned += gyro.take(still)
    return turned, t


class GyroYawTests(unittest.TestCase):

    def test_bias_learnt_while_still_and_removed_when_moving(self):
        gyro = GyroYaw()
        self.assertFalse(gyro.ready)
        turned, t = gyro_run(gyro, 0.05, 2.0, still=True)
        self.assertEqual(turned, 0.0)                 # still: no heading change
        self.assertTrue(gyro.ready)
        self.assertAlmostEqual(gyro.bias, 0.05)
        turned, _ = gyro_run(gyro, 0.05 + 0.5, 2.0, still=False, t0=t + 0.01)
        self.assertAlmostEqual(turned, 0.5 * 2.0, delta=0.01)

    def test_moving_samples_do_not_teach_the_bias(self):
        gyro = GyroYaw()
        gyro_run(gyro, 0.6, 1.0, still=False)
        self.assertEqual(gyro.bias, 0.0)
        self.assertFalse(gyro.ready)


class GyroHeadingTests(unittest.TestCase):

    def test_heading_change_from_gyro_replaces_the_wheels(self):
        odom = driven()
        odom.update(4096, 4096, dth=math.pi / 2)      # straight wheel counts, gyro says 90 deg
        self.assertAlmostEqual(odom.th, math.pi / 2)
        self.assertAlmostEqual(odom.x, 0.131 * math.pi * math.cos(math.pi / 4))


if __name__ == '__main__':
    unittest.main()
