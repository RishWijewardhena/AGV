"""Check the odometry maths against hand-computed poses."""

import math
import unittest

from agv_control.odometry import WheelOdometry, count_delta


def driven(*readings):
    """Odometry after the first reading (0, 0) and then each (left, right)."""
    odom = WheelOdometry()
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
        odom = WheelOdometry()
        odom.update(123456, -98765)
        self.assertEqual((odom.x, odom.y, odom.th, odom.distance), (0.0, 0.0, 0.0, 0.0))

    def test_counter_wrap(self):
        self.assertEqual(count_delta(-2147483600, 2147483600), 96)
        self.assertEqual(count_delta(2147483600, -2147483600), -96)


if __name__ == '__main__':
    unittest.main()
