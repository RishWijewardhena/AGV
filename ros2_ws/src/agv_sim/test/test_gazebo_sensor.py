"""Check the virtual sensor against the actual straight-strip world geometry."""

import math
from pathlib import Path
import unittest

from agv_sim.gazebo_sensor_geometry import StraightTapeSensor


WORLD = Path(__file__).resolve().parents[1] / 'worlds/agv_world.sdf'


class StraightTapeTests(unittest.TestCase):
    def setUp(self):
        self.sensor = StraightTapeSensor(WORLD)

    def reading(self, x=0, y=0, z=0, yaw=0):
        return self.sensor.read(x, y, z, 0, 0, math.sin(yaw/2), math.cos(yaw/2))

    def test_centred_on_the_visible_strip(self):
        self.assertEqual(self.reading(), (0, 0, 0, 0))

    def test_robot_left_of_tape_reports_tape_to_its_right(self):
        self.assertEqual(self.reading(y=0.04), (40, 0, 40, 0))

    def test_robot_right_of_tape_reports_tape_to_its_left(self):
        self.assertEqual(self.reading(y=-0.04), (-40, 0, -40, 0))

    def test_heading_includes_the_forward_sensor_offset(self):
        self.assertEqual(self.reading(yaw=math.radians(5)), (22, 5, 22, 5))

    def test_tape_outside_sensor_width_is_lost(self):
        self.assertIsNone(self.reading(y=0.10))

    def test_no_detection_past_the_end_of_the_visible_strip(self):
        self.assertIsNone(self.reading(x=3.1))

    def test_no_detection_when_sensor_is_lifted_too_high(self):
        self.assertIsNone(self.reading(z=0.10))

    def test_no_detection_when_sensor_is_below_tape(self):
        self.assertIsNone(self.reading(z=-0.04))

    def test_model_clearance_matches_tape_and_mount(self):
        self.assertAlmostEqual(self.sensor.sensor_bottom, 0.028)
        self.assertAlmostEqual(self.sensor.tape_top, 0.0013)

    def test_reading_works_when_driving_in_the_reverse_direction(self):
        self.assertEqual(self.reading(yaw=math.pi), (0, 0, 0, 0))

    def test_no_detection_when_sensor_is_tilted_on_its_side(self):
        self.assertIsNone(self.sensor.read(0, 0, 0, math.sin(math.pi/4), 0, 0,
                                          math.cos(math.pi/4)))

    def test_invalid_orientation_is_not_a_valid_sensor_reading(self):
        self.assertIsNone(self.sensor.read(0, 0, 0, 0, 0, 0, 0))


if __name__ == '__main__':
    unittest.main()
