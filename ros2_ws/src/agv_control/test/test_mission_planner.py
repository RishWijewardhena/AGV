"""Preserve route choices while moving planning out of the simulator."""

from pathlib import Path
import unittest

from agv_control.mission_planner import leg, load_route_map, plan_mission


CONFIG = Path(__file__).resolve().parents[1] / 'config/station_loop.json'
LEGS = {
    ('START', 'S3'): 'RRRR', ('START', 'S2'): 'LLRRLL',
    ('START', 'S1'): 'LLLL', ('S3', 'START'): 'LLLL',
    ('S3', 'S2'): 'RRRR', ('S3', 'S1'): 'LLRRLL',
    ('S2', 'START'): 'RRLLRR', ('S2', 'S3'): 'LLLL',
    ('S2', 'S1'): 'RRRR', ('S1', 'START'): 'RRRR',
    ('S1', 'S3'): 'RRLLRR', ('S1', 'S2'): 'LLLL',
}


class MissionPlannerTests(unittest.TestCase):
    def setUp(self):
        self.route_map = load_route_map(CONFIG)

    def test_every_leg_matches_original_simulation(self):
        for stations, expected in LEGS.items():
            with self.subTest(stations=stations):
                self.assertEqual(''.join(leg(*stations, self.route_map)), expected)

    def test_multi_station_mission_returns_to_start(self):
        route, stops = plan_mission(['S1', 'S3'], self.route_map)
        self.assertEqual(''.join(route), 'LLLLRRLLRRLLLL')
        self.assertEqual(stops, [1, 3, 4])

    def test_repeated_station_is_rejected(self):
        with self.assertRaises(ValueError):
            plan_mission(['S1', 'S1'], self.route_map)


if __name__ == '__main__':
    unittest.main()
