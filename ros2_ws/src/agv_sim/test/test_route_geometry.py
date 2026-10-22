"""Keep shared route configuration consistent with the reference simulated layout."""

import unittest

from agv_control.mission_planner import load_route_map
from agv_sim.tape_map import SPURS, loop_positions


class RouteGeometryTests(unittest.TestCase):
    def test_reference_layout_matches_shared_station_configuration(self):
        route_map = load_route_map()
        positions, length = loop_positions()
        self.assertAlmostEqual(route_map['loop_length_m'], length)
        self.assertEqual([(station['name'], station['navicode'])
                          for station in route_map['stations']],
                         [(name, code) for name, code, *_ in SPURS])
        for station in route_map['stations']:
            self.assertAlmostEqual(station['loop_position_m'], positions[station['name']])
