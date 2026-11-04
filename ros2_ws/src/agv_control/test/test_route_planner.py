"""Check the route planner on small hand-made maps."""

import unittest

from agv_control.route_planner import leg, plan_mission, side_at


def station(code):
    return {'kind': 'station', 'code': code, 'point': [0, 0]}


JUNCTION = {'kind': 'junction', 'point': [0, 0]}

# One Y: station 1 on the trunk, station 2 on branch L, station 3 on branch R.
#        2     3
#         L   R
#          \ /
#           J
#           | T
#           1
Y = {'nodes': [station(1), JUNCTION, station(2), station(3)],
     'edges': [{'ends': [[0, 'T'], [1, 'T']], 'length': 1.0},
               {'ends': [[1, 'L'], [2, 'T']], 'length': 2.0},
               {'ends': [[1, 'R'], [3, 'T']], 'length': 3.0}]}


def spur_loop(bypass):
    """Station 1 and a loop junction pair (J1, J2) with station 2 on a spur between them.

    1 - A ... J1 (L) arc -> B -> 2, and J2 (R) arc <- B; J1 (R) -- bypass -- J2 (L)
    """
    nodes = [station(1), dict(JUNCTION), dict(JUNCTION), dict(JUNCTION), station(2)]
    edges = [{'ends': [[0, 'T'], [1, 'T']], 'length': 1.0},      # 1 -> J1 trunk
             {'ends': [[1, 'L'], [3, 'R']], 'length': 0.8},      # J1 arc -> B
             {'ends': [[3, 'T'], [4, 'T']], 'length': 1.2},      # B stem -> 2
             {'ends': [[3, 'L'], [2, 'R']], 'length': 0.8}]      # B arc -> J2
    if bypass:
        edges.append({'ends': [[1, 'R'], [2, 'L']], 'length': 1.0})
    return {'nodes': nodes, 'edges': edges}


class SideTests(unittest.TestCase):

    def test_fork_takes_the_branch(self):
        self.assertEqual(side_at('T', 'L'), 'L')

    def test_merge_flips_the_branch(self):
        self.assertEqual(side_at('L', 'T'), 'R')


class LegTests(unittest.TestCase):

    def test_trunk_to_branch_is_a_fork(self):
        self.assertEqual(leg(Y, 1, 2), (['L'], 3.0))
        self.assertEqual(leg(Y, 1, 3), (['R'], 4.0))

    def test_branch_to_trunk_is_a_flipped_merge(self):
        self.assertEqual(leg(Y, 2, 1), (['R'], 3.0))

    def test_branch_to_branch_is_impossible(self):
        with self.assertRaises(ValueError):
            leg(Y, 2, 3)

    def test_route_does_not_pass_through_a_station(self):
        # 2 -> 3 is impossible even though 1 is reachable: the robot only turns at its goal
        with self.assertRaises(ValueError):
            leg(Y, 3, 2)

    def test_unknown_station(self):
        with self.assertRaises(ValueError):
            leg(Y, 1, 9)

    def test_shortest_way_is_chosen(self):
        m = spur_loop(bypass=True)
        m['nodes'].append(station(3))
        m['edges'].append({'ends': [[2, 'T'], [5, 'T']], 'length': 1.0})   # past J2 to station 3
        sides, metres = leg(m, 1, 3)
        self.assertEqual(sides, ['R', 'R'])             # J1 fork R onto the bypass, J2 merge
        self.assertAlmostEqual(metres, 3.0)

    def test_without_the_bypass_there_is_no_way_past(self):
        m = spur_loop(bypass=False)
        m['nodes'].append(station(3))
        m['edges'].append({'ends': [[2, 'T'], [5, 'T']], 'length': 1.0})
        with self.assertRaises(ValueError):
            leg(m, 1, 3)


class MissionTests(unittest.TestCase):

    def test_out_and_back(self):
        self.assertEqual(plan_mission(Y, [2], home=1), (['L', 'R'], [2, 1]))

    def test_same_station_twice_is_refused(self):
        with self.assertRaises(ValueError):
            plan_mission(Y, [1], home=1)


if __name__ == '__main__':
    unittest.main()
