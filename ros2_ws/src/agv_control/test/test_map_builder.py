"""Check the map builder on small hand-made trips."""

import json
import math
import os
import tempfile
import unittest

from agv_control.map_builder import anchor, build, main, open_ports, passages, ports


def event(kind, distance, x, th=0.0, y=0.0, **extra):
    return dict(event=kind, distance=distance, x=x, y=y, th=th, **extra)


# Station 1 -> fork R -> station 2, spin, merge back -> station 1, spin.
# Base positions are chosen so every sensor point (0.25 m ahead) lands on
# x = 0 (station 1), x = 1 (the junction) or x = 2 (station 2).
OUT_AND_BACK = [
    event('station', 0.0, -0.25, code=1),
    event('start', 1.0, 0.75, side='R'),
    event('fork', 1.2, 0.95, side='R'),
    event('station', 2.0, 1.75, code=2),
    event('turn', 2.0, 1.75),
    event('start', 2.5, 1.25, math.pi, side='L'),
    event('merge', 2.7, 1.05, math.pi, side='L'),
    event('station', 3.5, 0.25, math.pi, code=1),
    event('turn', 3.5, 0.25, math.pi),
]


def map_of(trip, **kwargs):
    return build(passages(anchor(trip, code=1)), **kwargs)


class PortTests(unittest.TestCase):

    def test_fork_leaves_through_the_chosen_branch(self):
        self.assertEqual(ports({'kind': 'fork', 'side': 'L'}), ('T', 'L'))

    def test_merge_arrives_through_the_other_branch(self):
        self.assertEqual(ports({'kind': 'merge', 'side': 'L'}), ('R', 'T'))

    def test_station_has_one_end(self):
        self.assertEqual(ports({'kind': 'station', 'code': 1}), ('T', 'T'))


class PassageTests(unittest.TestCase):

    def test_junction_is_placed_at_its_start_event(self):
        p = passages(OUT_AND_BACK)
        self.assertEqual([x['kind'] for x in p],
                         ['station', 'fork', 'station', 'turn', 'merge', 'station', 'turn'])
        self.assertAlmostEqual(p[1]['point'][0], 1.0)
        self.assertEqual(p[1]['distance'], 1.0)

    def test_end_without_start_is_skipped(self):
        trip = [event('fork', 1.0, 0.0, side='R'), event('station', 2.0, 1.0, code=1)]
        self.assertEqual([x['kind'] for x in passages(trip)], ['station'])

    def test_side_spaces_are_stripped(self):
        trip = [event('start', 1.0, 0.0, side=' L'), event('fork', 1.2, 0.2, side=' L')]
        self.assertEqual(passages(trip)[0]['side'], 'L')


class AnchorTests(unittest.TestCase):

    def test_station_becomes_the_origin(self):
        p = passages(anchor(OUT_AND_BACK, code=1))
        self.assertAlmostEqual(p[0]['point'][0], 0.0)
        self.assertAlmostEqual(p[0]['point'][1], 0.0)

    def test_map_does_not_depend_on_where_odometry_started(self):
        ox, oy, oth = 1.0, -2.0, math.radians(40)
        moved = [dict(e, x=ox + e['x'] * math.cos(oth) - e['y'] * math.sin(oth),
                      y=oy + e['x'] * math.sin(oth) + e['y'] * math.cos(oth),
                      th=e['th'] + oth) for e in OUT_AND_BACK]
        for a, b in zip(map_of(OUT_AND_BACK)[0], map_of(moved)[0]):
            self.assertAlmostEqual(math.dist(a['point'], b['point']), 0.0)

    def test_missing_station_raises(self):
        with self.assertRaises(StopIteration):
            anchor(OUT_AND_BACK, code=9)


class BuildTests(unittest.TestCase):

    def test_out_and_back_gives_three_nodes_and_two_edges(self):
        nodes, edges = map_of(OUT_AND_BACK)
        self.assertEqual([n['kind'] for n in nodes], ['station', 'junction', 'station'])
        self.assertEqual(sorted((tuple(map(tuple, e['ends'])), e['length']) for e in edges),
                         [(((0, 'T'), (1, 'T')), 1.0), (((1, 'R'), (2, 'T')), 1.0)])

    def test_open_port_is_the_branch_not_driven(self):
        nodes, edges = map_of(OUT_AND_BACK)
        self.assertEqual(open_ports(nodes, edges), [(1, 'L')])

    def test_length_after_a_spin_is_not_used(self):
        # the spin swings the sensor back, so the 0.5 m measured after it is ignored
        _, edges = map_of(OUT_AND_BACK)
        spur = next(e for e in edges if e['ends'] == [[1, 'R'], [2, 'T']])
        self.assertEqual(spur['length'], 1.0)

    def test_unknown_junction_breaks_the_edge(self):
        trip = [event('station', 0.0, -0.25, code=1),
                event('start', 1.0, 0.75, side='R'),
                event('unknown', 1.2, 0.95, side='R'),
                event('station', 2.0, 1.75, code=2)]
        _, edges = map_of(trip)
        self.assertEqual(edges, [])

    def test_junctions_further_apart_than_tolerance_stay_separate(self):
        trip = [event('station', 0.0, -0.25, code=1),
                event('start', 1.0, 0.75, side='R'), event('fork', 1.2, 0.95, side='R'),
                event('start', 1.5, 1.25, side='L'), event('merge', 1.7, 1.45, side='L')]
        self.assertEqual(len(map_of(trip)[0]), 3)
        self.assertEqual(len(map_of(trip, tolerance=0.6)[0]), 2)


class CommandTests(unittest.TestCase):

    def test_writes_the_map(self):
        with tempfile.TemporaryDirectory() as folder:
            trip_file = os.path.join(folder, 'trip.json')
            map_file = os.path.join(folder, 'map.json')
            with open(trip_file, 'w', encoding='utf-8') as f:
                json.dump(OUT_AND_BACK, f)
            main([trip_file, '-o', map_file, '--anchor', '1'])
            with open(map_file, encoding='utf-8') as f:
                result = json.load(f)
        self.assertEqual(result['anchor'], 1)
        self.assertEqual(len(result['nodes']), 3)
        self.assertEqual(result['nodes'][1]['point'], [1.0, 0.0])

    def test_missing_anchor_exits_with_an_error(self):
        with tempfile.TemporaryDirectory() as folder:
            trip_file = os.path.join(folder, 'trip.json')
            with open(trip_file, 'w', encoding='utf-8') as f:
                json.dump(OUT_AND_BACK, f)
            with self.assertRaises(SystemExit) as stop:
                main([trip_file, '--anchor', '9'])
        self.assertEqual(stop.exception.code, 1)


if __name__ == '__main__':
    unittest.main()
