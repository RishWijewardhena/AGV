"""Explore known tape graphs junction by junction and check the map that comes out."""

import json
import math
import os
import unittest

from agv_control.explorer import Explorer
from agv_control.map_builder import FLIP, SENSOR_X
from agv_control.route_planner import links, station_node
from agv_control.trip import parse_event


def station(code, x, y):
    return {'kind': 'station', 'code': code, 'point': [x, y]}


def junction(x, y):
    return {'kind': 'junction', 'point': [x, y]}


# Home 4 on the trunk of one Y, station 1 on branch L, station 2 on branch R.
Y = {'nodes': [station(4, 0, 0), junction(1, 0), station(1, 2, 1), station(2, 2, -1)],
     'edges': [{'ends': [[0, 'T'], [1, 'T']], 'length': 1.0},
               {'ends': [[1, 'L'], [2, 'T']], 'length': 1.4},
               {'ends': [[1, 'R'], [3, 'T']], 'length': 1.4}]}

TAUGHT = os.path.join(os.path.dirname(__file__), '..', 'config', 'site_map.json')


def explore(truth, home=4, limit=1000):
    """Drive the explorer over `truth`: forks ask choose(), merges and stations are forced.

    Every event is stamped with the sensor exactly on the true node. Returns (explorer, stops).
    """
    nodes = truth['nodes']
    graph = links(truth)
    explorer = Explorer(home)

    def arrive(i):
        x, y = nodes[i]['point']
        explorer.trip.move(x - SENSOR_X, y, 0.0)     # heading +x: the sensor lands on (x, y)

    def event(text):
        explorer.trip.record(parse_event(text))

    node = station_node(truth, home)
    arrive(node)
    assert not explorer.station(home)                  # driving out over the home tag
    leave, stops = 'T', []
    for _ in range(limit):
        (node, came_in, _), = graph[(node, leave)]
        arrive(node)
        if nodes[node]['kind'] == 'station':
            code = nodes[node]['code']
            assert explorer.station(code)
            stops.append(code)
            if code == home and explorer.finished():
                return explorer, stops
            event('turn')
            leave = 'T'
        elif came_in == 'T':
            side = explorer.choose()
            event(f'start,{side}')
            event(f'fork,{side}')
            leave = side
        else:
            side = FLIP[came_in]                       # merge: stay on the track we are on
            event(f'start,{side}')
            event(f'merge,{side}')
            leave = 'T'
    raise AssertionError('exploration did not finish')


def edge_set(site_map, index):
    """Edges as sets of (true node, port), so maps with other node numbers compare."""
    return {frozenset((index[a], a_port) for a, a_port in [tuple(end) for end in edge['ends']])
            for edge in site_map['edges']}


class ExploreTests(unittest.TestCase):

    def check_matches(self, truth, found):
        self.assertEqual(len(found['nodes']), len(truth['nodes']))
        index = {}                                     # found node -> true node, by position
        for i, node in enumerate(found['nodes']):
            index[i] = min(range(len(truth['nodes'])),
                           key=lambda k: math.dist(truth['nodes'][k]['point'], node['point']))
        self.assertEqual(sorted(index.values()), list(range(len(truth['nodes']))))
        self.assertEqual(edge_set(found, index), edge_set(truth, {i: i for i in index}))

    def test_y_visits_both_branches_then_stops_at_home(self):
        explorer, stops = explore(Y)
        self.assertEqual(stops, [2, 4, 1, 4])           # new R branch first, then L
        self.check_matches(Y, explorer.site_map())

    def test_taught_station_loop_is_explored_completely(self):
        with open(TAUGHT, encoding='utf-8') as f:
            truth = json.load(f)
        explorer, stops = explore(truth)
        self.assertEqual(stops[-1], 4)
        self.assertEqual(set(stops), {node['code'] for node in truth['nodes'] if node['kind'] == 'station'})
        self.assertEqual(explorer.progress()[1], 0)
        self.check_matches(truth, explorer.site_map())

    def test_not_finished_before_any_junction(self):
        explorer = Explorer()
        explorer.trip.move(0.0, 0.0, 0.0)
        self.assertFalse(explorer.station(4))
        self.assertFalse(explorer.finished())

    def test_choose_needs_odometry(self):
        with self.assertRaises(ValueError):
            Explorer().choose()


class TripTests(unittest.TestCase):

    def test_parse_event(self):
        self.assertEqual(parse_event('fork,R'), {'event': 'fork', 'side': 'R'})
        self.assertEqual(parse_event('turn'), {'event': 'turn'})

    def test_events_wait_for_odometry(self):
        explorer = Explorer()
        self.assertIsNone(explorer.trip.record({'event': 'turn'}))
        explorer.trip.move(0.0, 0.0, 0.0)
        explorer.trip.move(3.0, 4.0, 0.0)
        self.assertEqual(explorer.trip.record({'event': 'turn'})['distance'], 5.0)


if __name__ == '__main__':
    unittest.main()
