"""Station-loop routing from configuration, without simulated tape geometry."""

import json
import math
from pathlib import Path


def load_route_map(path=None):
    """Load station codes and counter-clockwise loop positions in metres."""
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('agv_control')) / 'config/station_loop.json'
    with open(path, encoding='utf-8') as source:
        route_map = json.load(source)
    length = route_map['loop_length_m']
    stations = route_map['stations']
    names = [station['name'] for station in stations]
    codes = [station['navicode'] for station in stations]
    if not math.isfinite(length) or length <= 0:
        raise ValueError('loop_length_m must be positive and finite')
    if len(names) != len(set(names)) or 'START' not in names:
        raise ValueError('Station names must be unique and include START')
    if len(codes) != len(set(codes)):
        raise ValueError('Navicodes must be unique')
    if any(not 0 <= station['loop_position_m'] < length for station in stations):
        raise ValueError('Station loop positions must be within the loop length')
    return route_map


def leg(a, b, route_map=None):
    """Select the shorter direction between two stations; preserve L/R choices."""
    if route_map is None:
        route_map = load_route_map()
    stations = route_map['stations']
    order = [station['name'] for station in stations]
    positions = {station['name']: station['loop_position_m'] for station in stations}
    length = route_map['loop_length_m']
    ccw = (positions[b] - positions[a]) % length
    if ccw <= length - ccw:
        passed = (order.index(b) - order.index(a) - 1) % len(order)
        return ['R', 'R'] + ['L', 'L'] * passed + ['R', 'R']
    passed = (order.index(a) - order.index(b) - 1) % len(order)
    return ['L', 'L'] + ['R', 'R'] * passed + ['L', 'L']


def plan_mission(targets, route_map=None):
    """Return (junction choices, stop codes) for START -> targets -> START."""
    if route_map is None:
        route_map = load_route_map()
    codes = {station['name']: station['navicode'] for station in route_map['stations']}
    visits = ['START'] + list(targets) + ['START']
    route = []
    for a, b in zip(visits[:-1], visits[1:]):
        if a == b:
            raise ValueError(f'{a} twice in a row')
        route += leg(a, b, route_map)
    return route, [codes[name] for name in visits[1:]]
