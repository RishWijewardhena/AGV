"""Plan routes on a taught tape map from build_map (no ROS)."""

import heapq
import json

FLIP = {'L': 'R', 'R': 'L'}


def load_map(path):
    """Read a map JSON written by build_map."""
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def links(site_map):
    """(node, port) -> [(other node, its port, length)] reached by driving out of that port."""
    out = {}
    for edge in site_map['edges']:
        (a, a_port), (b, b_port) = edge['ends']
        out.setdefault((a, a_port), []).append((b, b_port, edge['length']))
        out.setdefault((b, b_port), []).append((a, a_port, edge['length']))
    return out


def station_node(site_map, code):
    """Index of the station node with this Navicode."""
    for i, node in enumerate(site_map['nodes']):
        if node['kind'] == 'station' and node['code'] == code:
            return i
    raise ValueError(f'station {code} is not on the map')


def side_at(came_in, leaves):
    """The follower's side at a junction: fork -> branch taken, merge -> flipped branch."""
    return leaves if came_in == 'T' else FLIP[came_in]


def leaves(node, came_in, turn_at_stations=False):
    """List the ports to drive out of after entering `node` through `came_in` (None: starts there)."""
    if node['kind'] == 'station':
        return ['T'] if came_in is None or turn_at_stations else []
    return ['L', 'R'] if came_in == 'T' else ['T']    # never branch to branch


def search(site_map, start, came_in, is_goal, turn_at_stations=False):
    """Shortest drive from `start` to the first (node, port it came in through) where is_goal holds.

    Returns (sides, metres) with one 'L' or 'R' per junction passed, or None if there is no route.
    """
    nodes = site_map['nodes']
    graph = links(site_map)
    queue = [(0.0, start, came_in, [])]      # (metres, node, port it came in through, sides)
    done = set()
    while queue:
        metres, node, came_in, sides = heapq.heappop(queue)
        if (node, came_in) in done:
            continue
        done.add((node, came_in))
        if is_goal(node, came_in):
            return sides, metres
        for port in leaves(nodes[node], came_in, turn_at_stations):
            side = [side_at(came_in, port)] if nodes[node]['kind'] == 'junction' else []
            for other, other_port, length in graph.get((node, port), []):
                if length is not None:
                    heapq.heappush(queue, (metres + length, other, other_port, sides + side))
    return None


def leg(site_map, start_code, goal_code):
    """Shortest drive between two stations, leaving the start along its stem.

    Returns (sides, metres): one 'L' or 'R' per junction passed.
    """
    goal = station_node(site_map, goal_code)
    route = search(site_map, station_node(site_map, start_code), None,
                   lambda node, came_in: node == goal and came_in is not None)
    if route is None:
        raise ValueError(f'no route from station {start_code} to station {goal_code}')
    return route


def plan_mission(site_map, targets, home):
    """(junction sides, stop codes) for home -> targets -> home, all as Navicodes."""
    visits = [home] + list(targets) + [home]
    route = []
    for a, b in zip(visits[:-1], visits[1:]):
        if a == b:
            raise ValueError(f'station {a} twice in a row')
        route += leg(site_map, a, b)[0]
    return route, visits[1:]
