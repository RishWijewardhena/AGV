"""Build a tape map (junctions, stations, edges) from a recorded trip (no ROS)."""

import json
import math
import os

SENSOR_X = 0.25      # m from the drive axle to the MTS160
FLIP = {'L': 'R', 'R': 'L'}


def ports(passage):
    """(port the robot came in through, port it left through) at this node."""
    if passage['kind'] == 'fork':
        return 'T', passage['side']
    if passage['kind'] == 'merge':
        return FLIP[passage['side']], 'T'
    return 'T', 'T'                           # a station has a single end


def find_node(nodes, passage, tolerance):
    """Index of the node this passage belongs to; adds a new node if none matches."""
    for i, node in enumerate(nodes):
        if passage['kind'] == 'station':
            if node['kind'] == 'station' and node['code'] == passage['code']:
                return i
        elif node['kind'] == 'junction' and math.dist(node['point'], passage['point']) < tolerance:
            return i
    node = {'kind': 'station' if passage['kind'] == 'station' else 'junction',
            'point': passage['point']}
    if passage['kind'] == 'station':
        node['code'] = passage['code']
    nodes.append(node)
    return len(nodes) - 1


def build(passage_list, tolerance=0.3):
    """Nodes and edges from the output of passages()."""
    nodes = []
    lengths = {}         # ((node, port), (node, port)) -> lengths measured for that edge
    last = None          # (node, port it left through, distance) of the previous node
    turned = False       # a spin since the last node: the next length is not valid
    for passage in passage_list:
        if passage['kind'] == 'turn':
            turned = True
            continue
        if passage['kind'] in ('unknown', 'gap'):
            last = None                       # unsure which ports, or a new trip: no edge
            continue
        node = find_node(nodes, passage, tolerance)
        came_in, leaves = ports(passage)
        if last is not None:
            a, a_port, a_distance = last
            key = tuple(sorted([(a, a_port), (node, came_in)]))
            lengths.setdefault(key, [])
            if not turned:
                lengths[key].append(passage['distance'] - a_distance)
        last = (node, leaves, passage['distance'])
        turned = False
    edges = [{'ends': [list(a), list(b)], 'length': round(sum(d) / len(d), 3) if d else None}
             for (a, b), d in lengths.items()]
    return nodes, edges


def sensor_point(event, sensor_x=SENSOR_X):
    """Return the sensor position for an event stamped at ``base_link``.

    ``sensor_x`` is measured forward along the robot heading, so the offset
    must be rotated by ``th`` before it is added to the recorded pose.
    """
    return (event['x'] + sensor_x * math.cos(event['th']),
            event['y'] + sensor_x * math.sin(event['th']))


def anchor(trip, code=4):
    """Express the trip relative to the first pass over station `code`.

    The station is at (0, 0) and the robot's heading there is +x.
    """
    ref = next(e for e in trip if e['event'] == 'station' and e['code'] == code)
    ax, ay = sensor_point(ref)
    c, s = math.cos(ref['th']), math.sin(ref['th'])
    out = []
    for event in trip:
        event = dict(event)                   # copy, so the loaded trip stays unchanged
        if 'x' in event:
            dx, dy = event['x'] - ax, event['y'] - ay
            event['x'] = dx * c + dy * s      # rotate by -heading
            event['y'] = -dx * s + dy * c
            event['th'] = math.atan2(math.sin(event['th'] - ref['th']),
                                     math.cos(event['th'] - ref['th']))
        out.append(event)
    return out


def passages(trip):
    """One entry per junction, station or turn the robot passed, in order."""
    out = []
    start = None
    for event in trip:
        kind = event['event']
        if kind == 'start':
            # Place a junction at the first detection, then wait for its
            # matching end event to determine its kind and side.
            start = event
        elif kind in ('fork', 'merge', 'unknown'):
            # An unmatched end means recording began after the junction had
            # already started, so there is no reliable point to add.
            if start is None:
                continue
            out.append({'kind': kind, 'side': event.get('side', '').strip(),
                        'point': sensor_point(start), 'distance': start['distance']})
            start = None
        elif kind == 'station':
            # Stations are point events and carry their identifier directly.
            out.append({'kind': 'station', 'code': event['code'],
                        'point': sensor_point(event), 'distance': event['distance']})
        elif kind == 'turn':
            # Turns are retained as distance markers; they have no map point
            # until the surrounding geometry is interpreted.
            out.append({'kind': 'turn', 'distance': event['distance']})
    return out


def open_ports(nodes, edges):
    """Junction ends that no edge uses yet: tape that has not been taught."""
    used = {tuple(end) for edge in edges for end in edge['ends']}
    return [(i, port) for i, node in enumerate(nodes) if node['kind'] == 'junction'
            for port in ('T', 'L', 'R') if (i, port) not in used]


def to_site_map(passage_list, anchor_code, tolerance=0.3):
    """Return the map JSON shared by build_map and the explorer, with points rounded to mm."""
    nodes, edges = build(passage_list, tolerance)
    return {'anchor': anchor_code,
            'nodes': [dict(node, point=[round(v, 3) for v in node['point']]) for node in nodes],
            'edges': edges}


def write_map(path, site_map):
    """Save a map JSON, creating its folder; returns the absolute path."""
    path = os.path.abspath(os.path.expanduser(path))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(site_map, f, indent=1)
    return path


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description='Build a tape map from a recorded trip.')
    parser.add_argument('trips', nargs='+', help='trip JSON files written by tape_recorder')
    parser.add_argument('-o', '--output', help='map JSON to write (default: only print)')
    parser.add_argument('--anchor', type=int, default=4, help='Navicode of the reference station')
    parser.add_argument('--tolerance', type=float, default=0.3, help='m; junction grouping distance')
    args = parser.parse_args(argv)

    passage_list = []
    for name in args.trips:
        with open(name, encoding='utf-8') as f:
            trip = json.load(f)
        try:
            trip = anchor(trip, args.anchor)
        except StopIteration:
            parser.exit(1, f'{name} never passed station {args.anchor}\n')
        passage_list += [{'kind': 'gap'}] + passages(trip)
    site_map = to_site_map(passage_list, args.anchor, args.tolerance)
    nodes, edges = site_map['nodes'], site_map['edges']

    for i, node in enumerate(nodes):
        name = f"station {node['code']}" if node['kind'] == 'station' else 'junction'
        print(f'{i:2}  {name:10} ({node["point"][0]:6.2f}, {node["point"][1]:6.2f})')
    for edge in edges:
        (a, a_port), (b, b_port) = edge['ends']
        print(f'    {a}{a_port} - {b}{b_port}  {edge["length"]} m')
    missing = open_ports(nodes, edges)
    print('open ports (not taught yet):', ', '.join(f'{i}{p}' for i, p in missing) or 'none')

    if args.output:
        print('saved', write_map(args.output, site_map))
