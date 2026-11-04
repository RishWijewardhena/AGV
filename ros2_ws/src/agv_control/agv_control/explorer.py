"""Choose fork sides that drive every piece of tape, and map it on the way (no ROS)."""

from agv_control import map_builder, route_planner
from agv_control.trip import Trip


class Explorer:
    """Explore from the home station, turning around at every station tag."""

    def __init__(self, home=4, tolerance=0.3):
        self.home = home              # Navicode of the station the trip starts and ends at
        self.tolerance = tolerance    # m; junction grouping distance, as in build_map
        self.trip = Trip()

    def graph(self, here=None):
        """Nodes and edges driven so far; `here` adds the junction the robot is entering."""
        passage_list = map_builder.passages(self.trip.log)
        if here is not None:
            passage_list.append(here)
        return map_builder.build(passage_list, self.tolerance)

    def choose(self):
        """Side to take at the fork the robot has just reached: tape not driven yet if possible."""
        if self.trip.pose is None:
            raise ValueError('no odometry yet')
        x, y, th = self.trip.pose
        here = {'kind': 'fork', 'side': 'R', 'distance': self.trip.distance,
                'point': map_builder.sensor_point({'x': x, 'y': y, 'th': th})}
        nodes, edges = self.graph(here)
        node = map_builder.find_node(nodes, here, self.tolerance)
        unexplored = set(map_builder.open_ports(nodes, edges))
        for side in ('R', 'L'):
            if (node, side) in unexplored:
                return side

        site_map = {'nodes': nodes, 'edges': edges}
        if unexplored:              # drive to the nearest junction end not driven yet
            def is_goal(i, came_in):
                return any((i, port) in unexplored
                           for port in route_planner.leaves(nodes[i], came_in))
        else:                       # everything driven: go home
            home = route_planner.station_node(site_map, self.home)

            def is_goal(i, came_in):
                return i == home
        route = route_planner.search(site_map, node, 'T', is_goal, turn_at_stations=True)
        if route is None:
            raise ValueError('no driven route to the tape still to explore')
        return route[0][0]

    def station(self, code):
        """Record a station tag; True if the robot should turn around there.

        Every tag but the first is a dead end: the first is the home tag driven out over at the start.
        """
        first = not any(event['event'] == 'station' for event in self.trip.log)
        self.trip.record({'event': 'station', 'code': code})
        return not first

    def progress(self):
        """(nodes found, junction ends not driven yet)."""
        nodes, edges = self.graph()
        return len(nodes), len(map_builder.open_ports(nodes, edges))

    def finished(self):
        """Tell whether a junction was found and every junction end has been driven."""
        nodes, edges = self.graph()
        return (any(node['kind'] == 'junction' for node in nodes)
                and not map_builder.open_ports(nodes, edges))

    def site_map(self):
        """Return the explored map, anchored at the home station like build_map's output."""
        trip = map_builder.anchor(self.trip.log, self.home)
        return map_builder.to_site_map(map_builder.passages(trip), self.home, self.tolerance)
