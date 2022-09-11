import osmnx
import networkx


class DMA:
    def __init__(self, place, street_type='all_private'):
        # create a graph from a special place
        self.graph_map = osmnx.graph_from_place(query=place, network_type=street_type)
        # add speed and travel time to graph for weighted graph
        self.graph_map = osmnx.add_edge_speeds(G=self.graph_map)
        self.graph_map = osmnx.add_edge_travel_times(G=self.graph_map)
        # define basic nodes of map
        self.nodes = []

    # add nodes to self.nodes
    def add_nodes(self, nodes):
        self.nodes += nodes
        return self.nodes

    # remove nodes to self.nodes
    def remove_nodes(self, nodes):
        for node in nodes:
            try:
                self.nodes.remove(node)
            except ValueError:
                continue

    # crate a weighted graph of self.nodes
    def graph_nodes(self):
        # create an empty graph
        graph_node = networkx.Graph()
        # for loop for calculate length and travel time between all pairs ==> create a weighted graph of self.nodes
        for start in self.nodes:
            # convert node start to a node in graph_map
            node_start = osmnx.nearest_nodes(G=self.graph_map, X=start[1], Y=start[0])
            for end in self.nodes:
                # convert node start to a node in graph_map
                node_end = osmnx.nearest_nodes(G=self.graph_map, X=end[1], Y=end[0])
                # check for don't calculate weight between start and start
                if node_end != node_start:
                    # calculate length between start and end
                    length = networkx.shortest_path_length(G=self.graph_map, source=node_start, target=node_end,
                                                     weight='length')
                    # calculate travel time between start node and end node
                    time = networkx.shortest_path_length(G=self.graph_map, source=node_start, target=node_end,
                                                   weight='travel_time')
                    # add node start to graph_node
                    graph_node.add_node(node_for_adding=start)
                    # add edge(travel_time and length) between start node and end node
                    graph_node.add_edge(u_of_edge=start, v_of_edge=end, travel_time=time, length=length)
        return graph_node

    # order of node for find the shortest path
    def shortest_path(self, start, weight='length'):
        # define basic shortest_length and shortest_path ==> None
        shortest_length = None
        shortest_path = None
        graph = self.graph_nodes()
        # for loop for calculate the shortest path between all pairs
        for node in self.nodes:
            # check for don't calculate path between start and start
            if node != start:
                # calculate all path between all pairs
                paths = list(networkx.all_simple_paths(G=graph, source=start, target=node))
                # calculate the smallest weight of paths
                for path in paths:
                    # define basic weight of path
                    weight_path = 0
                    # check for use all nodes in path
                    if len(path) == len(graph.nodes):
                        # calculate weight of path
                        for point in path:
                            # check for use all nodes as start node without last node
                            if -(len(path) - path.index(point)) != -1:
                                # calculate weight between all nodes in path (weight of path)
                                weight_path += networkx.shortest_path_length(G=graph, source=point, target=path[path.index(point) + 1], weight=weight)
                        # check the current path is shortest or no
                        if shortest_length is None or weight_path < shortest_length:
                            # update shortest_path and shortest_length
                            shortest_path = path
                            shortest_length = weight_path
        return shortest_path

    # find route in map
    def route_in_map(self, path, weight='length'):
        # define basic route
        route = []
        # convert input nodes to nodes in graph_map
        for node in path:
            path[path.index(node)] = osmnx.nearest_nodes(G=self.graph_map, X=node[1], Y=node[0])
        # calculate route in graph_map
        for node in path:
            if -(len(path) - path.index(node)) != -1:
                route.append(networkx.shortest_path(G=self.graph_map, source=node, target=path[path.index(node) + 1], weight=weight))
        return route


nodes = [
    (30.29320355864577, 57.080280571604504),
    (30.28729303076457, 57.07993724872948),
    (30.28966469601973, 57.06178405973304),
    (30.298372662524653, 57.07083919656105),
    (30.2934814743869, 57.08465793617537)
]
x = DMA(place='Iran Kerman Kerman', street_type='drive')
x.nodes = x.add_nodes(nodes=nodes)
shortest_path = x.shortest_path(nodes[0], weight='length')
route = x.route_in_map(shortest_path)
print(route)
