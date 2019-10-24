import copy
from collections import defaultdict
from queue import deque
import exceptions

UNVISITED, VISITING, VISITED = -1, 0, 1


class Graph:
    """ Object representing a graph in adjacency list format. Supports directed
        and undirected graphs, edge weights, parallel edges, and loops. 
 
    Parameters
    ----------
    is_directed : bool
        Indication of whether the graph is directed

    Attributes
    ----------
    is_directed : bool
        Indication of whether or not the graph is directed (defaults to False)
    edges : list of tuples (directed) or frozensets (undirected)
        Edges contained within the graph
    vertices : set of hashable values
        Vertices contained within the graph
    adjacency_list : defaultdict of lists of adjacent vertices by vertex
        Dictionary indicating which vertices are directly reachable from each
        vertex. Vertices with outdegree of 0 are not included in the
        dictionary's keys.
    weights: defaultdict of numerical values keyed by tuples or frozensets
        A dictionary of edge weights, keyed by the corresponding edge
    has_cycle : bool
        Indication of whether the connected component of which the
        starting vertex is a member contains an unweighted cycle.
    is_bipartite : bool
        Indication of whether the connected component of which the
        starting vertex is a member is bipartite.
    visited : dict of str keyed by hashable values
        Indication of whether keyed vertex has been visited. 'White'
        indicates that the vertex has not been visited. 'Grey' indicates
        that the vertex has been visited but its neighbors have not.
        'Black' indicates that the vertex and its neighbors have been
        visited.
    parents : dict of hashable values keyed by hashable values
        Immediate parent of keyed vertex in a minimal edge length path
        from the starting vertex to the corresponding vertex. Value is
        None for the starting vertex and any vertex not reachable from
        the starting vertex.
    discovery_times : dict of int keyed by any hashable values
        Discovery time of corresponding vertex
    finish_times : dict of int keyed by any hashable values
        Finishing time of corresponding vertex
    distances : dict of int/float keyed by any hashable value
        Distance in edges from a starting vertex to the keyed vertex.
        float('inf') if vertex is not reachable from the start vertex.
    connected_components : set of frozensets of hashable values
        Collections of the vertices comprising each connected component
        of the graph.
    """

    def __init__(self, is_directed=False):
        """ Initialize Graph instance 
        """

        self.is_directed = is_directed
        self._is_weighted = None

        self.edges = []
        self.vertices = set()
        self.adjacency_list = defaultdict(list)
        self.weights = defaultdict(int)

        self.has_cycle = None
        self.is_bipartite = None

        self.visited = {}
        self.parents = {}
        self.discovery_times = {}
        self.finishing_times = {}
        self.distances = {}

        self._topological_sort = None
        self.connected_components = None

    @property
    def is_weighted(self):
        if self._is_weighted is None:
            weighted = sum([weight != 0 for weight in self.weights.values()])
            self._is_weighted == True if weighted else False
        return self._is_weighted

    @property
    def topological_sort(self):
        if not self.is_directed:
            raise exceptions.GraphTypeError("Graph is undirected. Topological sorting is not possible")

        if self._topological_sort is not None:
            self.dfs_explore()
        return self._topological_sort

    @topological_sort.setter
    def topological_sort(self, value):
        self._topological_sort = value

    def add_edge(self, v1, v2, weight=0):
        """ Add an edge to a graph.

        Add an edge to a graph by converting two hashable values to a tuple
        or frozenset, depending on whether the graph is directed, and adding it
        to the graph's list of edges. Also adds the included vertices to the
        graph's set of vertices if not already present, and updates the
        graph's adjacency list and weights accordingly.

        Parameters
        ----------
        v1 : any hashable value
            The name or ID of the first vertex - the originating vertex in
            the case of directed graphs
        v2 : any hashable value
            The name or ID of the second vertex - the terminating vertex in
            the case of directed graphs
        weight : numerical value
            The weight of the given edge (defaults to 0)
        """

        self.add_vertex(v1)
        self.add_vertex(v2)

        if self.is_directed:
            edge = (v1, v2)
            self.adjacency_list[v1].append(v2)
        else:
            edge = frozenset([v1, v2])
            self.adjacency_list[v1].append(v2)
            self.adjacency_list[v2].append(v1)

        self.edges.append(edge)
        self.weights[edge] = weight

    def add_vertex(self, vertex):
        """ Add an vertex to a graph.

        Add a vertex to the graph's set of vertices. Initiate default values for the vertex'x visited, parent, discovery time, finishing time, and distance properties.

        Parameters
        ----------
        vertex : any hashable value
            The vertex to be added to the graph
        """
        if vertex not in self.vertices:
            self.vertices.add(vertex)

            self.visited[vertex] = UNVISITED
            self.parents[vertex] = None
            self.discovery_times[vertex] = None
            self.finishing_times[vertex] = None
            self.distances[vertex] = None

    def transpose(self):
        """ Reverses directed edges of graph without creating new copy.
        """

        if not self.is_directed:
            return

        self.edges = [edge[::-1] for edge in self.edges]
        new_weights = {edge[::-1]: weight for edge,
                       weight in self.weights.items()}
        self.weights = new_weights

        new_adj_list = defaultdict(list)
        for vertex, neighbors in self.adjacency_list.items():
            for neighbor in neighbors:
                new_adj_list[neighbor].append(vertex)
        self.adjacency_list = new_adj_list

    def transposed(self):
        """ Return a new graph with directed edges reversed.

        Returns
        -------
        transposed_graph : Graph instance
            Copy of original graph with edge directions reversed.
        """

        if not self.is_directed:
            return copy.deepcopy(self)

        transposed_graph = Graph(is_directed=True)
        for edge in self.edges:
            transposed_graph.add_edge(edge[1], edge[0], self.weights[edge])

        return transposed_graph

    def bfs_explore(self, start):
        """ Update graph properties via breadth first traversal.

            Update is_bipartite, visited, parents, and distances properties of the graph using breadth first search from a given starting vertex.
        """
        if start not in self.vertices:
            raise exceptions.VertexNotFoundError(start)

        self.is_bipartite = True
        self.distances[start] = 0
        self.visited[start] = VISITING
        bipartite_colors = {vertex: None for vertex in self.vertices}
        bipartite_colors[start] = "red"

        queue = deque([start])
        while queue:
            vertex = queue.pop()
            for neighbor in self.adjacency_list[vertex]:

                if bipartite_colors[neighbor] == bipartite_colors[vertex]:
                    self.is_bipartite = False
                elif bipartite_colors[vertex] == "blue":
                    bipartite_colors[neighbor] = "red"
                else:
                    bipartite_colors[neighbor] = "blue"

                if self.visited[neighbor] == UNVISITED:
                    self.visited[neighbor] = VISITING
                    self.distances[neighbor] = self.distances[vertex] + 1
                    self.parents[neighbor] = vertex
                    queue.appendleft(neighbor)

            self.visited[vertex] = VISITED

    def dfs_explore(self):
        """ Update graph properties via depth first traversal.

            Update visited, parents, discovery_times, finishing_times, has_cycle, topological_sort, and connected_components properties of the graph using depth first search.
        """

        global time, stack
        time = 0
        stack = []
        in_stack = {vertex: False for vertex in self.vertices}
        lows = {}

        for vertex in self.vertices:
            if self.visited[vertex] == UNVISITED:
                self.dfs_visit(vertex, lows, in_stack)

        if (not self.has_cycle or self.is_directed):
            self.topological_sort.reverse()

    def dfs_visit(self, vertex, lows, in_stack):
        """ Updates values and recursively visits adjacent vertices.

            Parameters
            ----------
            vertex : hashable value
                The vertex currently being visited
            lows : dict of int keyed by hashable value
                The minimum discovery time of vertices reachable by the keyed
                vertex
            in_stack : dict of bool keyed by hashable value
                Indication of whether the keyed vertex is currently in the stack
                (used for Tarjan's algorithm)
        """

        global time, stack
        time += 1

        self.visited[vertex] = VISITING
        self.discovery_times[vertex] = time
        stack.append(vertex)
        in_stack[vertex] = True
        lows[vertex] = time

        self.topological_sort = []
        self.connected_components = set()

        for neighbor in self.adjacency_list[vertex]:
            if self.visited[neighbor] == UNVISITED:
                self.parents[neighbor] = vertex
                self.dfs_visit(neighbor, lows, in_stack)
                lows[vertex] = min(lows[vertex], lows[neighbor])

            else:
                if in_stack[neighbor]:
                    lows[vertex] = min(
                        lows[vertex], self.discovery_times[neighbor])
                if (not self.is_directed) and self.parents[vertex] != neighbor:
                    self.has_cycle = True
                if self.is_directed and self.visited[neighbor] == VISITING:
                    self.has_cycle = True

        time += 1
        self.visited[vertex] = VISITED
        self.topological_sort.append(vertex)
        self.finishing_times[vertex] = time

        if lows[vertex] == self.discovery_times[vertex]:
            component = set()
            vert = None
            while vert != vertex:
                vert = stack.pop()
                component.add(vert)
                in_stack[vert] = False
            self.connected_components.add(frozenset(component))

