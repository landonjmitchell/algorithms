import copy
import heapq
from collections import defaultdict
from queue import deque
import exceptions

UNVISITED, VISITING, VISITED = -1, 0, 1


class Graph:
    """ Object representing a graph in adjacency list format. Supports directed
        and undirected graphs, edge weights, and loops. 
 
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

        self.edges = []
        self.vertices = set()
        self.adjacency_list = defaultdict(list)
        self.weights = defaultdict(int)

        self.was_bfs_explored = False
        self.was_dfs_explored = False

        self._is_weighted = None
        self._is_strongly_connected = None
        self._has_cycle = None
        self._is_bipartite = None

        self.visited = {}
        self.parents = {}
        self.discovery_times = {}
        self.finishing_times = {}
        self.distances = defaultdict(lambda: defaultdict(float))

        self._topological_sort = None
        self._connected_components = None

    @property
    def is_weighted(self):
        if self._is_weighted is None:
            weighted = sum([weight != 0 for weight in self.weights.values()])
            self._is_weighted = True if weighted else False
        return self._is_weighted

    @property
    def is_strongly_connected(self):
        if self._is_strongly_connected is None:
            if len(self.connected_components) > 1:
                self._is_strongly_connected = False
                return False

            transposed_graph = self.transposed()
            if len(transposed_graph.connected_components) > 1:
                self._is_strongly_connected = False
                return False
        
            self._is_strongly_connected = True

        return self._is_strongly_connected

    @property
    def has_cycle(self):
        if self.was_dfs_explored:
            return self._has_cycle

        if self._has_cycle is None:
            self.dfs_explore()
        return self._has_cycle

    @has_cycle.setter
    def has_cycle(self, value):
        self._has_cycle = value

    @property
    def topological_sort(self):
        if self.was_dfs_explored:
            return self._topological_sort

        if self.has_cycle or not self.is_directed:
            return None

        if self._topological_sort is None:
            self.dfs_explore()
        return self._topological_sort

    @topological_sort.setter
    def topological_sort(self, value):
        self._topological_sort = value

    @property
    def connected_components(self):
        if self.was_dfs_explored:
            return self._connected_components

        if self._connected_components is None:
            self.dfs_explore()
        return self._connected_components

    @connected_components.setter
    def connected_components(self, value):
        self._connected_components = value

    @property
    def is_bipartite(self):
        
        if self._is_bipartite is None:
            self.reset_bfs_vertex_values()
            visited_vertices = set()
            for vertex in self.vertices:
                if vertex not in visited_vertices:
                    self.bfs_explore(vertex)
                    if not self.is_bipartite:
                        return False

                    self.is_bipartite = None
                    for v in self.vertices:
                        if self.visited[v] == VISITED:
                            visited_vertices.add(v)
            self.is_bipartite = True
                      
        return self._is_bipartite

    @is_bipartite.setter
    def is_bipartite(self, value):
        self._is_bipartite = value

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

        self.reset_graph_properties

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
            self.reset_graph_properties()

    def reset_graph_properties(self):
        """ Reset graph properties assigned by bfs or dfs.
        """
        
        if self.visited:
            self.was_bfs_explored = False
            self.was_dfs_explored = False

            self._is_weighted = None
            self.has_cycle = None
            self.is_bipartite = None

            self.reset_all_vertex_values()

            self.topological_sort = None
            self.connected_components = None

    def reset_bfs_vertex_values(self):
        for vertex in self.vertices:
            self.visited[vertex] = UNVISITED
            self.parents[vertex] = None
            self.distances[vertex] = defaultdict(float)

    def reset_dfs_vertex_values(self):
        for vertex in self.vertices:
            self.visited[vertex] = UNVISITED
            self.parents[vertex] = None
            self.discovery_times[vertex] = None
            self.finishing_times[vertex] = None

    def reset_all_vertex_values(self):
        self.reset_bfs_vertex_values()
        self.reset_dfs_vertex_values()

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

        self.reset_graph_properties()

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

        transposed_graph.reset_graph_properties()
        return transposed_graph

    def bfs_explore(self, start):
        """ Update graph properties via breadth first traversal.

            Update is_bipartite, visited, parents, and distances properties of the graph using breadth first search from a given starting vertex.
        """
        
        if start not in self.vertices:
            raise exceptions.VertexNotFoundError(start)

        self.reset_bfs_vertex_values()
        self.was_bfs_explored = True

        self.is_bipartite = True
        self.distances[start][start] = 0
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
                    self.distances[start][neighbor] = self.distances[start][vertex] + 1
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

        self.reset_dfs_vertex_values()
        self.was_dfs_explored = True
        self.has_cycle = False
        self.topological_sort = []
        self.connected_components = set()

        for vertex in self.vertices:
            if self.visited[vertex] == UNVISITED:
                self.dfs_visit(vertex, lows, in_stack)

        if self.is_directed and not self.has_cycle:
            self.topological_sort.reverse()
        else:
            self.topological_sort = None

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

    def shortest_unweighted_distance_path(self, start, end):
        """ Return miminal edge distance between two vertices.

        Return the shortest distance between start and end vertices in a
        graph, ignoring edge weights, if any exist.

        Parameters
        ----------
        start: Any hashable value
            The starting vertex from which to calculate distance and path
        end: Any hashable value
            The ending vertex to which to calculate distance and path

        Returns
        -------
        distance : int / float
            The minimum number of edges that need to be traversed to reach
            the end vertex from the start vertex. Returns float('inf') if end
            is unreachable from start.
        path : list of hashable values / None
            A minimumal edge path between the start and end vertices,
            inclusive of both start and end vertices. Returns None if no
            path from start to end vertices exists.

        """

        self.bfs_explore(start)
        distance = self.distances[start][end]

        if self.distances[end] == float('inf'):
            path = None
        else:
            path = []
            vertex = end
            while vertex is not None:
                path.append(vertex)
                vertex = self.parents[vertex]
            path.reverse()

        return distance, path

    def dijkstra_distances_paths(self, start):
        """ Return shortest weighted distances and paths from start vertex.

        Return the shortest weighted distances and paths from a starting
        vertex to all other vertices. There may be more than one valid
        shortest path for a given end vertex. Will not work on graphs with
        a cycle whose weights have a negative sum.

        Parameters
        ----------
 
        start : Any hashable value
            The starting vertex from which distances and paths are to be
            calculated

        Returns
        -------
        distances : dict of ints/floats keyed by hashable values
            The shortest weighted distance between the starting vertex and
            the corresponding vertex. Value will be infinite if the ending
            vertex is not reachable from the starting vertex.
        paths : dict of lists of hashable values keyed by hashable values
            Lists of the ordered vertices composing a shortest path between
            the starting vertex and the corresponding end vertex. Value will
            be an empty list if the ending vertex is not reachable from the
            starting vertex.
        """
 
        paths, min_heap = self.initialize_single_source(start)

        while min_heap:
            heap_distances, vertex = heapq.heappop(min_heap)

            # Due to heapq's inability to update values in the heap, vertices may be added multiples times. We ignore vertices from the heap whose distance is greater than the vertex's current known minimal distance.
            if heap_distances > self.distances[start][vertex]:
                continue

            for neighbor in self.adjacency_list[vertex]:
                self.relax(start, vertex, neighbor, paths, min_heap)

        for vertex, path in paths.items():
            paths[vertex] = path + [vertex] if path else None

        return self.distances[start], paths

    def initialize_single_source(self, start):
        """ Return initialized values for Dijkstra algorithm from start vertex.

            Return data structures representing paths, and a min-heap
            priority queue used when beginning Dijkstra's algorithm.

            Parameters
            ----------
            start : Any hashable value
                The starting vertex that will be used in Dijkstra's algorithm

            Returns
            -------
            paths : dict of lists of hashable values keyed by hashable values
                Dictionary of empty lists keyed by vertex to which will be
                appended the shortest paths to that vertex.
            min_heap : list of 2-tuples of int/float and hashable values
                Data structure representing the current minimum distance and
                vertex ID for each vertex in the graph. Initial value for starting vertex is (0, [starting vertex ID]).
        """

        paths = {vertex: [] for vertex in self.vertices}
        self.distances[start] = {v: float('inf') for v in self.vertices}
        self.distances[start][start] = 0

        min_heap = [(float('inf'), v) for v in self.vertices if v != start]
        min_heap.append((0, start))
        heapq.heapify(min_heap)

        return paths, min_heap

    def relax(self, start, vertex, neighbor, paths, min_heap):
        """ Update current known minimum distance and path for a given vertex.

            Updates the currently known minimum distance and minimum distance
            path for a given vertex.

            Parameters
            ----------
            start : Any hashable value
                The starting vertex in the current Dijkstra algorithm
            vertex : Any hashable value
                The vertex with the current minimal distance from the start
            neighbor :
                A vertex adjacent to the current minimally distanced vertex.
            paths : dict of lists of hashable values keyed by hashable values
                Lists of the ordered vertices composing a current shortest path
                between the starting vertex and the corresponding end vertex.
            min_heap : list of 2-tuples of int/float and hashable values
                Data structure representing the current minimum distance and
                vertex ID for each vertex in the graph.
        """

        edge = (vertex, neighbor) if self.is_directed else frozenset(
            [vertex, neighbor])
        edge_weight = self.weights[edge]

        current_distance = self.distances[start][neighbor]
        new_distance = self.distances[start][vertex] + edge_weight

        if  current_distance > new_distance:
            self.distances[start][neighbor] = new_distance
            heapq.heappush(min_heap, (new_distance, neighbor))
            paths[neighbor] = paths[vertex] + [vertex]


