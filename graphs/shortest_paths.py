import heapq
import graphs, exceptions, bfs, dfs, test_graphs

def shortest_unweighted_distance(graph, start, end):
    """ Return miminal edge distance between two vertices.

        Return the shortest distance between start and end vertices in a
        graph, ignoring edge weights, if any exist.

        Parameters
        ----------
        graph : Graph instance
            The graph from which the distance is to be calculated
        start: Any hashable value
            The starting vertex from which to calculate the distance
        end: Any hashable value
            The ending vertex to which the distance is calculated

        Returns
        -------
        distance : int / float
            The minimum number of edges that need to be traversed to reach
            the end vertex from the start vertex. Returns float('inf') if end
            is unreachable from start.

        Raises
        ------
        VertexNotFoundError
            If 'start' or 'end' argument is not a vertex in the graph
    """

    if start not in graph.vertices:
        raise exceptions.VertexNotFound(start)
    if end not in graph.vertices:
        raise exceptions.VertexNotFound(end)

    c, distances, p, hc, b = bfs.bfs(graph, start)
    return distances[end]


def shortest_unweighted_path(graph, start, end):
    """ Return a minimal edge path between two vertices.

        Return a shortest path from start to end in an unweighted graph.
        There may be more than one.

        Parameters
        ----------
        graph : Graph instance
            The graph from which the path is to be calculated
        start: Any hashable value
            The starting vertex from which to determine the path
        end: Any hashable value
            The ending vertex to which the path is determined

        Returns
        -------
        path : list of hashable values / None
            A minimumal edge path between the start and end vertices,
            inclusive of both start and end vertices. Returns None if no
            path from start to end vertices exists.

        Raises
        ------
        VertexNotFoundError
            If 'start' or 'end' argument is not a vertex in the graph
    """

    if start not in graph.vertices:
        raise exceptions.VertexNotFound(start)
    if end not in graph.vertices:
        raise exceptions.VertexNotFound(end)

    c, distances, parent, hc, b = bfs.bfs(graph, start)
    if distances[end] == float('inf'):
        return None

    path = []
    vertex = end
    while vertex is not None:
        path.append(vertex)
        vertex = parent[vertex]

    return path[::-1]


def dijkstra(graph, start):
    """ Return shortest weighted distances and paths from start vertex.

        Return the shortest weighted distances and paths from a starting
        vertex to all other vertices. There may be more than one valid
        shortest path for a given end vertex. Will not work on graphs with
        a cycle whose weights have a negative sum.

        Parameters
        ----------
        graph : Graph instance
            The graph from which paths and distances are to be calculated
        start : Any hashable value
            The starting vertex from which paths and distances are to be
            calculated

        Returns
        -------
        paths : dict of lists of hashable values keyed by hashable values
            Lists of the ordered vertices composing a shortest path between
            the starting vertex and the corresponding end vertex. Value will
            be an empty list if the ending vertex is not reachable from the
            starting vertex.
        distance : dict of ints/floats keyed by hashable values
            The shortest weighted distance between the starting vertex and
            the corresponding vertex. Value will be infinite if the ending
            vertex is not reachable from the starting vertex.

        Raises
        ------
        VertexNotFoundError
            If 'start' argument is not a vertex in the graph
    """

    if start not in graph.vertices:
        raise exceptions.VertexNotFound(start)

    paths, distances, min_heap = initialize_single_source(graph, start)

    while min_heap:
        heap_distances, vertex = heapq.heappop(min_heap)

        # Due to heapq's inability to update values in the heap, vertices may
        # be added multiples times. We ignore vertices from the heap whose
        # distance is greater than the vertex's current known minimal distance.
        if heap_distances > distances[vertex]:
            continue

        for neighbor in graph.adjacency_list[vertex]:
            relax(graph, vertex, neighbor, paths, distances, min_heap)

    for vertex, path in paths.items():
        paths[vertex] = path + [vertex] if path else path

    return paths, distances


def initialize_single_source(graph, start):
    """ Return initialized values for Dijkstra algorithm from start vertex.

        Return data structures representing paths, distances, and a min-heap
        priority queue used when beginning Dijkstra's algorithm.

        Parameters
        ----------
        graph : Graph instance
            The graph that will be used in Dijkstra's algorithm
        start : Any hashable value
            The starting vertex that will be used in Dijkstra's algorithm

        Returns
        -------
        paths : dict of lists of hashable values keyed by hashable values
            Dictionary of empty lists keyed by vertex to which will be
            appended the shortest paths to that vertex.
        distance : dict of ints/floats keyed by hashable values
            Dictionary that represents the shortest known distance to each
            vertex from the starting vertex. Starting vertex has a value of 0.
        min_heap : list of 2-tuples of int/float and hashable values
            Data structure representing the current minimum distance and
            vertex ID for each vertex in the graph. Initial value for starting
            vertex is (0, [starting vertex ID]).
    """
    paths = {vertex: [] for vertex in graph.vertices}
    distances = {vertex: float('inf') for vertex in graph.vertices}
    distances[start] = 0

    min_heap = [(float('inf'), v) for v in graph.vertices if v != start]
    min_heap.append((0, start))
    heapq.heapify(min_heap)

    return paths, distances, min_heap


def relax(graph, vertex, neighbor, paths, distances, min_heap):
    """ Update current known minimum distance and path for a given vertex.

        Updates the currently known minimum distance and minimum distance
        path for a given vertex.

        Parameters
        ----------
        graph : Graph instance
            The graph being used in Dijkstra's algorithm
        vertex : Any hashable value
            The vertex with the current minimal distance from the start
        neighbor :
            A vertex adjacent to the current minimally distanced vertex.
        paths : dict of lists of hashable values keyed by hashable values
            Lists of the ordered vertices composing a current shortest path
            between the starting vertex and the corresponding end vertex.
        distance : dict of ints/floats keyed by hashable values
            The current shortest weighted distance between the starting vertex
            and the corresponding vertex.
        min_heap : list of 2-tuples of int/float and hashable values
            Data structure representing the current minimum distance and
            vertex ID for each vertex in the graph.
    """

    edge = (vertex, neighbor) if graph.directed else frozenset([vertex, neighbor])
    edge_weight = graph.weights[edge]

    if distances[neighbor] > (distances[vertex] + edge_weight):
        distances[neighbor] = (distances[vertex] + edge_weight)
        heapq.heappush(min_heap, (distances[neighbor], neighbor))
        paths[neighbor] = paths[vertex] + [vertex]
