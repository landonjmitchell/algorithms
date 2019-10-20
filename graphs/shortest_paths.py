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
