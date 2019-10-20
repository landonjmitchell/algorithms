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
