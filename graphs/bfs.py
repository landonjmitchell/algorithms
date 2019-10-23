from queue import deque
import graphs, exceptions, test_graphs


def bfs(graph, start):
    """ Return results of breadth first traversal of a graph.

        Return information derived from a breadth first traversal of a graph,
        starting from a specified vertex.

        Parameters
        ----------
        graph : Graph instance
            The graph to be traversed using breadth first search
        start: Any hashable value
            The starting vertex from which to transverse the graph

        Returns
        -------
        colors : dict of str keyed by any hashable value
            Indication of whether keyed vertex has been visited. 'White'
            indicates that the vertex has not been visited. 'Grey' indicates
            that the vertex has been visited but its neighbors have not.
            'Black' indicates that the vertex and its neighbors have been
            visited.
        distances : dict of int/float keyed by any hashable value
            Distance in edges from the starting vertex to the keyed vertex.
            float('inf') if vertex is not reachable from the start vertex.
        parents : dict of any hashable value keyed by any hashable value / None
            Immediate parent of keyed vertex in a minimal edge length path
            from the starting vertex to the corresponding vertex. Value is
            None for the starting vertex and any vertex not reachable from
            the starting vertex.
        has_cycle : bool
            Indication of whether the connected component of which the
            starting vertex is a member contains an unweighted cycle.
        is_bipartite : bool
            Indication of whether the connected component of which the
            starting vertex is a member is bipartite.

        Raises
        ------
        VertexNotFoundError
            If 'start' argument is not a vertex in the graph

    """

    if start not in graph.vertices:
        raise exceptions.VertexNotFoundError(start)

    has_cycle = False
    is_bipartite = True

    distances = {vertex: float('inf') for vertex in graph.vertices}
    bipartite_colors = {vertex: None for vertex in graph.vertices}
    parents = {vertex: None for vertex in graph.vertices}
    colors = {vertex: "white" for vertex in graph.vertices}

    distances[start] = 0
    colors[start] = "grey"
    bipartite_colors[start] = "red"

    queue = deque([start])
    while queue:
        vertex = queue.pop()
        for neighbor in graph.adjacency_list[vertex]:

            if bipartite_colors[neighbor] == bipartite_colors[vertex]:
                is_bipartite = False
            elif bipartite_colors[vertex] == "blue":
                bipartite_colors[neighbor] = "red"
            else:
                bipartite_colors[neighbor] = "blue"

            if colors[neighbor] == "white":
                colors[neighbor] = "grey"
                distances[neighbor] = distances[vertex] + 1
                parents[neighbor] = vertex
                queue.appendleft(neighbor)
            elif parents[vertex] != neighbor:
                has_cycle = True

        colors[vertex] = "black"

    return colors, distances, parents, has_cycle, is_bipartite
