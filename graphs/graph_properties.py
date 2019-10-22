import graphs, exceptions, bfs, dfs, test_graphs


def has_undirected_cycle(graph):
    """ Determine if there is cycle in undirected graph.

        Determine if there is a cycle in an undirected graph using breadth
        first search. Tests all connected components of a disconnected graph.

        Parameters
        ----------
        graph : Graph instance
            The graph to explore for cycles

        Returns
        -------
        distance : bool
            Indication of whether the graph contains a cycle

        Raises
        ------
        GraphTypeError
            If a directed graph is passed
    """

    if graph.directed:
        raise exceptions.GraphTypeError("Graph is directed")

    color, d, p, has_cycle, b = bfs.bfs(graph, next(iter(graph.vertices)))
    for vertex in graph.vertices:
        if has_cycle:
            return True
        if color[vertex] == "white":
            temp_color, d, p, has_cycle, b = bfs.bfs(graph, vertex)
            for vertex in graph.vertices:
                color[vertex] = "black" if temp_color[vertex] == "black" else color[vertex]

    return has_cycle


def is_bipartite(graph):
    """ Determines if a graph is bipartite.

        Determine if a graph is bipartite using breadth first search. Tests
        all connected components of a disconnected graph.

        Parameters
        ----------
        graph : Graph instance
            The graph to test as bipartite

        Returns
        -------
        distance : bool
            Indication of whether the graph is bipartite
    """

    color, d, p, hc, is_bipartite = bfs.bfs(graph, next(iter(graph.vertices)))
    for vertex in graph.vertices:
        if not is_bipartite:
            return False
        if color[vertex] == "white":
            temp_color, d, p, hc, is_bipartite = bfs.bfs(graph, vertex)
            for vertex in graph.vertices:
                color[vertex] = "black" if temp_color[vertex] == "black" else color[vertex]

    return is_bipartite
