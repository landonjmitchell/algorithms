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


def has_cycle(graph):
    """ Determine if there is cycle in a graph.

        Determine if there is a cycle in a graph using depth first search.
        Tests all connected components of a disconnected graph.

        Parameters
        ----------
        graph : Graph instance
            The graph to explore for cycles

        Returns
        -------
        distance : bool
            Indication of whether the graph contains a cycle
    """

    c, p, d, f, has_cyc, ts, cc = dfs.dfs(graph)
    return has_cyc


def is_strongly_connected(graph):
    """ Determine if a graph is connected.

        Determine if an undirected graph is connected, or a directed graph is
        strongly connected, using depth first search.

        Parameters
        ----------
        graph : Graph instance
            The graph to explore

        Returns
        -------
        distance : bool
            Indication of whether the graph is (strongly) connected
    """

    cp, p, d, f, hs, ts, connected_components = dfs.dfs(graph)
    if len(connected_components) > 1:
        return False

    graph.reverse()
    cp, p, d, f, hs, ts, connected_components = dfs.dfs(graph)
    if len(connected_components) > 1:
        return False

    return True


def undirected_connected_components(graph):
    """ Return the connected components of an undirected graph.

        Return a list of sets containing the ID of the vertices included in
        each connected component of an undirected graph using depth first
        search.

        Parameters
        ----------
        graph : Graph instance
            The graph to explore

        Returns
        -------
        connected_components : list of sets of hashable values
            List of sets containing the IDs of the vertices included in each
            connected component.

        Raises
        ------
        GraphTypeError
            If a directed graph is passed
    """

    if graph.directed:
        raise exceptions.GraphTypeError("Graph is directed")
        
    cp, p, d, f, hs, ts, connected_components = dfs.dfs(graph)
    return connected_components


def topological_sort(graph):
    """ Return a topological sorting of a directed graph's vertices.

        Return a topological sorting of a graph's vertices if one exists.
        Direct graphs with cycles will not have a valid topological sorting.
        There may be more than one valid topological sorting of a given graph.
        Uses depth first search.

        Parameters
        ----------
        graph : Graph instance
            A directed graph from which to build a topological ordering

        Returns
        -------
        top_sort : list of hashable values / None
            list of the graph's vertices topologically sorted, i.e. a
            'linear ordering of its vertices such that for every directed
            edge uv from vertex u to vertex v, u comes before v in the
            ordering'. Returns None if the graph has a cycle.

        Raises
        ------
        GraphTypeError
            If an undirected path is passed
    """

    if not graph.directed:
        raise exceptions.GraphTypeError("Graph is undirected. Topological sorting is not possible")

    c, p, d, f, hs, top_sort, cc = dfs.dfs(graph)
    return top_sort
