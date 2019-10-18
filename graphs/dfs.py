import graphs
import test_graphs

def dfs(graph):
    """ Return information derived from a depth first traversal of a graph.

        Parameters
        ----------
        graph : Graph instance
            The graph to be traversed using depth first search

        Returns
        -------
        colors : dict of str keyed by any hashable values
            Indication of whether keyed vertex has been visited. 'White'
            indicates that the vertex has not been visited. 'Grey' indicates
            that the vertex has been visited but its neighbors have not.
            'Black' indicates that the vertex and its neighbors have been
            visited.
        discover : dict of int keyed by any hashable values
            Discovery time of corresponding vertex
        finish : dict of int keyed by any hashable values
            Discovery time of corresponding vertex
        parents : dict of any hashable values keyed by any hashable values / None
            Immediate parent of keyed vertex in a minimal edge length path
            from the starting vertex to the corresponding vertex. Value is
            None for the starting vertex and any vertex not reachable from
            the starting vertex.
        cycle : bool
            Indication of whether the graph has a cycle.
        top_sort : list of any hashable values / None
            List of graph vertices, topologically sorted. More than one valid
            topological sort may be possible for a given graph. None is
            returned if there is no way to topologically sort the graph.
        connected_components : list of sets of any hashable values
            Collections of the vertices comprising each connected component
            of the graph.
    """

    global time, directed, cycle, top_sort, connected_components
    time = 0
    directed = graph.directed
    cycle = False
    top_sort = []
    connected_components = []

    colors = {vertex: "white" for vertex in graph.vertices}
    parents = {vertex: None for vertex in graph.vertices}
    discovery = {}
    finish = {}

    for vertex in graph.vertices:
        if colors[vertex] == "white":
            connected_components.append(set())
            dfs_visit(graph, vertex, colors, parents, discovery, finish)

    top_sort = None if (cycle or not directed) else top_sort[::-1]
    return colors, parents, discovery, finish, cycle, top_sort, connected_components


def dfs_visit(graph, vertex, colors, parents, discovery, finish):
    """ Updates values and recursively visits adjacent vertices.

        Parameters
        ----------
        graph : Graph instance
            The graph to be traversed using depth first search
        colors : dict of str keyed by any hashable values
            Indication of whether keyed vertex has been visited. 'White'
            indicates that the vertex has not been visited. 'Grey' indicates
            that the vertex has been visited but not all of its neighbors
            have been visited yet.'Black' indicates that the vertex and its
            neighbors have been visited.
        discover : dict of int keyed by any hashable values
            Discovery time of corresponding vertex
        finish : dict of int keyed by any hashable values
            Discovery time of corresponding vertex
        parents : dict of any hashable values keyed by any hashable values / None
            Immediate parent of keyed vertex according to discovery order.
        top_sort : list of any hashable values / None
            List of graph vertices, topologically sorted.
        connected_components : list of sets of any hashable values
            Collections of the vertices comprising each connected component
            of the graph.
    """
    
    global time, directed, cycle, top_sort, connected_components
    time += 1

    colors[vertex] = "grey"
    discovery[vertex] = time
    num_cc = len(connected_components)
    connected_components[num_cc - 1].add(vertex)

    for neighbor in graph.adjacency_list[vertex]:
        if colors[neighbor] == "white":
            parents[neighbor] = vertex
            dfs_visit(graph, neighbor, colors, parents, discovery, finish)
        elif (not directed) and parents[vertex] != neighbor:
            cycle = True
        elif directed and colors[neighbor] == "grey":
            cycle = True

    colors[vertex] = "black"
    top_sort.append(vertex)
    time += 1
    finish[vertex] = time
