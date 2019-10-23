from collections import defaultdict
import graphs, exceptions, test_graphs

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
        connected_components : set of frozensets of hashable values
            Collections of the vertices comprising each connected component
            of the graph.
    """

    global time, directed, cycle, top_sort, stack
    directed = graph.directed

    time = 0
    discovery = {}
    finish = {}

    cycle = False
    top_sort = []
    colors = {vertex: "white" for vertex in graph.vertices}
    parents = {vertex: None for vertex in graph.vertices}

    stack = []
    in_stack = {vertex: False for vertex in graph.vertices}
    lows = {}
    connected_components = set()


    for vertex in graph.vertices:
        if colors[vertex] == "white":
            dfs_visit(graph, vertex, colors, parents, discovery,
                      finish, lows, in_stack, connected_components)

    top_sort = None if (cycle or not directed) else top_sort[::-1]
    return colors, parents, discovery, finish, cycle, top_sort, connected_components


def dfs_visit(graph, vertex, colors, parents, discovery, finish, lows, in_stack, connected_components):
    """ Updates values and recursively visits adjacent vertices.

        Parameters
        ----------
        graph : Graph instance
            The graph to be traversed using depth first search
        vertex : hashable value
            The vertex currently being visited
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
        lows : dict of int keyed by hashable value
            The minimum discovery time of vertices reachable by the keyed
            vertex
        in_stack : dict of bool keyed by hashable value
            Indication of whether the keyed vertex is currently in the stack
            (used for Tarjan's algorithm)
        connected_components : set of frozensets of hashable values
            Collections of the vertices comprising each connected component
            of the graph.
    """

    global time, directed, cycle, top_sort, stack
    time += 1

    colors[vertex] = "grey"
    discovery[vertex] = time
    stack.append(vertex)
    in_stack[vertex] = True
    lows[vertex] = time

    for neighbor in graph.adjacency_list[vertex]:
        if colors[neighbor] == "white":
            parents[neighbor] = vertex
            dfs_visit(graph, neighbor, colors, parents, discovery, finish, lows, in_stack, connected_components)
            lows[vertex] = min(lows[vertex], lows[neighbor])

        else:
            if in_stack[neighbor]:
                lows[vertex] = min(lows[vertex], discovery[neighbor])
            if (not directed) and parents[vertex] != neighbor:
                cycle = True
            if directed and colors[neighbor] == "grey":
                cycle = True

    colors[vertex] = "black"
    top_sort.append(vertex)
    time += 1
    finish[vertex] = time

    if lows[vertex] == discovery[vertex]:
        component = set()
        vert = None
        while vert != vertex:
            vert = stack.pop()
            component.add(vert)
            in_stack[vert] = False
        connected_components.add(frozenset(component))
