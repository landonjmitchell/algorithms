from collections import defaultdict
import test_graphs

class Graph:
    """ Object representing a graph in adjacency list format. Supports directed
        and undirected graphs. Supports edge weights, parallel edges, and loops.
        Includes methods for adding edges and vertices.

    Parameters
    ----------
    directed : bool
        Indication of whether the graph is directed

    Attributes
    ----------
    directed : bool
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
    """

    def __init__(self, directed=False):
        """ Initialize Graph instance """

        self.directed = directed
        self.edges = []
        self.vertices = set()
        self.adjacency_list = defaultdict(list)
        self.weights = defaultdict(int)

    def add_edge(self, v1, v2, weight=1):
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
            The weight of the given edge (defaults to 1)
        """

        self.vertices.update((v1, v2))
        if self.directed:
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

        Add a vertex to the graph's set of vertices.

        Parameters
        ----------
        vertex : any hashable value
            The vertex to be added to the graph
        """
        self.vertices.add(vertex)

    def reverse(self):
        """ Return graph with reversed directed edges.
        """

        if not self.directed:
            return

        self.edges = [edge[::-1] for edge in self.edges]
        new_weights = {edge[::-1]: weight for edge, weight in self.weights.items()}
        self.weights = new_weights

        new_adj_list = defaultdict(list)
        for vertex, neighbors in self.adjacency_list.items():
            for neighbor in neighbors:
                new_adj_list[neighbor].append(vertex)
        self.adjacency_list = new_adj_list
