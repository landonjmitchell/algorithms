
class GraphTypeError(Exception):
    """ Error indicating an incompatible graph type has been passed to a
        graph function.

    raise: GraphTypeError
        Raises error indicating incompatible graph type
    """

    pass


class VertexNotFoundError(KeyError):
    """ Error indicating a vertex passed to a method or function is not in
        the set of the vertices belonging to the graph that was passed.

    raise: VertexNotFound
        Raises error indicating not matching vertex in the graph
    """

    def __init__(self, vertex=None, *args, **kwargs):
        self.vertex = vertex
        if vertex is not None:
            msg = f"Vertex '{str(self.vertex)}' not found in graph"
        else:
            msg = 'Vertex not found'
        super().__init__(msg, *args, **kwargs)
