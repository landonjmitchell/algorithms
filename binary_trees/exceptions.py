class MissingNodeError(KeyError):
    """ Error indicating a node at a specific tree position does not exist.
    """

    def __init__(self, depth, position, *args, **kwargs):
        msg = f"No node exists at depth: {depth}, position {position}"
        super().__init__(msg, *args, **kwargs)


class NodePositionError(IndexError):
    """ Error indicating the given binary tree position does not exist.
    """

    def __init__(self, depth, position, *args, **kwargs):
        msg = f"Invalid tree position: (depth: {depth}, position {position})"
        super().__init__(msg, *args, **kwargs)


class NodePositionOccupiedError(Exception):
    """ Error indication a node position is already occupied.
    """

    pass


class NodeHasChildrenError(Exception):
    """ Error indication a node has children and so cannot be removed.
    """

    pass

