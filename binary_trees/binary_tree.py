from collections import deque

from exceptions import MissingNodeError, NodePositionError, NodePositionOccupiedError

class BinaryTree:

    def __init__(self, root=None):
        self.root = root
        self._height = None
        self.size = 0

    @property
    def height(self):
        if self._height is not None:
            return self._height

        if self.root is None:
            self._height = -1
            return -1

        queue = deque()
        queue.append(self.root)
        height = -1
        while queue:
            height += 1
            num_nodes = len(queue)
            for _ in range(num_nodes):
                node = queue.pop()

                if node.left is not None:
                    queue.appendleft(node.left)
                if node.right is not None:
                    queue.appendleft(node.right)

        self._height = height
        return height

    def is_height_balanced(self):
        pass
        
    def add_node(self, node, depth, position):

        if depth == 0:
            self.root = node
            return

        odd = True if position % 2 else False
        parent_position = (position - 1) // 2 if odd else position // 2
        parent = self.get_node(depth - 1, parent_position)

        if odd:
            parent.add_right_child(node)
        else:
            parent.add_left_child(node)

    def get_node(self, depth, position):
        if position < 0 or position > 2**depth - 1:
            raise NodePositionError(depth, position)

        if position < 0 or position > 2**depth - 1 or depth < 0:
            raise NodePositionError(depth, position)

        node = self.root
        low, high = 0, 2**depth - 1
        for _ in range(depth):
            mid = (high + low) / 2
            if position > mid:
                low = mid
                node = node.right
            else:
                high = mid
                node = node.left

            if node is None:
                raise MissingNodeError(depth, position)

        return node

    def value_at_node(self, depth, position):
        node = self.get_node(depth, position)
        return node.value

        

class Node:
    
    def __init__(self, value):
        self.value = value
        self.left = None
        self.right = None

    def add_left_child(self, node):
        if self.left is not None:
            raise NodePositionOccupiedError('Left node already exists')
        self.left = node

    def add_right_child(self, node):
        if self.right is not None:
            raise NodePositionOccupiedError('Right node already exists')
        self.right = node

