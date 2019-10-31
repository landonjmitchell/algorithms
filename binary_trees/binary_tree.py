from collections import deque

from exceptions import MissingNodeError, NodePositionError, NodePositionOccupiedError

class BinaryTree:

    def __init__(self, root=None):
        self.root = root
        self.depth = None
        self.size = 0

    def is_balanced(self, strict=False):
        """ Determine if a binary tree is balanced

            When strict, returns True if the tree has the minimal possible height. When not strict, returns True if the depth of the nearest leaf is within one of the depth of the farthest leaf.

            Parameters
            ----------
            strict : bool
                True to test for minimal possible height, False to test for maximum leaf depth difference of 1. Defaults to False.

            Returns
            -------
            balanced : bool
                True if balanced, False if not.


        """

        if self.root is None:
            return True

        height, min_height = 0, 0
        min_found = False
        queue = deque()
        queue.append(self.root)
        while queue:
            num_nodes = len(queue)
            for _ in range(num_nodes):
                node = queue.pop()

                # found empty child
                if strict and (node.left is None or node.right is None):
                    min_found = True

                # found leaf node
                if not strict and node.left is None and node.right is None:
                    min_found = True

                if node.left is not None:
                    queue.appendleft(node.left)
                if node.right is not None:
                    queue.appendleft(node.right)
                
            if abs(height - min_height) > 1:
                return False

            min_height += 1 if not min_found else 0
            height += 1

        return True
        
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

