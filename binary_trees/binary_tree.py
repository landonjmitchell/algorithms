from collections import deque

from exceptions import MissingNodeError, NodePositionError, NodePositionOccupiedError, NodeHasChildrenError

class BinaryTree:

    def __init__(self, root=None):
        self.root = root
        self._height = None
        self.size = 0

    @property
    def height(self):
        return self.height_recursive(self.root)

    def height_recursive(self, root):

        if root is None:
            return -1
        else:
            return max(self.height_recursive(root.left) + 1, self.height_recursive(root.right) + 1)

    def height_iterative(self, root):
        if root is None:
            return -1

        queue = deque()
        queue.append(root)
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

        return height

    def add_node(self, node, depth, position):
        """ Add node at given tree position.

            Parameters
            ----------
            node : Node instance
                node to be added
            depth : int
                depth of node
            position : int
                horizontal position of node
        """
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

    def remove_node(self, depth, position):
        """ Remove node at given tree position.

            Parameters
            ----------
            depth : int
                depth of node
            position : int
                horizontal position of node

            Raises
            ------
            NodeHasChildrenError
                If node to be removed has children
        """

        if depth == position == 0:
            root = self.root
            if root is None or root.left is not None or root.right is not None:
                raise NodeHasChildrenError("Can't remove node with children")
        
        odd = True if position % 2 else False
        parent_position = (position -1) // 2 if odd else position // 2
        parent = self.get_node(depth - 1, parent_position)

        if odd:
            node = parent.right
            if not node.is_leaf():
                raise NodeHasChildrenError("Can't remove node with children")
            parent.right = None                
        else:
            node = parent.left
            if not node.is_leaf():
                raise NodeHasChildrenError("Can't remove node with children")
            parent.left = None
                
    def get_node(self, depth, position):
        """ Return node at given tree position.

            Parameters
            ----------
            depth : int
                depth of node
            position : int
                horizontal position of node

            Raises
            ------
            NodePositionError
                If the given position is not a valid binary tree node position.

            MissingNodeError
                If no node exists at the given position or a parent of that position.

        """

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

    def data_at_node(self, depth, position):
        node = self.get_node(depth, position)
        return node.data

    def is_height_balanced(self):
        """ Determine if tree is height-balanced.

            Returns
            -------
            height_balanced: bool
                True if for every node in the the tree, the difference in the height of the left and right subtrees is no greater than one.
        """

        return self.height_balance(self.root) > -1

    def height_balance(self, root):
        """ Return height of of tree if balanced, else -1.

            Parameters
            ----------
            root : Node instance
                The node from which to determine if the tree and its subtrees are balanced.

            Returns
            -------
            height_balance: int
                The height of the tree with the rooted at root, else -1.
        """

        if root is None:
            return 0

        left_height = self.height_balance(root.left)
        right_height = self.height_balance(root.right)
        if left_height == -1 or right_height == -1:
            return -1

        if abs(left_height - right_height) > 1:
            return -1

        return max(left_height, right_height) + 1

    def is_bst(self, root=False):
        root = self.root if not root else root
        if root is None:
            return True

        stack = [[-float('inf'), root, float('inf')]]
        while stack:
            min_val, root, max_val = stack.pop()
            if not (min_val < root.data < max_val):
                return False
            if root.left is not None:
                stack.append([min_val, root.left, root.data])
            if root.right is not None:
                stack.append([root.data, root.right, max_val])

        return True

class Node:
    
    def __init__(self, data):
        self.data = data
        self.left = None
        self.right = None

    def is_leaf(self):
        return self.left is None and self.right is None

    def add_left_child(self, node):
        if self.left is not None:
            raise NodePositionOccupiedError('Left node already exists')
        self.left = node

    def add_right_child(self, node):
        if self.right is not None:
            raise NodePositionOccupiedError('Right node already exists')
        self.right = node

