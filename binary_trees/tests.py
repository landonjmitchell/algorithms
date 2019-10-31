import unittest

from test_trees import (tree_a, tree_b, tree_c, tree_d, tree_e)
from binary_tree import BinaryTree, Node
from exceptions import MissingNodeError, NodePositionOccupiedError, NodePositionError

class TestBinaryTreeFunctions(unittest.TestCase):

    def test_add_get_node(self):
        """ Tests adding and retrieving nodes at specific positions.
        """

        self.test_node_1 = Node(99)
        tree_a.add_node(self.test_node_1, 2, 0)
        node = tree_a.get_node(2, 0)
        self.assertTrue(node is self.test_node_1)

        with self.assertRaises(MissingNodeError):
            tree_a.get_node(2, 1)
            tree_a.add_node(Node(77), 5, 1)

        with self.assertRaises(NodePositionError):
            tree_a.get_node(-1, 0)
            tree_a.add_node(Node(66), 2, 3)
            tree_a.add_node(Node(55), 4, -1)

        # removed added node
        tree_a.remove_node(2, 0)

    def test_data_at_node(self):
        """ Tests returning the data of a node at a specific position.
        """

        self.assertTrue(tree_a.data_at_node(0, 0), 5)
        self.assertTrue(tree_a.data_at_node(1, 0), 3)
        self.assertTrue(tree_a.data_at_node(1, 1), 8)

        with self.assertRaises(MissingNodeError):
            tree_a.data_at_node(3, 0)
            tree_a.data_at_node(2, 1)

        with self.assertRaises(NodePositionError):
            tree_a.data_at_node(-1, 0)
            tree_a.data_at_node(2, 3)
            tree_a.data_at_node(4, -1)

    def test_height(self):
        """ Tests accurate height data of a tree.
        """

        self.assertEqual(tree_a.height, 1)
        self.assertEqual(tree_b.height, 4)
        self.assertEqual(tree_d.height, 3)
        self.assertEqual(tree_e.height, 3)


    def test_is_height_balanced(self):
        """ Tests determining if binary tree is height_balanced.
        """
        
        self.assertTrue(tree_a.is_height_balanced())
        self.assertFalse(tree_b.is_height_balanced())
        self.assertFalse(tree_c.is_height_balanced())
        self.assertFalse(tree_d.is_height_balanced())
        self.assertTrue(tree_e.is_height_balanced())


    def test_is_bst(self):
        """ Tests determining if binary tree is a binary search tree.
        """

        self.assertTrue(tree_a.is_bst())
        self.assertTrue(tree_b.is_bst())
        self.assertFalse(tree_c.is_bst())
        self.assertTrue(tree_d.is_bst())
        self.assertFalse(tree_e.is_bst())
    
class TestNodeFunctions(unittest.TestCase):

    def test_add_children(self):
        """ Tests adding children to a node.
        """

        parent_node = tree_a.get_node(1, 0)
        new_node_left = Node('left')
        new_node_right = Node('right')

        parent_node.add_left_child(new_node_left)
        child_node_left = tree_a.get_node(2, 0)
        self.assertTrue(new_node_left is child_node_left)

        parent_node.add_right_child(new_node_right)
        child_node_right = tree_a.get_node(2, 1)
        self.assertTrue(new_node_right is child_node_right)

        with self.assertRaises(NodePositionOccupiedError):
            parent_node.add_left_child(new_node_left)
            parent_node.add_right_child(new_node_right)

if __name__ == "__main__":
    unittest.main()
