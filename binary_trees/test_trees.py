from binary_tree import BinaryTree, Node

r"""

Tree A: 
Balanced, BST

   5
  / \
 3   9

"""

tree_a = BinaryTree()
tree_a.add_node(Node(5), 0, 0)
tree_a.add_node(Node(3), 1, 0)
tree_a.add_node(Node(8), 1, 1)

r"""

Tree B:
Not Strictly Balanced, BST

       ___5___
      /       \
    _2_       _9_
   /   \     /   \
  1     4   6     11
 /     /     \      \
0     3       7      12
               \
                8

"""
tree_b = BinaryTree()
tree_b.add_node(Node(5), 0, 0)
tree_b.add_node(Node(2), 1, 0)
tree_b.add_node(Node(9), 1, 1)
tree_b.add_node(Node(1), 2, 0)
tree_b.add_node(Node(4), 2, 1)
tree_b.add_node(Node(6), 2, 2)
tree_b.add_node(Node(11), 2, 3)
tree_b.add_node(Node(0), 3, 0)
tree_b.add_node(Node(3), 3, 2)
tree_b.add_node(Node(7), 3, 5)
tree_b.add_node(Node(8), 4, 11)
tree_b.add_node(Node(12), 3, 7)


r"""

Tree C:
Not Balanced, BST

       ___5___
      /       \
    _2_       _9_
   /   \     /   \
  1     4   6     11
 /     /     \      
0     3       7      
               \
                8

"""
tree_c = BinaryTree()
tree_c.add_node(Node(5), 0, 0)
tree_c.add_node(Node(2), 1, 0)
tree_c.add_node(Node(9), 1, 1)
tree_c.add_node(Node(1), 2, 0)
tree_c.add_node(Node(4), 2, 1)
tree_c.add_node(Node(6), 2, 2)
tree_c.add_node(Node(11), 2, 3)
tree_c.add_node(Node(0), 3, 0)
tree_c.add_node(Node(3), 3, 2)
tree_c.add_node(Node(7), 3, 5)
tree_c.add_node(Node(8), 4, 11)


