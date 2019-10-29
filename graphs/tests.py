import math
import unittest

import graphs, exceptions, test_graphs

class TestGraphFunctions(unittest.TestCase):

    def setUp(self):
        # Undirected, Acyclic, Weighted, Disconnected, Bipartite
        self.graph_a = test_graphs.graph_a
        # Undirected, Acyclic, Weighted, Disconnected, Bipartite
        self.graph_a2 = test_graphs.graph_a2
        # Undirected, Acyclic, Unweighted, Disconnected, Bipartite
        self.graph_a3 = test_graphs.graph_a3
        # Undirected, Cyclic, Weighted, Connected, Not Bipartite
        self.graph_b = test_graphs.graph_b
        # Directed, Acyclic, Weighted, Disconnected, Bipartite
        self.graph_c = test_graphs.graph_c
        # Directed, Cyclic, Weighted, Disconnected, Not Bipartite
        self.graph_d = test_graphs.graph_d
        # Directed, Cyclic, Weighted, Connected, Not Bipartite
        self.graph_d2 = test_graphs.graph_d2

    def test_is_bipartite(self):
        """ Tests whether a graph is bipartite using bfs
        """

        self.assertTrue(self.graph_a.is_bipartite)
        self.assertFalse(self.graph_b.is_bipartite)
        self.assertTrue(self.graph_c.is_bipartite)
        self.assertFalse(self.graph_d.is_bipartite)

    def test_has_cycle(self):
        """ Tests dfs detection of cycle in a graph
        """

        self.assertFalse(self.graph_a.has_cycle)
        self.assertTrue(self.graph_b.has_cycle)
        self.assertFalse(self.graph_c.has_cycle)
        self.assertTrue(self.graph_d.has_cycle)

    def test_is_strongly_connected(self):
        """ Tests if graph is (strongly) connected
        """

        self.assertFalse(self.graph_a.is_strongly_connected)
        self.assertTrue(self.graph_a2.is_strongly_connected)
        self.assertFalse(self.graph_d.is_strongly_connected)
        self.assertTrue(self.graph_a2.is_strongly_connected)

    def test_connected_components(self):
        """ Tests accurate grouping of vertices in the connected components
            of a graph.
        """

        a_cc = {frozenset({'F', 'G', 'H', 'A', 'C', 'D', 'E', 'B', 'I'}),
                frozenset({'Z', 'Y'})}
        a2_cc = {frozenset({'G', 'I', 'H', 'B', 'E', 'C', 'A', 'F', 'D'})}
        d_cc = {frozenset({'Y'}), frozenset({'I', 'G', 'H'}), frozenset({'Z'}),
                frozenset({'A'}), frozenset({'F'}), frozenset({'B'}),
                frozenset({'E'}), frozenset({'D'}), frozenset({'C'})}
        d2_cc = {frozenset({'B', 'F', 'G', 'H', 'D', 'C', 'I', 'A'})}

        self.assertEqual(self.graph_a.connected_components, a_cc)
        self.assertEqual(self.graph_a2.connected_components, a2_cc)
        self.assertEqual(self.graph_d.connected_components, d_cc)
        self.assertEqual(self.graph_d2.connected_components, d2_cc)

    def test_topological_sort(self):
        """ Tests for accurate topological sorting of a graph.
            Should return None if topological sorting is not possible.
        """

        self.assertIsNone(self.graph_a.topological_sort)
        self.assertIsNone(self.graph_b.topological_sort)

        top_sort_graph_c = self.graph_c.topological_sort
        indices = {val: i for i, val in enumerate(top_sort_graph_c)}
        for vertex in ['B', 'C', 'D', 'F', 'H', 'I']:
            self.assertLess(indices['A'], indices[vertex])
        self.assertLess(indices['E'], indices['F'])
        self.assertLess(indices['G'], indices['H'])
        self.assertLess(indices['H'], indices['I'])
        self.assertLess(indices['Y'], indices['Z'])

        self.assertIsNone(self.graph_d.topological_sort)

    def test_shortest_paths(self):
        a_paths = {'Z': None, 'D': ['A', 'B', 'C', 'D'], 'C': ['A', 'B', 'C'],
                   'A': ['A'], 'F': ['A', 'B', 'C', 'F'], 'Y': None, 
                   'B': ['A', 'B'], 'G': ['A', 'B', 'H', 'G'], 
                   'I': ['A', 'B', 'H', 'I'], 'H': ['A', 'B', 'H'], 
                   'E': ['A', 'B', 'C', 'F', 'E']}

        a3_paths = {'B': ['A', 'B'], 'F': ['A', 'B', 'C', 'F'], 
                    'H': ['A', 'B', 'H'], 'Z': ['Z'], 'A': ['A'], 
                    'Y': ['Y'], 'D': ['A', 'B', 'C', 'D'], 
                    'I': ['A', 'B', 'H', 'I'], 'E': ['A', 'B', 'C', 'F', 'E'], 'C': ['A', 'B', 'C'], 'G': ['A', 'B', 'H', 'G']}

        b_paths = {'Z': None, 'C': ['A', 'B', 'C'], 'I': ['A', 'B', 'C', 'I'],
                   'F': ['A', 'H', 'G', 'F'], 'Y': None, 'G': ['A', 'H', 'G'],
                   'B': ['A', 'B'], 'D': ['A', 'B', 'C', 'D'], 
                   'E': ['A', 'H', 'G', 'F', 'E'], 'H': ['A', 'H'], 'A': ['A']}

        c_paths = {'E': None, 'C': ['A', 'B', 'C'], 'H': ['A', 'B', 'H'], 
                   'F': ['A', 'B', 'C', 'F'], 'G': None, 'A': ['A'], 'Y': None, 'I': ['A', 'B', 'H', 'I'], 'Z': None, 'B': ['A', 'B'],
                   'D': ['A', 'B', 'C', 'D']}

        self.assertEqual(self.graph_a.shortest_paths('A'), a_paths)
        self.assertEqual(self.graph_a3.shortest_paths('A'), a3_paths)
        self.assertEqual(self.graph_b.shortest_paths('A'), b_paths)
        self.assertEqual(self.graph_c.shortest_paths('A'), c_paths)

if __name__ == "__main__":
    unittest.main()
