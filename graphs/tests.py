import math
import unittest

import graphs, exceptions, bfs, dfs, test_graphs
import shortest_paths as sp
import graph_properties as gp

class TestGraphFunctions(unittest.TestCase):

    def setUp(self):
        # Undirected, Acyclic, Weighted, Disconnected, Bipartite
        self.graph_a = test_graphs.graph_a
        # Undirected, Cyclic, Weighted, Disconnected, Not Bipartite
        self.graph_b = test_graphs.graph_b
        # Directed, Acyclic, Weighted, Disconnected, Bipartite
        self.graph_c = test_graphs.graph_c
        # Directed, Cyclic, Weighted, Disconnected, Not Bipartite
        self.graph_d = test_graphs.graph_d

    def test_bfs_shortest_unweighted_distance(self):
        """ Tests for accurate minimum unweighted distance from a specified
            starting vertex to a specified end vertex.
        """

        self.assertEqual(sp.shortest_unweighted_distance(self.graph_b, "A", "I"), 2)
        self.assertEqual(sp.shortest_unweighted_distance(self.graph_b, "A", "Z"), math.inf)
        self.assertEqual(sp.shortest_unweighted_distance(self.graph_a, "A", "I"), 3)
        self.assertEqual(sp.shortest_unweighted_distance(self.graph_a, "A", "Z"), math.inf)
        self.assertEqual(sp.shortest_unweighted_distance(self.graph_c, "A", "F"), 3)
        self.assertEqual(sp.shortest_unweighted_distance(self.graph_c, "A", "G"), math.inf)
        self.assertEqual(sp.shortest_unweighted_distance(self.graph_d, "A", "G"), 4)
        self.assertEqual(sp.shortest_unweighted_distance(self.graph_d, "A", "Z"), math.inf)

    def test_bfs_shortest_unweighted_path(self):
        """ Tests for accurate path finding using bfs. A graph may have more
            than one possible valid path between two vertices.
        """

        graph_b_path = sp.shortest_unweighted_path(self.graph_b, "A", "E")
        self.assertIn(graph_b_path, (['A', 'B', 'C', 'D', 'E'], ['A', 'H', 'G', 'F', 'E'], ['A', 'B', 'C', 'F', 'E']))
        self.assertIsNone(sp.shortest_unweighted_path(self.graph_b, "A", "Z"))
        self.assertEqual(sp.shortest_unweighted_path(self.graph_a, "A", "E"), ['A', 'B', 'C', 'F', 'E'])
        self.assertIsNone(sp.shortest_unweighted_path(self.graph_a, "A", "Z"))
        self.assertEqual(sp.shortest_unweighted_path(self.graph_c, "A", "F"), ['A', 'B', 'C', 'F'])
        self.assertIsNone(sp.shortest_unweighted_path(self.graph_c, "A", "Z"))
        self.assertEqual(sp.shortest_unweighted_path(self.graph_d, "A", "G"), ['A', 'B', 'H', 'I', 'G'])
        self.assertIsNone(sp.shortest_unweighted_path(self.graph_d, "A", "Z"))


    def test_bfs_has_undirected_cycle(self):
        """ Tests for bfs detection of cycle in undirected graph. Raises
            GraphTypeError if passed a directed graph.
        """

        self.assertTrue(gp.has_undirected_cycle(self.graph_b))
        self.assertFalse(gp.has_undirected_cycle(self.graph_a))
        with self.assertRaises(exceptions.GraphTypeError):
            gp.has_undirected_cycle(self.graph_c)
        with self.assertRaises(exceptions.GraphTypeError):
            gp.has_undirected_cycle(self.graph_d)


    def test_bfs_is_bipartite(self):
        """ Tests whether a graph is bipartite using bfs """

        self.assertFalse(gp.is_bipartite(self.graph_b))
        self.assertTrue(gp.is_bipartite(self.graph_a))
        self.assertTrue(gp.is_bipartite(self.graph_c))
        self.assertFalse(gp.is_bipartite(self.graph_d))


    def test_dfs_has_cycle(self):
        """ Tests dfs detection of cycle in a graph"""

        self.assertFalse(gp.has_cycle(self.graph_a))
        self.assertTrue(gp.has_cycle(self.graph_b))
        self.assertFalse(gp.has_cycle(self.graph_c))
        self.assertTrue(gp.has_cycle(self.graph_d))


    def test_topological_sort(self):
        """ Tests for accurate topological sorting of a graph.
            Should return None if topological sorting is not possible.
        """

        self.assertIsNone(gp.topological_sort(self.graph_b))
        self.assertIsNone(gp.topological_sort(self.graph_a))

        top_sort_graph_c = gp.topological_sort(self.graph_c)
        indices = {val: i for i, val in enumerate(top_sort_graph_c)}
        for vertex in ['B', 'C', 'D', 'F', 'H', 'I']:
            self.assertLess(indices['A'], indices[vertex])
        self.assertLess(indices['E'], indices['F'])
        self.assertLess(indices['G'], indices['H'])
        self.assertLess(indices['H'], indices['I'])
        self.assertLess(indices['Y'], indices['Z'])

        self.assertIsNone(gp.topological_sort(self.graph_d))


    def test_dijkstra_shortest_path(self):
        """ Tests for accurate shortest path from one vertex to another using
            Dijkstra's algorithm. Some graphs may have multiple equal-length
            shortest paths. Should return None if ending vertex is not
            reachable from the starting vertex.
        """

        self.assertEqual(sp.dijkstra_shortest_path(
            self.graph_b, 'A', 'E'), ['A', 'H', 'G', 'F', 'E'])
        self.assertEqual(sp.dijkstra_shortest_path(
            self.graph_b, 'A', 'I'), ['A', 'B', 'C', 'I'])
        self.assertIsNone(sp.dijkstra_shortest_path(
            self.graph_b, 'A', 'Z'))

    def test_dijkstra_shortest_distance(self):
        """ Tests for accurate shortest distance from one vertex to another
            using Dijkstra's algorithm. Should return infinite if ending vertex
            is not reachable from starting vertex.
        """

        self.assertEqual(sp.dijkstra_shortest_distance(
            self.graph_b, 'A', 'E'), 21)
        self.assertEqual(sp.dijkstra_shortest_distance(
            self.graph_b, 'A', 'I'), 14)
        self.assertEqual(sp.dijkstra_shortest_distance(
            self.graph_b, 'A', 'Z'), float('inf'))


if __name__ == "__main__":
    unittest.main()
