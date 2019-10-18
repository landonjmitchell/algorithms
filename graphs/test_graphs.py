from graphs import Graph


"""
Graph A:
Undirected, Acyclic, Weighted, Disconnected, Bipartite

    /---(4)---B---(8)---C---(7)---D
   /          |           \
  /         (11)           \--(4)--F--(10)--E
 /            |
A             |
              |  /--(7)--I
              | /
              H---(1)----G

Y---(5)---Z
"""

graph_a = Graph()
graph_a.add_edge('A', 'B', 4)
graph_a.add_edge('B', 'C', 8)
graph_a.add_edge('B', 'H', 11)
graph_a.add_edge('C', 'D', 7)
graph_a.add_edge('C', 'F', 4)
graph_a.add_edge('E', 'F', 10)
graph_a.add_edge('G', 'H', 1)
graph_a.add_edge('H', 'I', 7)
graph_a.add_edge('Y', 'Z', 5)

"""
Graph B:
Undirected, Cyclic, Weighted, Disconnected, Not Bipartite

    /---(4)---B---(8)---C---(7)---D---(9)---\
   /          |         | \       |          \
  /           |     (2)~|  \      |           \
 /            |         |   \     |            \
A        (11)~|   (7)---I   (4)   |~(14)        E
 \            |  /      |      \  |            /
  \           | /       |~(6)   \ |           /
   \          |/        |        \|          /
    \---(8)---H---(1)---G---(2)---F---(10)--/

Y---(5)---Z
"""

graph_b = Graph()
graph_b.add_edge('A', 'B', 4)
graph_b.add_edge('A', 'H', 8)
graph_b.add_edge('B', 'C', 8)
graph_b.add_edge('B', 'H', 11)
graph_b.add_edge('C', 'D', 7)
graph_b.add_edge('C', 'F', 4)
graph_b.add_edge('C', 'I', 2)
graph_b.add_edge('D', 'E', 9)
graph_b.add_edge('D', 'F', 14)
graph_b.add_edge('E', 'F', 10)
graph_b.add_edge('F', 'G', 2)
graph_b.add_edge('G', 'H', 1)
graph_b.add_edge('G', 'I', 6)
graph_b.add_edge('H', 'I', 7)
graph_b.add_edge('Y', 'Z', 5)


"""
Graph C:
Directed, Acyclic, Weighted, Disconnected, Bipartite

    /--(4)>>--B----(8)>>---C--(7)>>--D
   /          |             \
  /         (11)             \--(4)>>--F--<<(10)--E
 /            v                                  /
A             v                    /---(12)>>---/
              |  /--(7)>>-I       /
              | /                /
              H------<<(1)------G

Y--(5)>>--Z
"""

graph_c = Graph(directed=True)
graph_c.add_edge('A', 'B', 4)
graph_c.add_edge('B', 'C', 8)
graph_c.add_edge('B', 'H', 11)
graph_c.add_edge('C', 'D', 7)
graph_c.add_edge('C', 'F', 4)
graph_c.add_edge('E', 'F', 10)
graph_c.add_edge('G', 'H', 1)
graph_c.add_edge('G', 'E', 6)
graph_c.add_edge('H', 'I', 7)
graph_c.add_edge('Y', 'Z', 5)


"""
Graph D:
Directed, Cyclic, Weighted, Disconnected, Not Bipartite

    /--(4)>>--B----(8)>>---C--(7)>>--D
   /          |             \
  /          (11)            \--(4)>>--F--<<(10)--E
A             v
              v
              |  /--(7)>>-I--(6)>>
              | /                 \
              H------<<(1)---------G

Y--(5)>>--Z

"""

graph_d = Graph(directed=True)
graph_d.add_edge('A', 'B', 4)
graph_d.add_edge('B', 'C', 8)
graph_d.add_edge('B', 'H', 11)
graph_d.add_edge('C', 'D', 7)
graph_d.add_edge('C', 'F', 4)
graph_d.add_edge('E', 'F', 10)
graph_d.add_edge('G', 'H', 1)
graph_d.add_edge('H', 'I', 7)
graph_d.add_edge('I', 'G', 6)
graph_d.add_edge('Y', 'Z', 5)
