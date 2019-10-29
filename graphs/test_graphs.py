from graph import Graph


r"""
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


r"""
Graph A2:
Undirected, Acyclic, Weighted, Connected, Bipartite

    /---(4)---B---(8)---C---(7)---D
   /          |           \
  /         (11)           \--(4)--F--(10)--E
 /            |
A             |
              |  /--(7)--I
              | /
              H---(1)----G
"""

graph_a2 = Graph()
graph_a2.add_edge('A', 'B', 4)
graph_a2.add_edge('B', 'C', 8)
graph_a2.add_edge('B', 'H', 11)
graph_a2.add_edge('C', 'D', 7)
graph_a2.add_edge('C', 'F', 4)
graph_a2.add_edge('E', 'F', 10)
graph_a2.add_edge('G', 'H', 1)
graph_a2.add_edge('H', 'I', 7)


r"""
Graph A3:
Undirected, Acyclic, Unweighted, Disconnected, Bipartite

    /---------B---------C---------D
   /          |           \
  /           |            \-------F--------E
 /            |
A             |
              |  /-------I
              | /
              H----------G

Y---------Z
"""

graph_a3 = Graph()
graph_a3.add_edge('A', 'B')
graph_a3.add_edge('B', 'C')
graph_a3.add_edge('B', 'H')
graph_a3.add_edge('C', 'D')
graph_a3.add_edge('C', 'F')
graph_a3.add_edge('E', 'F')
graph_a3.add_edge('G', 'H')
graph_a3.add_edge('H', 'I')
graph_a3.add_edge('Y', 'Z')


r"""
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


r"""
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

graph_c = Graph(is_directed=True)
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


r"""
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

graph_d = Graph(is_directed=True)
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


r"""
Graph D2:
Directed, Cyclic, Weighted, Connected, Not Bipartite

    /--(4)>>--B----(8)>>---C--(7)>>--D--(1)>>
   /          |             \               /
  /          (11)            \--(4)>>--F --/
A             v                        |
  \           v                       (3)
   \          |  /--(7)>>-I--(6)>>     v
    \         | /                 \    v
     \-<<(8)--H------<<(1)---------G--/
"""

graph_d2 = Graph(is_directed=True)
graph_d2.add_edge('A', 'B', 4)
graph_d2.add_edge('B', 'C', 8)
graph_d2.add_edge('B', 'H', 11)
graph_d2.add_edge('C', 'D', 7)
graph_d2.add_edge('C', 'F', 4)
graph_d2.add_edge('D', 'F', 1)
graph_d2.add_edge('F', 'G', 3)
graph_d2.add_edge('G', 'H', 1)
graph_d2.add_edge('H', 'A', 8)
graph_d2.add_edge('H', 'I', 7)
graph_d2.add_edge('I', 'G', 6)


r"""
Graph D3:
Directed, Cyclic, Weighted, Connected, Not Bipartite, Negative Cycle

    /--(4)>>--B----(8)>>---C--(7)>>--D--(1)>>
   /          |             \               /
  /          (11)            \--(4)>>--F --/
A             v                        |
  \           v                       (3)
   \          |  /--(-7)>>-I--(6)>>     v
    \         | /                 \    v
     \-<<(8)--H------<<(1)---------G--/
"""

graph_d3 = Graph(is_directed=True)
graph_d3.add_edge('A', 'B', 4)
graph_d3.add_edge('B', 'C', 8)
graph_d3.add_edge('B', 'H', 11)
graph_d3.add_edge('C', 'D', 7)
graph_d3.add_edge('C', 'F', 4)
graph_d3.add_edge('D', 'F', 1)
graph_d3.add_edge('F', 'G', 3)
graph_d3.add_edge('G', 'H', 1)
graph_d3.add_edge('H', 'A', 8)
graph_d3.add_edge('H', 'I', -7)
graph_d3.add_edge('I', 'G', 6)
