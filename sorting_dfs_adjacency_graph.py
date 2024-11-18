import networkx as nx
import numpy as np
import decomposition

def create_adjacency(polygons):
    # Creating adjacent matrix for the sub-polygons to tell which sub-polygons are connected
    m = len(polygons)
    A = np.zeros((m, m))
    G = nx.Graph()

    # Go through each edge in p_i and compare with each edge in p_j
    for i, p_i in  enumerate(polygons):
        for j, p_j in enumerate(polygons):
            # Ignore if the two polygons are equal
            if i == j:
                A[i, j] = 0
                continue

            # Test if the two polygons p_i and p_j are adjacent (either complete or partial)
            if decomposition.polygons_are_adjacent(p_i, p_j, i, j):
                # Update the adjacent matrix
                A[i, j] = 1
                A[j, i] = 1
                G.add_edge(f'P{i}', f'P{j}')
            else:
                A[i, j] = np.inf
                #print(f'{i} and {j} are adjacent')
    #nx.draw(G, with_labels=True, node_color='lightblue', edge_color='gray', node_size=2000, font_size=15)
    return A,G

def solve_dfs(polygons, intersections):
    """ Use DFS to order the sub-polygons based on the adjacency graph

    :param polygons: List of polygons
    :param intersections: List of intersections
    :return sorted_polys: The sorted list of polygons
    """

    adjacency_matrix, adjacency_graph = create_adjacency(polygons)
    start = next(iter(adjacency_graph.nodes()))

    # Perform DFS on the graph starting from the specified start node
    dfs_order = list(nx.dfs_preorder_nodes(adjacency_graph, start))

    # Convert the node labels back to polygon indices
    dfs_route = [int(node[1:]) for node in dfs_order]

    # Reorder the polygons based on the DFS traversal order
    sorted_polys = [polygons[i] for i in dfs_route]
    sorted_inters = [intersections[i] for i in dfs_route]

    return sorted_polys, sorted_inters