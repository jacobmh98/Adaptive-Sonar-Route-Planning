import networkx as nx
import numpy as np
import polygon_coverage_intersections
import antipodal_pairs
from decomposition import polygons_are_adjacent
from global_variables import *
from Polygon import Polygon, Vertex

def get_nearest_neighbour_vertex(poly1, poly2):
    nearest_vertex = None
    min_distance = float('inf')

    # Loop over all vertices in poly1 and poly2
    for v1 in poly1.vertices:
        for v2 in poly2.vertices:
            # Calculate the Euclidean distance between the two vertices
            distance = np.linalg.norm(v1.v - v2.v)

            # Check if this is the closest vertex found so far
            if distance < min_distance:
                min_distance = distance
                nearest_vertex = v2  # Nearest vertex in poly2 is a shared vertex with poly1

    return nearest_vertex

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
            if polygons_are_adjacent(p_i, p_j, i, j):
                # Update the adjacent matrix
                A[i, j] = 1
                A[j, i] = 1
                G.add_edge(f'P{i}', f'P{j}')
            else:
                A[i, j] = np.inf
                #print(f'{i} and {j} are adjacent')
    return A,G

def sort_sub_polygons_using_dfs(G, polygons, start):
    """ Use DFS to order the sub-polygons based on the adjacency graph

    :param G: nx graph
    :param polygons: List of polygons
    :param start: string, start node
    :return sorted_polys: The sorted list of polygons
    """
    # Perform DFS on the graph starting from the specified start node
    dfs_order = list(nx.dfs_preorder_nodes(G, start))

    # Convert the node labels back to polygon indices
    dfs_order_indices = [int(node[1:]) for node in dfs_order]

    # Get the DFS-based order of polygons
    ordered_polys = dfs_order_indices

    # Reorder the polygons based on the DFS traversal order
    sorted_polys = [polygons[i] for i in ordered_polys]

    return sorted_polys

def remove_unnecessary_vertices(polygon):
    """Remove vertices that form a straight line with their neighbors.

    :param polygon: Polygon object
    :param epsilon: Tolerance for floating point comparisons
    :return: A new polygon with unnecessary vertices removed
    """
    new_vertices = []
    index = 0  # Start index for the new vertices

    for i in range(len(polygon.vertices)):
        v_prev = polygon.vertices[i - 1]
        v_curr = polygon.vertices[i]
        v_next = polygon.vertices[(i + 1) % len(polygon.vertices)]

        # Vector from previous vertex to current vertex
        vector1 = np.array([v_curr.x - v_prev.x, v_curr.y - v_prev.y, 0])  # Add 3rd dimension as 0
        # Vector from current vertex to next vertex
        vector2 = np.array([v_next.x - v_curr.x, v_next.y - v_curr.y, 0])  # Add 3rd dimension as 0

        # Compute the cross product and only take the z-component
        cross_product = np.cross(vector1, vector2)[-1]

        # If cross product is not zero (within epsilon), the vertex is necessary
        if abs(cross_product) > optimize_epsilon:
            # Assign the new index to the vertex
            new_vertex = Vertex(index, v_curr.x, v_curr.y)
            new_vertices.append(new_vertex)
            index += 1

    # Return a new polygon with filtered vertices
    return Polygon(new_vertices)


def rotating_calipers_path_planner(polygon, current_path_width, current_polygon_index, d_pq):
    """ Algorithm 2: Rotating Calipers Path Planner.
    Computes the optimal back-and-forth path that covers a convex polygon efficiently by testing all antipodal pairs.

    :param polygon: Polygon
    :param current_path_width: Float path width for the current polygon
    :param current_polygon_index: index of current polygon
    :param d_pq: List of tuples representing antipodal pairs (b, a).
    :return optimal_path: The best back-and-forth path (computed using best_path).
    """
    # Initialize variables to store the best path and the minimal cost
    min_cost = float('inf')
    optimal_intersections = None

    # Iterate over all antipodal pairs (b, a)
    for (i, j) in d_pq:
        # Compute the best path for the current antipodal pair
        current_intersections = polygon_coverage_intersections.best_intersection(polygon, current_path_width, i, j)

        # TODO: Create a better cost function
        current_cost = len(current_intersections)

        # Update the optimal path if the current path has a lower cost
        if current_cost < min_cost:
            min_cost = current_cost
            optimal_intersections = current_intersections

    return optimal_intersections


def multi_intersection_planning(polygons, current_path_width):
    """
    :param polygons: List of Polygons
    :param current_path_width: Width of the planned path
    :return: List of lists containing intersection points for each polygon.
    """
    # Creating the list to store intersections for each polygon
    total_intersections = []

    for i, current_poly in enumerate(polygons):
        # Computing current polygon's antipodal points
        antipodal_vertices = antipodal_pairs.compute_antipodal_pairs(current_poly)

        # Removing neighbor pairs and duplicate pairs
        filtered_antipodal_vertices = antipodal_pairs.filter_and_remove_redundant_pairs(
            current_poly, antipodal_vertices
        )

        # Computing the diametric antipodal pairs (minimizing number of paths)
        diametric_antipodal_pairs = antipodal_pairs.filter_diametric_antipodal_pairs(
            current_poly, filtered_antipodal_vertices
        )

        # Computing the intersections for the current polygon
        intersections = rotating_calipers_path_planner(current_poly, current_path_width, i, diametric_antipodal_pairs)

        # Check if intersections found, otherwise initialize an empty list
        if not intersections:
            intersections = []

        total_intersections.append(intersections)

    return total_intersections