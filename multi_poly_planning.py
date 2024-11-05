import networkx as nx
import numpy as np
import polygon_coverage_path
import rotating_calipers_antipodal_pairs
import matplotlib.pyplot as plt
from functions import polygons_are_adjacent
from global_variables import ext_p_start, ext_p_end
from Polygon import Polygon

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

def remove_unnecessary_vertices(polygon, epsilon=1e-9):
    """Remove vertices that form a straight line with their neighbors.

    :param polygon: Polygon object
    :param epsilon: Tolerance for floating point comparisons
    :return: A new polygon with unnecessary vertices removed
    """
    new_vertices = []
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
        if abs(cross_product) > epsilon:
            v_curr.index = len(new_vertices)
            new_vertices.append(v_curr)

    # Return a new polygon with filtered vertices
    return Polygon(new_vertices)

def rotating_calipers_path_planner(polygons, current_polygon_index, path, dx, d_pq):
    """ Algorithm 2: Rotating Calipers Path Planner.
    Computes the optimal back-and-forth path that covers a convex polygon efficiently by testing all antipodal pairs.

    :param path:
    :param polygons: List Polygon
    :param current_polygon_index: index of current polygon
    :param dx: float, the path width
    :param d_pq: List of tuples representing antipodal pairs (b, a).
    :return optimal_path: The best back-and-forth path (computed using best_path).
    """
    # Initialize variables to store the best path and the minimal cost
    min_cost = float('inf')
    optimal_path = None

    # Iterate over all antipodal pairs (b, a)
    for (i, j) in d_pq:
        # Compute the best path for the current antipodal pair
        current_path, current_cost = polygon_coverage_path.best_path(polygons, current_polygon_index, path, dx, i, j)

        # Update the optimal path if the current path has a lower cost
        if current_cost < min_cost:
            min_cost = current_cost
            optimal_path = current_path

    return optimal_path

def multi_path_planning(polygons, dx, include_external_start_end):
    """
    :param polygons: List of Polygons
    :param dx: float, path width
    :param include_external_start_end: Bool, indicate if external start and end point included
    :return:
    """
    # Creating the np array to store the total path
    total_path = np.empty((0,2))

    if include_external_start_end:
        # Appending external start point to path
        total_path = np.append(total_path, [ext_p_start], axis=0)

    for i, current_poly in enumerate(polygons):
        b_index = 0  # Always start at first vertex in the current polygon

        # Computing current polygons antipodal points
        antipodal_vertices = rotating_calipers_antipodal_pairs.compute_antipodal_pairs(current_poly)
        # Removes neighbour pairs and double pairs, i.e. for [0,1] and [1,0] only 1 of them is necessary as both [1,0] and [0,1] is checked in best path algorithm
        filtered_antipodal_vertices = rotating_calipers_antipodal_pairs.filter_and_remove_redundant_pairs(current_poly, antipodal_vertices)
        # Computing the diametric antipodal pairs (Minimizing number of paths computed, as diametric pairs produce the shortest paths)
        diametric_antipodal_pairs = rotating_calipers_antipodal_pairs.filter_diametric_antipodal_pairs(current_poly, filtered_antipodal_vertices)
        # Getting index of point a (the diametric antipodal point of b)
        # diametric_antipode_index = rotating_calipers_antipodal_pairs.get_diametric_antipodal_point_index(diametric_antipodal_pairs, b_index)

        shortest_path = rotating_calipers_path_planner(polygons, i, total_path, dx, diametric_antipodal_pairs)

        if shortest_path.size == 0:  # Probably not necessary
            shortest_path = np.empty((0, 2))  # Resetting shortest_path for total_path

        total_path = np.vstack([total_path, shortest_path])

    # Appending the external end point as last point in path
    if include_external_start_end:
        total_path = np.append(total_path, [ext_p_end], axis=0)

    return total_path

def multi_poly_plot(polygon, polygons, dx, include_external_start_end, ps, pe, path):
    """
    Plot multiple polygons, the path between the polygons, and the start/end points of the mission.

    :param polygon: Polygon
    :param polygons: List of the sub polygons
    :param include_external_start_end: bool
    :param path: NumPy array, array of points representing the path [[x1, y1], [x2, y2], ...]
    :param ps: Starting point of the mission
    :param pe: Ending point of the mission
    :param dx: float, the distance by which the vector should be offset (this defines the width of coverage)
    """
    coverage = False
    plot_sub_polygons = True

    fig, ax = plt.subplots(1, 1)
    color = "k"
    # Plot the polygon
    if not plot_sub_polygons:
        x_coords, y_coords = polygon.get_coords()
        ax.plot(x_coords, y_coords, f'{color}-', marker='o')
        ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], f'{color}-')
        ax.set_aspect('equal')

    if plot_sub_polygons:
        for i, poly in enumerate(polygons):
            x_coords, y_coords = poly.get_coords()
            ax.plot(x_coords, y_coords, 'k-', marker='o',markersize=3, label='Polygon')
            ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'k-')

            # Find the center of the polygon to place the label
            centroid_x = np.mean(x_coords)
            centroid_y = np.mean(y_coords)

            # Label the polygon with its number (the order number)
            ax.text(centroid_x, centroid_y, f'{i}', fontsize=10, color='blue')

    # Plot the path
    if len(path) > 0:
        path_x, path_y = path[:, 0], path[:, 1]
        ax.plot(path_x, path_y, 'r-', label='Path', linewidth=1)

        # Highlight the start and end points of the path
        ax.plot(path_x[0], path_y[0], 'go', markersize=8, label='Start Point')  # Start point
        ax.plot(path_x[-1], path_y[-1], 'yo', markersize=8, label='End Point')  # End point

        # Compute and plot coverage area along the path
        if coverage:
            for i in range(len(path) - 1):
                p1 = path[i]
                p2 = path[i + 1]
                # Vector along the path segment
                segment_vector = p2 - p1
                # Normalize the vector to get the perpendicular direction
                perp_vector = np.array([-segment_vector[1], segment_vector[0]])
                if np.linalg.norm(perp_vector) != 0:
                    perp_vector = perp_vector / np.linalg.norm(perp_vector) * dx / 2

                # Create four corners of the coverage area for this segment
                corner1 = p1 + perp_vector
                corner2 = p1 - perp_vector
                corner3 = p2 - perp_vector
                corner4 = p2 + perp_vector

                # Plot the coverage area for this segment as a filled polygon
                ax.fill([corner1[0], corner2[0], corner3[0], corner4[0]],
                        [corner1[1], corner2[1], corner3[1], corner4[1]],
                        'orange', alpha=0.3, label='_nolegend_')

    else:
        print("Empty path")

    # Plot the mission start and end points
    if include_external_start_end:
        ax.plot(ps[0], ps[1], 'bo', markersize=12, label='Mission Start Point')
        ax.text(ps[0], ps[1], 'Start', fontsize=14, color='blue')

        ax.plot(pe[0], pe[1], 'mo', markersize=12, label='Mission End Point')
        ax.text(pe[0], pe[1], 'End', fontsize=14, color='magenta')

    plt.grid(True)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()