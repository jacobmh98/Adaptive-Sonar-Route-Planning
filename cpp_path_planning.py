import numpy as np
import cpp_path_intersections

def multi_intersection_planning(polygons, current_path_width, current_overlap_distance):
    """
    :param polygons: List of Polygons
    :param current_path_width: Float, chosen width of the planned path
    :param current_overlap_distance: Float, chosen overlap width
    :return: List of lists containing intersection points for each polygon.
    """
    # Creating the list to store intersections for each polygon
    total_intersections = []

    for i, current_poly in enumerate(polygons):
        # Computing boundary box for the polygon
        boundary_box = current_poly.compute_boundary()

        # Finding optimal vertices for the path planner
        b, b_mate, a = find_b_bmate_a_indicies(current_poly)

        # Computing path intersections
        intersections = cpp_path_intersections.get_path_intersections(current_poly, current_path_width, current_overlap_distance, b, b_mate, a, boundary_box)

        # Ensuring None cant be appended to the list
        if not intersections:
            intersections = []

        # Appending the current polygons intersections to the total list of intersections
        total_intersections.append(intersections)

    return total_intersections


def find_b_bmate_a_indicies(polygon):
    """ Finds the indices of the two vertices in the polygon that form the longest edge
    and the index of a third vertex that is furthest from the line defined by the two vertices.

    :param polygon: A Polygon object.
    :return: Tuple of indices (i, j, k) where:
             - i, j are the indices of the two vertices forming the longest edge.
             - k is the index of the vertex furthest from the line defined by i and j.
    """
    max_edge_length = 0
    longest_edge = (0, 0)
    vertices = polygon.vertices
    num_vertices = len(vertices)

    # Find the longest edge
    for i in range(num_vertices):
        j = (i + 1) % num_vertices  # Next vertex (neighbor)
        edge_length = np.linalg.norm(vertices[i].get_array() - vertices[j].get_array())
        if edge_length > max_edge_length:
            max_edge_length = edge_length
            longest_edge = (i, j)

    # Extract the two vertices forming the longest edge
    v1 = vertices[longest_edge[0]]
    v2 = vertices[longest_edge[1]]
    max_perpendicular_distance = 0
    furthest_point_index = None

    # Find the vertex furthest from the line defined by v1 and v2
    for k in range(num_vertices):
        if k == longest_edge[0] or k == longest_edge[1]:
            continue

        vk = vertices[k]

        # Perpendicular distance from vk to the line defined by v1 and v2
        line_vec = v2.get_array() - v1.get_array()
        point_vec = vk.get_array() - v1.get_array()
        line_length = np.linalg.norm(line_vec)
        if line_length > 0:
            perpendicular_distance = np.abs(np.cross(line_vec.flatten(), point_vec.flatten()) / line_length)
        else:
            perpendicular_distance = 0  # Degenerate case

        if perpendicular_distance > max_perpendicular_distance:
            max_perpendicular_distance = perpendicular_distance
            furthest_point_index = k

    return longest_edge[0], longest_edge[1], furthest_point_index


# Unused, but use for comparison
def rotating_calipers_path_planner(polygon, current_path_width, current_overlap_distance, d_pq, boundary_box):
    """ Algorithm 2: Rotating Calipers Path Planner.
    Computes the optimal back-and-forth path that covers a convex polygon efficiently by testing all antipodal pairs.

    :param polygon: Polygon
    :param current_path_width: Float path width for the current polygon
    :param d_pq: List of tuples representing antipodal pairs (b, a).
    :return optimal_path: The best back-and-forth path (computed using best_path).
    """
    # Initialize variables to store the best path and the minimal cost
    min_cost = float('inf')
    optimal_intersections = None

    # Iterate over all antipodal pairs (b, a) in the current polygon
    for (i, j) in d_pq:
        # Compute the best path for the current antipodal pair

        current_intersections = cpp_path_intersections.get_path_intersections(polygon, current_path_width, current_overlap_distance, i, j, boundary_box)

        # Simple cost function, can be improved
        current_cost = len(current_intersections)

        # Update the optimal path if the current path has a lower cost
        if current_cost < min_cost:
            min_cost = current_cost
            optimal_intersections = current_intersections

    return optimal_intersections