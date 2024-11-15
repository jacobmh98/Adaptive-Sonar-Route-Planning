import networkx as nx
import numpy as np
import decomposition
import cpp_path_intersections
import cpp_antipodal_pairs


def rotating_calipers_path_planner(polygon, current_path_width, current_overlap_distance, d_pq):
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
        current_intersections = cpp_path_intersections.best_intersection(polygon, current_path_width, current_overlap_distance, i, j)

        # TODO: Create a better cost function?
        current_cost = len(current_intersections)

        # Update the optimal path if the current path has a lower cost
        if current_cost < min_cost:
            min_cost = current_cost
            optimal_intersections = current_intersections

    return optimal_intersections


def multi_intersection_planning(polygons, current_path_width, current_overlap_distance):
    """
    :param polygons: List of Polygons
    :param current_path_width: Width of the planned path
    :return: List of lists containing intersection points for each polygon.
    """
    # Removing collinear vertices before planning intersections
    #removed_col_sub_polygons = []
    ##for poly in polygons:
     #   removed_col_sub_polygons.append(decomposition.remove_collinear_vertices(poly))

    # Creating the list to store intersections for each polygon
    total_intersections = []

    for i, current_poly in enumerate(polygons):
        # Computing current polygon's antipodal points
        antipodal_vertices = cpp_antipodal_pairs.compute_antipodal_pairs(current_poly)

        # Removing neighbor pairs and duplicate pairs
        filtered_antipodal_vertices = cpp_antipodal_pairs.filter_and_remove_redundant_pairs(
            current_poly, antipodal_vertices
        )

        # Computing the diametric antipodal pairs (minimizing number of paths)
        diametric_antipodal_pairs = cpp_antipodal_pairs.filter_diametric_antipodal_pairs(
            current_poly, filtered_antipodal_vertices
        )

        # Computing the intersections for the current polygon
        #intersections = rotating_calipers_path_planner(current_poly, current_path_width, diametric_antipodal_pairs, total_intersections)
        intersections = rotating_calipers_path_planner(current_poly, current_path_width, current_overlap_distance, filtered_antipodal_vertices)

        # Check if intersections found, otherwise initialize an empty list
        if not intersections:
            intersections = []

        total_intersections.append(intersections)

    return total_intersections