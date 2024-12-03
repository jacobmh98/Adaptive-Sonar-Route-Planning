import numpy as np
import cpp_connect_path

def get_pairs(intersections):
    """
    Generate start-end pairs for a path based on intersections.

    :param intersections: List of intersection points
    :return: List of start-end pairs
    """
    first_pair = (intersections[0][0], intersections[0][1])
    path = cpp_connect_path.connect_path_for_tsp(first_pair, intersections)
    all_pairs = [
        (path[0], path[-1]),
        (path[1], path[-2]),
        (path[-2], path[1]),
        (path[-1], path[0])
    ]

    return all_pairs


import math


def euclidean_distance(p1, p2):
    """Compute Euclidean distance between two points."""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def compute_distances(subpolygons):
    """
    Compute distances between end points of one subpolygon
    and start points of the next subpolygon.

    Args:
        subpolygons: List of subpolygons, each with 4 start/end pairs.

    Returns:
        distances: Nested dictionary of distances between pairs.
    """
    distances = {}

    # Iterate through subpolygons
    for n in range(len(subpolygons) - 1):
        distances[n] = {}  # Initialize dictionary for subpolygon n

        current_pairs = subpolygons[n]
        next_pairs = subpolygons[n + 1]

        # Compute distances between all pairs
        for i, (_, end_point) in enumerate(current_pairs):  # End points of current subpolygon
            for j, (start_point, _) in enumerate(next_pairs):  # Start points of next subpolygon
                distances[n][(i, j)] = euclidean_distance(end_point, start_point)

    return distances


def solve_greedy_tsp_sorting(polygons, all_intersections):
    """
    Solve the TSP by testing all start pairs and iteratively finding the closest path.

    Args:
        polygons: List of polygons.
        all_intersections: List of intersections for each polygon.

    Returns:
        Best sorted polygons, intersections, and total distance.
    """

    # Step 1: Generate start/end pairs for each polygon
    all_polygon_start_end_pairs = [get_pairs(intersections) for intersections in all_intersections]
    print(f"all_polygon_start_end_pairs: {all_polygon_start_end_pairs}")

    # Initialize variables to track the best path
    best_path = None
    best_total_distance = float('inf')
    best_sorted_polygons = None
    best_sorted_col_removed_polygons = None
    best_sorted_intersections = None

    # Step 2: Test all 4 starting pairs for the first polygon
    for start_pair_index in range(4):  # 4 pairs in the first polygon
        path = [(0, start_pair_index)]  # Start with the first polygon and this pair
        total_distance = 0
        current_pair_index = start_pair_index
        current_end_point = all_polygon_start_end_pairs[0][start_pair_index][1]  # End point of the current pair

        visited = set()  # Keep track of visited polygons
        visited.add(0)  # Mark the first polygon as visited

        # Step 3: Iteratively find the closest pair for all other polygons
        while len(visited) < len(all_polygon_start_end_pairs):
            # Find the next closest polygon and pair
            next_polygon_index = None
            next_pair_index = None
            min_distance = float('inf')

            for n in range(len(all_polygon_start_end_pairs)):
                if n in visited:
                    continue  # Skip already visited polygons

                for j in range(4):  # Test all 4 pairs in the current polygon
                    start_point = all_polygon_start_end_pairs[n][j][0]  # Start point of the pair
                    distance = euclidean_distance(current_end_point, start_point)

                    if distance < min_distance:
                        min_distance = distance
                        next_polygon_index = n
                        next_pair_index = j

            # Update the path and total distance
            path.append((next_polygon_index, next_pair_index))
            total_distance += min_distance
            current_end_point = all_polygon_start_end_pairs[next_polygon_index][next_pair_index][1]  # Update end point
            visited.add(next_polygon_index)  # Mark as visited

        # Step 4: Check if this path is the best
        if total_distance < best_total_distance:
            best_total_distance = total_distance
            best_path = path

            # Sort polygons and intersections based on the best path
            best_sorted_polygons = [polygons[p[0]] for p in path]
            best_sorted_col_removed_polygons = [polygons[p[0]] for p in path]
            best_sorted_intersections = [all_intersections[p[0]] for p in path]

    print(f"Best path: {best_path}, Total Distance: {best_total_distance}")

    # Step 5: Return the best results
    return best_sorted_polygons, best_sorted_col_removed_polygons, best_sorted_intersections