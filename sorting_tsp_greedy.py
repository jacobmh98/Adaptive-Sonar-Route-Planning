import cpp_connect_path
import math


def get_pairs(intersections):
    """ Generate start-end pairs for a path based on intersections.

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


def euclidean_distance(p1, p2):
    """Compute Euclidean distance between two points."""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def line_intersection(p1, p2, q1, q2, epsilon=1e-9):
    """ Find intersection between two line segments (p1, p2) and (q1, q2). """
    r = [p2[0] - p1[0], p2[1] - p1[1]]
    s = [q2[0] - q1[0], q2[1] - q1[1]]

    r_cross_s = r[0] * s[1] - r[1] * s[0]
    if abs(r_cross_s) < epsilon:  # Lines are parallel
        return None

    p1_q1 = [q1[0] - p1[0], q1[1] - p1[1]]
    t = (p1_q1[0] * s[1] - p1_q1[1] * s[0]) / r_cross_s
    u = (p1_q1[0] * r[1] - p1_q1[1] * r[0]) / r_cross_s

    # Check if intersection is within both line segments
    if (0 <= t <= 1) and (0 <= u <= 1):
        intersection_point = [p1[0] + t * r[0], p1[1] + t * r[1]]
        return intersection_point
    return None


def count_intersections_with_region(current_end_point, current_start_point, region):
    """ Count the number of intersections a line from the current endpoint to the start point has with the region polygon.

    :param current_end_point: The current endpoint of the path.
    :param current_start_point: The start point of the path.
    :param region: The region polygon to test intersections with.
    :return: The number of intersections.
    """
    intersections_count = 0

    # Iterate through the edges of the region polygon
    for i in range(len(region.vertices)):
        v1 = region.vertices[i]
        v2 = region.vertices[(i + 1) % len(region.vertices)]

        # Check if the line from the current end point to the current start point intersects with this edge
        intersection = line_intersection(current_end_point, current_start_point, (v1.x, v1.y), (v2.x, v2.y))

        if intersection is not None:
            intersections_count += 1

    return intersections_count


def compute_distances(subpolygons):
    """ Compute distances between end points of one subpolygon
        and start points of the next subpolygon.

    param subpolygons: List of subpolygons, each with 4 start/end pairs.
    returns distances: Nested dictionary of distances between pairs.
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


def solve_greedy_tsp_sorting(polygons, all_intersections, region, intersection_weight = 100, long_path_penalty = 250, long_path_distance = 500):
    """ Solve the TSP by testing all start pairs and iteratively finding the closest path.
    :param polygons: List of polygons.
    :param all_intersections: List of intersections for each polygon.
    :param region: Polygon representing the region to check intersections with.

    :return: Best sorted polygons, intersections, and total distance.
    """

    # Generate start/end pairs for each polygon
    all_polygon_start_end_pairs = [get_pairs(intersections) for intersections in all_intersections]

    # Initialize variables to track the best path
    best_path = None
    best_total_score = float('inf')

    # Test all 4 starting pairs for the first polygon
    for start_pair_index in range(4):  # 4 pairs in the first polygon
        path = [(0, start_pair_index)]  # Start with the first polygon and this pair
        total_distance = 0
        total_score = 0
        current_end_point = all_polygon_start_end_pairs[0][start_pair_index][1]  # End point of the current pair

        visited = set()  # Keep track of visited polygons
        visited.add(0)  # Mark the first polygon as visited

        # Iteratively find the closest pair for all other polygons
        while len(visited) < len(all_polygon_start_end_pairs):
            # Find the next closest polygon and pair
            next_polygon_index = None
            next_pair_index = None
            min_score = float('inf')

            for n in range(len(all_polygon_start_end_pairs)):
                if n in visited:
                    continue  # Skip already visited polygons

                for j in range(4):  # Test all 4 pairs in the current polygon
                    start_point = all_polygon_start_end_pairs[n][j][0]  # Start point of the pair
                    distance = euclidean_distance(current_end_point, start_point)
                    intersections = count_intersections_with_region(current_end_point, start_point, region)
                    if distance > long_path_distance:
                        current_score = distance + (intersection_weight * intersections) + long_path_penalty
                    else:
                        current_score = distance + (intersection_weight * intersections)

                    # Prioritize the path with the shortest distance, and in case of ties, minimize the number of intersections
                    if current_score < min_score:
                        min_distance = distance
                        min_score = current_score
                        next_polygon_index = n
                        next_pair_index = j

            # Update the path and total distance
            path.append((next_polygon_index, next_pair_index))
            total_distance += min_distance
            total_score += min_score
            current_end_point = all_polygon_start_end_pairs[next_polygon_index][next_pair_index][1]  # Update end point
            visited.add(next_polygon_index)  # Mark as visited

        # Check if this path is the best
        #if total_distance < best_total_distance:
        if total_score < best_total_score:
            best_total_score = total_score
            best_path = path

    # Sort polygons and intersections based on the best path
    best_sorted_polygons = [polygons[p[0]] for p in best_path]
    best_sorted_col_removed_polygons = [polygons[p[0]] for p in best_path]
    best_sorted_intersections = [all_intersections[p[0]] for p in best_path]

    return best_sorted_polygons, best_sorted_col_removed_polygons, best_sorted_intersections
