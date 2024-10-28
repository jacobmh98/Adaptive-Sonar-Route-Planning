import numpy as np
import Polygon  # Ensure this module is correctly defined with 3D centroid support

def calculate_total_distance(points, end_point):
    total_distance = 0.0

    for i in range(len(points) - 1):
        distance = np.linalg.norm(points[i + 1] - points[i])
        total_distance += distance
    total_distance += np.linalg.norm(points[-1] - end_point)

    return total_distance


def compute_normal_vector(hard_edge_start, hard_edge_end):
    edge_vector = hard_edge_end - hard_edge_start
    normal_vector = np.array([-edge_vector[1], edge_vector[0], 0.0], dtype=float)
    normal_vector /= np.linalg.norm(normal_vector)
    return normal_vector


def lines_intersect(p1, p2, p3, p4, epsilon=1e-6):
    def orientation(a, b, c):
        val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
        return 0 if abs(val) < epsilon else (1 if val > 0 else 2)

    def on_segment(p, q, r, epsilon=1e-6):
        return (min(p[0], r[0]) - epsilon <= q[0] <= max(p[0], r[0]) + epsilon and
                min(p[1], r[1]) - epsilon <= q[1] <= max(p[1], r[1]) + epsilon)

    o1, o2, o3, o4 = orientation(p1, p2, p3), orientation(p1, p2, p4), orientation(p3, p4, p1), orientation(p3, p4, p2)
    return (o1 != o2 and o3 != o4) or (o1 == 0 and on_segment(p1, p3, p2, epsilon) or
                                       o2 == 0 and on_segment(p1, p4, p2, epsilon) or
                                       o3 == 0 and on_segment(p3, p1, p4, epsilon) or
                                       o4 == 0 and on_segment(p3, p2, p4, epsilon))


def is_point_on_segment(point, seg_start, seg_end, epsilon=1e-6):
    vector_a = seg_end - seg_start
    vector_b = point - seg_start
    cross_product = np.abs(np.cross(vector_a, vector_b))

    if cross_product > epsilon:
        return False

    min_x, max_x = min(seg_start[0], seg_end[0]), max(seg_start[0], seg_end[0])
    min_y, max_y = min(seg_start[1], seg_end[1]), max(seg_start[1], seg_end[1])
    return (min_x - epsilon <= point[0] <= max_x + epsilon and
            min_y - epsilon <= point[1] <= max_y + epsilon)


def check_line_intersects_hard_edges(start_point, end_point, hard_edges, ignore_edge=None):
    closest_edge = None
    min_distance = float('inf')

    for hard_edge_start, hard_edge_end in hard_edges:
        # Skip the edge to ignore if specified
        if ignore_edge is not None and (
                np.array_equal(hard_edge_start, ignore_edge[0]) and np.array_equal(hard_edge_end, ignore_edge[1])):
            continue

        if lines_intersect(start_point, end_point, hard_edge_start, hard_edge_end):
            midpoint = (hard_edge_start + hard_edge_end) / 2
            distance = np.linalg.norm(start_point - midpoint)
            if distance < min_distance:
                min_distance = distance
                closest_edge = (hard_edge_start, hard_edge_end)

    return closest_edge


def check_line_intersects_excluding_start_point_edges(start_point, end_point, hard_edges):
    closest_edge = None
    min_distance = float('inf')

    for hard_edge_start, hard_edge_end in hard_edges:
        # Ignore this edge if it contains the start point
        if is_point_on_segment(start_point, hard_edge_start, hard_edge_end):
            continue

        # Check if the line between start_point and end_point intersects with the hard edge
        if lines_intersect(start_point, end_point, hard_edge_start, hard_edge_end):
            midpoint = (hard_edge_start + hard_edge_end) / 2
            distance = np.linalg.norm(start_point - midpoint)
            if distance < min_distance:
                min_distance = distance
                closest_edge = (hard_edge_start, hard_edge_end)

    return closest_edge


def is_point_on_hard_edge(point, hard_edges):
    for hard_edge_start, hard_edge_end in hard_edges:
        if is_point_on_segment(point, hard_edge_start, hard_edge_end):
            return True, (hard_edge_start, hard_edge_end)  # Return the hard edge as well
    return False, None

def calculate_cross_product_2d(v1, v2):
    """Calculate the cross product of two 2D vectors."""
    return v1[0] * v2[1] - v1[1] * v2[0]

def is_line_moving_into_polygon(start_point, end_point, hard_edge, polygon):
    hard_edge_start, hard_edge_end = hard_edge

    # Create 2D vectors from the points
    line_vector = (end_point[0] - start_point[0], end_point[1] - start_point[1])
    hard_edge_vector = (hard_edge_end[0] - hard_edge_start[0], hard_edge_end[1] - hard_edge_start[1])

    # Calculate centroid
    centroid = polygon.calculate_centroid()  # Ensure this returns a 2D tuple (x, y)

    # Create vector from hard edge start to centroid
    centroid_vector = (centroid[0] - hard_edge_start[0], centroid[1] - hard_edge_start[1])

    # Calculate the cross products
    cross_product_line = calculate_cross_product_2d(hard_edge_vector, line_vector)
    cross_product_centroid = calculate_cross_product_2d(hard_edge_vector, centroid_vector)

    # Determine the direction of movement
    return (cross_product_line > 0) and (cross_product_centroid > 0)


def find_adjacent_hard_edge(hard_edge, point, hard_edges):
    hard_edge_start, hard_edge_end = hard_edge

    for edge in hard_edges:
        # Check if the edge is not the same as the original hard edge
        if not (np.array_equal(edge[0], hard_edge_start) and np.array_equal(edge[1], hard_edge_end)):
            edge_start, edge_end = edge
            # Check if the given point matches either end of the adjacent edge
            if np.array_equal(point, edge_start) or np.array_equal(point, edge_end):
                return edge  # Return the adjacent hard edge

    return None  # Return None if no adjacent edge is found


def find_hard_edges_sharing_point(point, hard_edges):
    sharing_edges = []
    for hard_edge_start, hard_edge_end in hard_edges:
        if np.array_equal(point, hard_edge_start) or np.array_equal(point, hard_edge_end):
            sharing_edges.append((hard_edge_start, hard_edge_end))
    return sharing_edges


def find_optimal_path(left_path, right_path, end_point):
    # Choosing the shortest path (first by number of turns, then by distance)
    if len(left_path) == len(right_path):
        left_dist = calculate_total_distance(left_path, end_point)
        right_dist = calculate_total_distance(right_path, end_point)
        return left_path if left_dist < right_dist else right_path

    elif len(left_path) < len(right_path):
        return left_path
    else:
        return right_path

def follow_and_create_path(temp_path, end_point, current_hard_edge, hard_edges_list, direction):
    while True:
        # Checking for a free path
        intersecting_edge = check_line_intersects_excluding_start_point_edges(temp_path[-1], end_point,
                                                                              hard_edges_list)
        # Check if a free path is found then stop searching
        if not intersecting_edge:
            break

        # Finding the next hard edge to follow around the obstacle, using prev point to get edge, so always in correct direction
        current_hard_edge = find_adjacent_hard_edge(current_hard_edge, temp_path[-1], hard_edges_list)

        # Which next point to append depends on direction
        if direction == 'right':
            temp_path.append(current_hard_edge[0])
        else:
            temp_path.append(current_hard_edge[1])

    return temp_path


def avoid_hard_edges(start_point, end_point, current_polygon, polygons, hard_edges_list):
    intermediate_points = []
    current_poly_index = polygons.index(current_polygon)
    prev_poly_index = current_poly_index - 1

    # Check if the start point is on a hard edge
    start_on_hard_edge, start_hard_edge = is_point_on_hard_edge(start_point, hard_edges_list)
    print(f"Start hard edge: {start_hard_edge}")

    # Test case where start point is on a hard edge
    if start_on_hard_edge:
        print("On hard edge")

        # Check if the line from the start point to the end point is moving into polygon and not the obstacle
        if is_line_moving_into_polygon(start_point, end_point, start_hard_edge, polygons[prev_poly_index]):
            print("Going into clear area")
            # Check for intersections with hard edges, ignoring the edge where the start point lies
            intersecting_edge = check_line_intersects_hard_edges(start_point, end_point, hard_edges_list,
                                                                 ignore_edge=start_hard_edge)
            if intersecting_edge:
                right_temp_path = follow_and_create_path([start_point, intersecting_edge[0]], end_point, start_hard_edge, hard_edges_list, direction='right')
                left_temp_path = follow_and_create_path([start_point, intersecting_edge[1]], end_point, start_hard_edge, hard_edges_list,direction='left')
                intermediate_points = find_optimal_path(left_temp_path, right_temp_path, end_point)

        # Line from start to end is moving into obstacle
        else:
            print("Going into unclear area")
            intersecting_edge = check_line_intersects_hard_edges(start_point, end_point, hard_edges_list)

            if intersecting_edge:
                right_temp_path = follow_and_create_path([start_point, intersecting_edge[0]], end_point, start_hard_edge, hard_edges_list, direction='right')
                left_temp_path = follow_and_create_path([start_point, intersecting_edge[1]], end_point, start_hard_edge, hard_edges_list,direction='left')
                intermediate_points = find_optimal_path(left_temp_path, right_temp_path, end_point)

    else:
        print("Point not on hard edge")
        start_hard_edge = check_line_intersects_hard_edges(start_point, end_point, hard_edges_list)

        # No intersections found, so it is a clear path
        if start_hard_edge:
            right_temp_path = follow_and_create_path([start_point, start_hard_edge[0]], end_point, start_hard_edge,
                                                     hard_edges_list, direction='right')
            left_temp_path = follow_and_create_path([start_point, start_hard_edge[1]], end_point, start_hard_edge,
                                                    hard_edges_list, direction='left')
            intermediate_points = find_optimal_path(left_temp_path, right_temp_path, end_point)

    return intermediate_points
