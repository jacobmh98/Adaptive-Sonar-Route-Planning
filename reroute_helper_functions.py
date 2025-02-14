import numpy as np

# Helper functions used for both obstacles and region hard edges
def is_start_end_on_edge(start_point, end_point, v1, v2, epsilon = 1e-9):
    edge_vector = np.array(v2) - np.array(v1)
    start_vector = np.array(start_point) - np.array(v1)
    end_vector = np.array(end_point) - np.array(v1)

    # Check collinearity and within bounds
    start_on_edge = (
            abs(np.cross(edge_vector, start_vector)) < epsilon and
            0 <= np.dot(start_vector, edge_vector) <= np.dot(edge_vector, edge_vector)
    )
    end_on_edge = (
            abs(np.cross(edge_vector, end_vector)) < epsilon and
            0 <= np.dot(end_vector, edge_vector) <= np.dot(edge_vector, edge_vector)
    )
    return start_on_edge, end_on_edge

def calculate_winding_number(point, vertices, epsilon = 1e-9):
    """Calculate the winding number for a point with respect to a polygon."""
    x, y = point
    winding_number = 0

    for i in range(len(vertices)):
        v1 = vertices[i]
        v2 = vertices[(i + 1) % len(vertices)]  # Wrap around to the first vertex
        x1, y1 = v1.x, v1.y
        x2, y2 = v2.x, v2.y

        # Calculate vectors
        vec1 = np.array([x1 - x, y1 - y])
        vec2 = np.array([x2 - x, y2 - y])

        # Calculate cross product and angle
        cross = np.cross(vec1, vec2)
        dot = np.dot(vec1, vec2)
        angle = np.arctan2(cross, dot)

        # Accumulate winding number
        winding_number += angle

    return abs(winding_number) > epsilon

def is_point_on_edges(point, edges, epsilon=1e-9):
    """
    Checks if a given point lies on any edge in a list of edges.

    :param point: The point to check as a tuple (x, y).
    :param edges: List of edges, where each edge is represented as ((x1, y1), (x2, y2)).
    :param epsilon: Float tolerance for precision issues.
    :return: True if the point lies on any edge, False otherwise.
    """
    px, py = point

    for edge in edges:
        (x1, y1), (x2, y2) = edge

        # Check if the point is collinear with the edge
        edge_vector = (x2 - x1, y2 - y1)
        point_vector = (px - x1, py - y1)
        cross_product = abs(edge_vector[0] * point_vector[1] - edge_vector[1] * point_vector[0])

        if cross_product > epsilon:
            continue  # Point is not collinear

        # Check if the point lies within the bounds of the edge
        dot_product = (point_vector[0] * edge_vector[0] + point_vector[1] * edge_vector[1])
        edge_length_squared = edge_vector[0]**2 + edge_vector[1]**2

        if 0 <= dot_product <= edge_length_squared:
            return True  # Point lies on this edge

    return False

def compute_path_distance(path):
    """Computes the total distance traveled along a path.

    :param path: List of points representing the path, where each point is a tuple (x, y) or a numpy array.
    :return: Float representing the total distance traveled along the path.
    """

    if len(path) < 2:
        return 0.0  # No distance if the path has fewer than 2 points

    total_distance = 0.0

    for i in range(len(path) - 1):
        point_a = np.array(path[i])
        point_b = np.array(path[i + 1])
        total_distance += np.linalg.norm(point_b - point_a)

    return total_distance

def find_best_path(left_temp_path, right_temp_path):
    """Determines the best path direction between left and right paths, considering both turns and distance.

    :param left_temp_path: List of points representing the left path.
    :param right_temp_path: List of points representing the right path.
    :return: The best path (list of points) based on a mix of fewer turns and shorter distance.
    """
    # Weight for each turn in terms of equivalent distance
    turn_weight = 10  # low weigth, as it is transit lines, and sonar is turned off during transit so not as punishing to turn

    def compute_path_cost(path):
        # Number of turns is len(path) - 1
        turns = len(path) - 1
        distance = compute_path_distance(path)
        return distance + (turns * turn_weight)

    left_cost = compute_path_cost(left_temp_path)
    right_cost = compute_path_cost(right_temp_path)


    if left_cost <= right_cost:
        return left_temp_path
    else:
        return right_temp_path


def line_intersection(p1, p2, q1, q2, epsilon=1e-9):
    """Checks if line segment (p1, p2) intersects (q1, q2).

    :param p1: Tuple representing the start of the first line segment.
    :param p2: Tuple representing the end of the first line segment.
    :param q1: Tuple representing the start of the second line segment.
    :param q2: Tuple representing the end of the second line segment.
    :param epsilon: Float tolerance for floating-point comparisons.
    :return: Boolean indicating whether the segments intersect.
    """

    r = np.array(p2) - np.array(p1)
    s = np.array(q2) - np.array(q1)
    r_cross_s = np.cross(r, s)

    if abs(r_cross_s) < epsilon:
        return False  # The lines are parallel or collinear

    q1_p1 = np.array(q1) - np.array(p1)
    t = np.cross(q1_p1, s) / r_cross_s
    u = np.cross(q1_p1, r) / r_cross_s

    return 0 <= t <= 1 and 0 <= u <= 1