import math
from plot_cpp import *
from global_variables import *


def create_vector(v1, v2):
    """Compute a vector from vertex v1 to vertex v2.

    :param v1: NumPy array, start point
    :param v2: NumPy array, end point
    :return: NumPy array representing the vector from v1 to v2
    """
    return v2 - v1


def compute_offset_vector(vector, sweep_direction, d):
    """Compute a vector parallel to the given vector, offset perpendicularly
    in the direction specified by sweep_direction.

    :param vector: NumPy array, the original vector
    :param sweep_direction: int, the direction to offset the vector (+1 or -1)
    :param d: float, the distance by which the vector should be offset
    :return new_v1: NumPy array, the new starting point of the offset vector
    :return new_v2: NumPy array, the new ending point of the offset vector
    """
    v1 = vector[0]
    v2 = vector[1]
    vector = create_vector(v1, v2)
    perp_vector = np.array([-vector[1], vector[0]])
    perp_vector_normalized = perp_vector / np.linalg.norm(perp_vector)
    offset = perp_vector_normalized * d * sweep_direction
    new_v1 = v1 + offset
    new_v2 = v2 + offset

    return new_v1, new_v2


def compute_sweep_direction(b, b_mate, a):
    """Determine the sweep direction (+1 or -1) based on the relative position of point a
    to the vector defined by v1 and v2.

    :param b: NumPy array, the start point of the vector (b)
    :param b_mate: NumPy array, the end point of the vector (b_mate)
    :param a: NumPy array with 2D point coordinates
    :return: int, -1 or +1 to indicate sweep direction from the original vector
    """
    vector_b_bmate = create_vector(b, b_mate)
    vector_b_a = create_vector(b, a)
    perp_vector = np.array([-vector_b_bmate[1], vector_b_bmate[0]])
    perp_vector_normalized = perp_vector / np.linalg.norm(perp_vector)
    sweep_direction = np.sign(np.dot(perp_vector_normalized.flatten(), vector_b_a.flatten()))

    return sweep_direction


def extend_vector_to_boundary(vector, boundary):
    """ Extend the vector from v1 to v2 to intersect with the boundary box

    :param vector: Numpy array of the vector
    :param boundary: List of polygon's boundaries
    :return extended_v1: NumPy array, the new start point of the extended vector
    :return extended_v2: NumPy array, the new end point of the extended vector
    """
    # Extract the boundary box values
    min_x, max_x, min_y, max_y = boundary

    # Extending the vector slightly to handle edge cases
    boundary_buffer = 1e-6
    min_x -= boundary_buffer
    max_x += boundary_buffer
    min_y -= boundary_buffer
    max_y += boundary_buffer

    # Extract start and end point of input vector
    v1 = vector[0]
    v2 = vector[1]

    # Calculate the direction vector from v1 to v2
    direction = create_vector(v1, v2)

    # Calculate the scaling factors needed to reach the boundary box
    if direction[0] != 0:  # Check for non-vertical lines
        t_min_x = (min_x - v1[0]) / direction[0]
        t_max_x = (max_x - v1[0]) / direction[0]
    else:
        # Vertical line case
        t_min_x = -np.inf  # No need to stretch horizontally
        t_max_x = np.inf

    if direction[1] != 0:  # Check for non-horizontal lines
        t_min_y = (min_y - v1[1]) / direction[1]
        t_max_y = (max_y - v1[1]) / direction[1]
    else:
        # Horizontal line case
        t_min_y = -np.inf
        t_max_y = np.inf

    # Choose the correct scaling factor (the smallest positive value to reach the boundary)
    t_min = max(min(t_min_x, t_max_x), min(t_min_y, t_max_y))
    t_max = min(max(t_min_x, t_max_x), max(t_min_y, t_max_y))

    # If t_min or t_max are infinite, it means there is no intersection
    if np.isinf(t_min):
        extended_v1 = v1
    else:
        extended_v1 = v1 + direction * t_min

    if np.isinf(t_max):
        extended_v2 = v2
    else:
        extended_v2 = v1 + direction * t_max

    return np.array([extended_v1, extended_v2])


def point_to_line_distance(p, v1, v2):
    """Compute the perpendicular distance from point p to the line segment defined by v1 and v2.

    :param p: Point (Vertex) to measure the distance from
    :param v1: Starting point of the line segment
    :param v2: Ending point of the line segment
    :return: Perpendicular distance as a float
    """

    x0, y0 = p.x, p.y
    x1, y1 = v1.x, v1.y
    x2, y2 = v2.x, v2.y

    # Line equation parameters
    num = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    denom = np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)

    # Check for zero division
    if denom < intersection_epsilon:
        return 0  # If the two points are the same, return 0 distance

    return num / denom


def get_smallest_diameter(polygon):
    """Compute the smallest diameter of the polygon by checking distances
    between vertices and vertex-to-edge distances.

    :param polygon: Polygon object containing vertices
    :return: Smallest diameter as a float
    """

    n = len(polygon.vertices)

    if n == 3:
        # For a triangle, check both vertex-to-vertex and vertex-to-edge distances
        v1, v2, v3 = polygon.vertices[0], polygon.vertices[1], polygon.vertices[2]

        # Compute vertex-to-vertex distances
        dist_v1_v2 = np.linalg.norm(v1.get_array() - v2.get_array())
        dist_v2_v3 = np.linalg.norm(v2.get_array() - v3.get_array())
        dist_v3_v1 = np.linalg.norm(v3.get_array() - v1.get_array())

        # Compute vertex-to-edge distances
        dist_v1_to_edge = point_to_line_distance(v1, v2, v3)
        dist_v2_to_edge = point_to_line_distance(v2, v1, v3)
        dist_v3_to_edge = point_to_line_distance(v3, v1, v2)

        # Return the minimum of all the computed distances
        return min(dist_v1_v2, dist_v2_v3, dist_v3_v1, dist_v1_to_edge, dist_v2_to_edge, dist_v3_to_edge)

    else:
        # For polygons with more than 3 vertices, find the smallest distance between any two vertices
        min_distance = float('inf')

        for i in range(n):
            for j in range(i + 1, n):
                dist = np.linalg.norm(polygon.vertices[i].get_array() - polygon.vertices[j].get_array())
                if dist < min_distance:
                    min_distance = dist

        return min_distance


# Function to check if two lines are parallel in 2D
def are_lines_parallel_2d(p1, p2, q1, q2, epsilon=1e-9):
    determinant = (p2[0] - p1[0]) * (q2[1] - q1[1]) - (p2[1] - p1[1]) * (q2[0] - q1[0])
    return abs(determinant) < epsilon


# Function to calculate the distance between two points
def distance_between_points(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))


# Function to check for parallel lines and return the closest one to v_initial
def find_closest_parallel_intersection(v_initial, intersections, epsilon=1e-9):
    # Extract points from v_initial
    p1, p2 = v_initial[0], v_initial[1]
    min_distance = float('inf')
    closest_intersection = None

    # Loop through each set of intersections
    for polygon_intersections in intersections:
        for intersection_pair in polygon_intersections:
            # Each intersection_pair contains two points, check if they are parallel to v_initial
            q1, q2 = intersection_pair

            # Check if the lines are parallel
            if are_lines_parallel_2d(p1, p2, q1, q2, epsilon):
                # Compute the distance between the midpoints of the two lines to determine proximity
                midpoint_q = (q1 + q2) / 2
                midpoint_p = (p1 + p2) / 2
                distance = distance_between_points(midpoint_p, midpoint_q)

                # If this distance is smaller than the current minimum, update the closest intersection
                if distance < min_distance:
                    min_distance = distance
                    closest_intersection = intersection_pair

    return closest_intersection


def get_path_intersections(poly, current_path_width, current_overlap_distance, b_index, b_mate_index, a_index, boundary):
    """ Algorithm 1 - GetPath algorithm from page 5 in Coverage Path Planning for 2D Convex Regions,
        Adjusted for multi polygons. Just returns the list of intersection pairs

    :param poly: Polygon, using Polygon class
    :param current_path_width: Float, the chosen path width
    :param current_overlap_distance: Float, the chosen overlap distance
    :param b_index: Starting vertex index
    :param b_mate_index: b's counterclockwise neighbour index
    :param a_index: b's antipodal point index
    :param boundary: List of polygon's boundaries
    :return intersections: A multidim list of intersection points, as pairs
    """
    # Getting the three vertices as points from the polygon
    b = poly.vertices[b_index].v.flatten()
    b_mate = poly.vertices[b_mate_index].v.flatten()
    a = poly.vertices[a_index].v.flatten()

    # Finding direction from vector b, b_mate towards a (+1 or -1)
    sweep_direction = compute_sweep_direction(b, b_mate, a)

    # First pass is offset from polygon edge with half path width, unless path width is too large for the polygon
    diameter = point_to_line_distance(poly.vertices[a_index], poly.vertices[b_index], poly.vertices[b_mate_index])

    # First pass is offset from polygon edge with half path width, unless path width is too large for the polygon
    if current_path_width > diameter:
        # If path width is larger than the diameter, then change path to half the diameter size
        delta_init = diameter / 2
    else:
        delta_init = current_path_width / 2

    # Creating a vector from vertex b to b_mate
    v_initial = np.array([b, b_mate])

    # Checking if overlap distance does not make delta_init negative, if so, overlap dist is reduced to be just smaller than d_x
    if delta_init - current_overlap_distance <= 0:
        init_overlap_distance = delta_init - 0.01
    else:
        init_overlap_distance = current_overlap_distance

    # Optimizing by doing the next check just once instead of in the while loop
    if (2 * delta_init) - current_overlap_distance <= 0:
        remaining_overlap_distance = 2 * delta_init - 0.01
    else:
        remaining_overlap_distance = current_overlap_distance

    # Offsetting vector b to b_mate with delta_init towards point a
    v_offset = compute_offset_vector(v_initial, sweep_direction, delta_init - init_overlap_distance)

    # Extending the offset vector to polygon boundaries to find all intersection points with poly edge (2 points)
    v_extended = extend_vector_to_boundary(v_offset, boundary)
    prev_v_extended = v_extended

    # Fail-safe parameters for the while loop
    max_iterations = 10000
    counter = 0

    # Initializing intersections list
    all_intersections = []

    # Loop until no intersections are found
    while True:
        # Find the new intersections
        new_intersections = poly.find_intersections(v_extended)

        # Check if no new intersections are found
        if not new_intersections:

            # Create new vector with half current path width distance (which is delta init plus a small distance to avoid unnecessary overlap), to ensure complete coverage near the far edge
            # Offset in opposite direction to get back into poly area
            v_half_offset = compute_offset_vector(prev_v_extended, sweep_direction, delta_init - init_overlap_distance + 0.1)
            v_half_extended = extend_vector_to_boundary(v_half_offset, boundary)
            check_near_edge_intersections = poly.find_intersections(v_half_extended)

            if check_near_edge_intersections:
                # Create tuples of two consecutive points and append to all_intersections
                all_intersections.append((check_near_edge_intersections[0], check_near_edge_intersections[1]))
            break  # Only do it once

        else:
            # Create tuples of two consecutive points and append to all_intersections
            all_intersections.append((new_intersections[0], new_intersections[1]))
            prev_v_extended = v_extended

        # Computing next extended offset vector, offset with the full path width (2x delta init)
        v_offset = compute_offset_vector(v_extended, sweep_direction, 2 * delta_init - remaining_overlap_distance)
        v_extended = extend_vector_to_boundary(v_offset, boundary)

        # Avoid infinite while loop
        if counter >= max_iterations:
            print(f"Max iterations of {max_iterations} reached.")
            break
        counter += 1

    return all_intersections


def compute_angle(i, j):
    """ Compute the angle between two points in radians.

    :param i: First point as a NumPy array or list [x1, y1]
    :param j: Second point as a NumPy array or list [x2, y2]
    :return: Angle in radians between the two points.
    """
    delta_x = j[0] - i[0]
    delta_y = j[1] - i[1]
    return np.arctan2(delta_y, delta_x)


# Unused now, but kept for report and comparisons
def best_intersection(poly, current_path_width, current_overlap_distance, i, j, boundary):
    """ Using Rotating Calipers Path Planning to compute the optimal intersections for the given polygon

    :param poly: Polygon
    :param current_path_width: Chosen path width as a float
    :param current_overlap_distance: Chosen overlap distance as a float
    :param i: Index of first point in antipodal pair
    :param j: Index of second point in antipodal pair
    :param boundary: List of polygon's boundaries
    :return intersections: A list of intersection pairs for the given polygon
    """
    vertex_amount = len(poly.vertices)

    # Triangle edge case
    if vertex_amount == 3:
        b_index = i
        b_mate_index = j

        if poly.get_mate(b_index) == b_mate_index:
            a_index = poly.get_mate(b_mate_index)
        else:
            a_index = poly.get_mate(b_index)

        intersections = get_path_intersections(poly, current_path_width, current_overlap_distance, b_index, b_mate_index, a_index, boundary)

    else:
        # Rotating the caliper in clockwise direction
        if (compute_angle(poly.vertices[i].v, poly.vertices[j].v) - math.pi) < 0:
            b_index = j
            a_index = i
        else:
            b_index = i
            a_index = j

        # Rotating the caliper in counterclockwise direction
        phi = compute_angle(poly.vertices[b_index].v, poly.vertices[a_index].v) - math.pi
        gamma_b = compute_angle(poly.vertices[b_index-1].v, poly.vertices[b_index].v)
        gamma_a = compute_angle(poly.vertices[a_index-1].v, poly.vertices[a_index].v) - phi

        if gamma_b < gamma_a:
            if b_index == 0:
                b2_index = vertex_amount-1
            else:
                b2_index = b_index - 1
            a2_index = a_index
        else:
            if a_index == 0:
                b2_index = vertex_amount-1
            else:
                b2_index = a_index - 1
            a2_index = b_index

        # Checking if new points are neighbours, if so these are not needed
        if are_neighbors(len(poly.vertices), b2_index, a2_index):
            intersections = get_path_intersections(poly, current_path_width, current_overlap_distance, b_index, poly.get_mate(b_index), a_index, boundary)
            return intersections

        # In cases of either b2 or a2 is negative, the standard is returned (Should never hit)
        elif b2_index < 0 or a2_index < 0:
            intersections = get_path_intersections(poly, current_path_width, current_overlap_distance, b_index, poly.get_mate(b_index), a_index, boundary)
            return intersections

        # Finding path that holds the least flight lines (smallest dist between point b and a creates longer and fewer lines)
        if distance_between_points(poly.vertices[b_index].v, poly.vertices[a_index].v) < distance_between_points(poly.vertices[b2_index].v, poly.vertices[a2_index].v):
            intersections = get_path_intersections(poly, current_path_width, current_overlap_distance, b_index, poly.get_mate(b_index), a_index, boundary)
        else:
            intersections = get_path_intersections(poly, current_path_width, current_overlap_distance, b2_index, poly.get_mate(b2_index), a2_index, boundary)

    return intersections


def are_neighbors(polygon_length, index1, index2):
    """ Check if two points are neighbors (adjacent) on a polygon.

    :param polygon_length: Total number of vertices in the polygon.
    :param index1: Index of the first vertex.
    :param index2: Index of the second vertex.
    :return: True if the two points are neighbors, False otherwise.
    """
    # Calculate the forward and backward neighbors
    forward_neighbor = (index1 + 1) % polygon_length  # Next vertex, wrapping around

    backward_neighbor = (index1 - 1 + polygon_length) % polygon_length  # Previous vertex, wrapping around

    # Check if the second index matches either neighbor
    return index2 == forward_neighbor or index2 == backward_neighbor