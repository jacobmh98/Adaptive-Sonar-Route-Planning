import numpy as np
import matplotlib.pyplot as plt
import math
from Polygon import Polygon, Vertex
from matplotlib.patches import Patch
from coverage_plots import *

from functions import plot_results3
from global_variables import *


def create_vector(v1, v2):
    """ Computing a vector from vertex v1 to vertex v2
    :param v1: NumPy array, start point
    :param v2: NumPy array, end point
    :return vector:
    """
    return v2 - v1

def compute_offset_vector(vector, sweep_direction, d):
    """ Compute a vector parallel to the vector from v1 to v2, but offset perpendicularly
    in the direction specified by sweep_direction.

    :
    :param sweep_direction: int, the direction to offset the vector (+1 or -1)
    :param d: float, the distance by which the vector should be offset
    :return new_v1: NumPy array, the new starting point of the offset vector
    :return new_v2: NumPy array, the new ending point of the offset vector
    """
    v1 = vector[0]
    v2 = vector[1]
    # Find the perpendicular direction to the vector from v1 to v2
    vector = create_vector(v1,v2)
    perp_vector = np.array([-vector[1], vector[0]])

    # Normalize the perpendicular vector
    perp_vector_normalized = perp_vector / np.linalg.norm(perp_vector)

    # Offset the original vector by d in the direction of the perpendicular vector
    offset = perp_vector_normalized * d * sweep_direction

    # New starting and ending points for the offset vector
    new_v1 = v1 + offset
    new_v2 = v2 + offset

    return new_v1, new_v2

def compute_sweep_direction(v1, v2, a):
    """
    :param v1: NumPy array, the start point of the vector (b)
    :param v2: NumPy array, the end point of the vector (b_mate)
    :param a: NumPy array with 2D point coordinates
    :return: int, -1 or +1 to indicate sweep direction from original vector
    """
    # Calculate the original vector from b to b_mate
    vector_v1_v2 = create_vector(v1, v2)

    # Calculate the vector from b to a
    vector_v1_a = create_vector(v1, a)

    # Find the perpendicular direction from v1-v2 towards a
    perp_vector = np.array([-vector_v1_v2[1], vector_v1_v2[0]])

    # Normalize the perpendicular vector and calculate direction towards a
    perp_vector_normalized = perp_vector / np.linalg.norm(perp_vector)
    sweep_direction = np.sign(
        np.dot(perp_vector_normalized.flatten(), vector_v1_a.flatten()))  # -1 or +1 depending on which side a is

    return sweep_direction

def compute_boundary(poly):
    coords = poly.vertices_matrix()
    boundary = (np.min(coords[0, :]), np.max(coords[0, :]),
                     np.min(coords[1, :]), np.max(coords[1, :]))
    return boundary

def extend_vector_to_boundary(vector, boundary):
    """ Extend the vector from v1 to v2 to intersect with the boundary box

    :param vector: Numpy array of the vector
    :param boundary: List of polygon's boundaries
    :return extended_v1: NumPy array, the new start point of the extended vector
    :return extended_v2: NumPy array, the new end point of the extended vector
    """
    # Extract the boundary box values
    min_x, max_x, min_y, max_y = boundary

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
        t_min_y = -np.inf  # No need to stretch vertically
        t_max_y = np.inf

    # Choose the correct scaling factor (the smallest positive value to reach the boundary)
    t_min = max(min(t_min_x, t_max_x), min(t_min_y, t_max_y))
    t_max = min(max(t_min_x, t_max_x), max(t_min_y, t_max_y))

    # If t_min or t_max are infinite, it means there is no intersection
    if np.isinf(t_min):
        extended_v1 = v1  # No extension needed
    else:
        extended_v1 = v1 + direction * t_min

    if np.isinf(t_max):
        extended_v2 = v2  # No extension needed
    else:
        extended_v2 = v1 + direction * t_max

    return np.array([extended_v1, extended_v2])

def distance_between_points(v1, v2):
    """ Calculate the Euclidean distance between two 2D points.

    :param v1: NumPy array (2D point)
    :param v2: NumPy array (2D point)
    :return distance: float, the distance between the two points.
    """
    return math.sqrt((v2[0] - v1[0]) ** 2 + (v2[1] - v1[1]) ** 2)


def point_to_line_distance(p, v1, v2):
    """Compute the perpendicular distance from point p to the line segment defined by v1 and v2."""
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
    """Compute the smallest diameter of the polygon."""
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

def get_path_intersections(poly, path_width, b_index, b_mate_index, a_index, boundary):
    """ Algorithm 1 - GetPath algorithm from page 5 in Coverage Path Planning for 2D Convex Regions,
        Adjusted for multi polygons. Just returns the list of intersection pairs

    :param poly: Polygon P, using Polygon class
    :param b_index: Starting vertex index
    :param b_mate_index: b's counterclockwise neighbour index
    :param a_index: b's diametral antipodal point index
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
    smallest_diameter = get_smallest_diameter(poly)

    # If path width is larger than the smallest diameter, then change path to half the diameter size
    if path_width > smallest_diameter:
        # In case of path width being too large to fit inside polygon
        delta_init = smallest_diameter / 2
    else:
        delta_init = path_width / 2

    # Creating a vector from vertex b to b_mate
    v_initial = np.array([b, b_mate])
    # Offsetting vector b to b_mate with delta_init towards point a
    v_offset = compute_offset_vector(v_initial, sweep_direction, delta_init)
    # Extending the offset vector to polygon boundaries to find all intersection points with poly edge (2 points)
    v_extended = extend_vector_to_boundary(v_offset, boundary)

    #L_flight2 = compute_offset_vector(L_flight1[0], L_flight1[1], sweep_direction, dx)
    #L_flight2 = np.array(extend_vector_to_boundary(L_flight2[0], L_flight2[1], boundary))
    #plot_vectors_simple(poly, b, b_mate, a, L_flight, L_flight1, L_flight2, boundary, dx)

    #def plot_vectors_simple(poly, b, b_mate, a, L_flight_ext, boundary, show_legend=True):

    # Fail-safe parameters for the while loop
    max_iterations = 10000
    counter = 0

    all_intersections = []

    # Loop until no intersections are found
    while True:

        # Find the new intersections
        new_intersections = Polygon.find_intersections(poly, v_extended)

        # Check if no new intersections are found
        if not new_intersections:
            break
        else:
            # Assume new_intersections is a list of points in the form [[x1, y1], [x2, y2]]
            for i in range(0, len(new_intersections), 2):
                # Create tuples of two consecutive points and append to all_intersections
                all_intersections.append((new_intersections[i], new_intersections[i + 1]))

        # Ensure coverage gets as close to polygon edge as needed for full coverage
        """  TODO: Need to create function to check full coverage, else best coverage path wont get chosen as it is longer than path without full coverage
        if (distance_between_points(a, ip1) <= dx) or (distance_between_points(a, ip2) <= dx):
            if go_to_edge:  # Only divide once per poly, to avoid never ending loop
                dx = dx/2
                go_to_edge = False
        """

        # Computing next extended offset vector, offset with full path width dx
        v_offset = compute_offset_vector(v_extended, sweep_direction, path_width)
        v_extended = extend_vector_to_boundary(v_offset, boundary)

        # Avoid infinite while loop
        if counter >= max_iterations:
            print(f"Max iterations of {max_iterations} reached.")
            break
        counter += 1

    return all_intersections

def compute_angle(i, j):
    """
    Compute the angle between two points in radians.

    :param i: First point as a NumPy array or list [x1, y1]
    :param j: Second point as a NumPy array or list [x2, y2]
    :return: Angle in radians between the two points.
    """
    delta_x = j[0] - i[0]
    delta_y = j[1] - i[1]
    return np.arctan2(delta_y, delta_x)


def best_intersection(poly, path_width, i, j):
    """
    :param poly: Polygon
    :param i: Index of first point in antipodal pair
    :param j: Index of second point in antipodal pair
    :return: A list of intersection pairs
    """
    boundary = compute_boundary(poly)

    # Triangle edge case
    if len(poly.vertices) == 3:
        b_index = i
        b_mate_index = j

        if poly.get_mate(b_index) == b_mate_index:
            a_index = poly.get_mate(b_mate_index)
        else:
            a_index = poly.get_mate(b_index)

        intersections = get_path_intersections(poly, path_width, b_index, b_mate_index, a_index, boundary)

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
            b2_index = b_index - 1
            a2_index = a_index
        else:
            b2_index = a_index - 1
            a2_index = b_index

        # Finding path that holds the least flight lines (smallest dist between point b and a creates longer and fewer lines)
        if distance_between_points(poly.vertices[b_index].v, poly.vertices[a_index].v) < distance_between_points(poly.vertices[b2_index].v, poly.vertices[a2_index].v):
            intersections = get_path_intersections(poly, path_width, b_index, poly.get_mate(b_index), a_index, boundary)
        else:
            intersections = get_path_intersections(poly, path_width, b2_index, poly.get_mate(b2_index), a2_index, boundary)

    return intersections