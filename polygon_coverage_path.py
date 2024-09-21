from venv import create

import numpy as np
from Polygon import Polygon, Vertex
import matplotlib.pyplot as plt

def create_vector(v1, v2):
    """ Computing a vector from vertex v1 to vertex v2

    :param v1: NumPy array, start point
    :param v2: NumPy array, end point
    :return vector:
    """
    return v2 - v1

def compute_offset_vector(v1,v2, sweep_direction, d):
    """ Compute a vector parallel to the vector from v1 to v2, but offset perpendicularly
    in the direction specified by sweep_direction.

    :param v1: NumPy array, the start point of the original vector
    :param v2: NumPy array, the end point of the original vector
    :param sweep_direction: int, the direction to offset the vector (+1 or -1)
    :param d: float, the distance by which the vector should be offset
    :return new_v1: NumPy array, the new starting point of the offset vector
    :return new_v2: NumPy array, the new ending point of the offset vector
    """
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
    :param v1: NumPy array, the start point of the vector
    :param v2: NumPy array, the end point of the vector
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
    sweep_direction = np.sign(np.dot(perp_vector_normalized, vector_v1_a))  # -1 or +1 depending on which side a is

    return sweep_direction

def extend_vector_to_boundary(v1, v2, boundaries):
    """ Extend the vector from v1 to v2 to intersect with the boundary box

    :param v1: NumPy array, the start point of the original vector
    :param v2: NumPy array, the end point of the original vector
    :param boundaries: NumPy array [min_x, max_x, min_y, max_y] representing the boundary box
    :return extended_v1: NumPy array, the new start point of the extended vector
    :return extended_v2: NumPy array, the new end point of the extended vector
    """
    # Extract the boundary box values
    min_x, max_x, min_y, max_y = boundaries

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

    return extended_v1, extended_v2

def get_path(poly, dx, b_index, b_mate_index, a_index, boundaries):
    """ GetPath algorithm from page 5 in Coverage Path Planning for 2D Convex Regions

    :param poly: Polygon P, using Polygon class
    :param dx: Path width (0.1 = 100 meters)
    :param b_index: Starting point index
    :param b_mate_index: b's counterclockwise neighbour index
    :param a_index: b's diametral antipodal point index
    :param boundaries: P's boundaries
    :return path: A path which fully covers P
    """

    # Getting the three vertices as points from the polygon
    b = poly.vertices[b_index].v
    b_mate = poly.vertices[b_mate_index].v
    a = poly.vertices[a_index].v

    sweep_direction = compute_sweep_direction(b, b_mate, a)

    # First pass is offset from polygon edge with half path width
    delta_init = dx / 2

    FORWARD = True  # Direction of path (true is from b to b_mate)
    # L_flight = create_vector(b, b_mate)
    L_flight = compute_offset_vector(b, b_mate, sweep_direction, delta_init)
    L_flight = extend_vector_to_boundary(L_flight[0], L_flight[1], boundaries)

    # Initialize the empty path
    path = []

    # Fail-safe parameters for the while loop
    max_iterations = 10000
    counter = 0

    # Loop until no intersections is found
    while not (Polygon.find_intersections(poly, L_flight[0], L_flight[1]) == []):
        ip1, ip2 = Polygon.find_intersections(poly, L_flight[0], L_flight[1])

        # Check and connect path
        if FORWARD:
            path.append(ip1)
            path.append(ip2)
            FORWARD = False  # Change direction

        else:
            path.append(ip2)
            path.append(ip1)
            FORWARD = True  # Change direction

        # Computing next extended offset vector, offset with full path width dx
        L_flight = compute_offset_vector(L_flight[0], L_flight[1], sweep_direction, dx)
        L_flight = extend_vector_to_boundary(L_flight[0], L_flight[1], boundaries)

        if counter > max_iterations:
            print(f"Max iterations of {max_iterations} reached")
            break
        counter += 1

    print(f"Computed path after {counter} iterations.")
    path = np.array(path)

    plot_path(b, b_mate, a, sweep_direction, dx, boundaries, poly, path)

    return path

def plot_path(b, b_mate, a, sweep_direction, dx, boundaries, polygon, path):
    """ Plot the original vector from b to b_mate, the offset vector, the boundary box, the polygon,
    including the intersection points between the offset vector and the polygon, and the path points.

    :param b: Start point of the original vector
    :param b_mate: End point of the original vector
    :param a: Diametric point of b, the direction towards which the vector should be offset
    :param sweep_direction: int, direction for the offset vectors
    :param dx: float, the distance by which the vector should be offset
    :param boundaries: array, representing the boundary box [min_x, max_x, min_y, max_y]
    :param polygon: Polygon, the polygon object to be plotted
    :param path: NumPy array, the array of points representing the path [[x1, y1], [x2, y2], ...]
    """
    # Compute the new extended offset vector
    #L_flight = create_vector(b, b_mate)
    new_b, new_b_mate = compute_offset_vector(b, b_mate, sweep_direction, dx/2)
    new_b, new_b_mate = extend_vector_to_boundary(new_b, new_b_mate, boundaries)

    # Find intersections with the polygon
    L_flight_offset_intersections = polygon.find_intersections(new_b, new_b_mate)

    fig, ax = plt.subplots()

    plot_vectors = False
    if plot_vectors:
        # Plot the original vector
        ax.quiver(b[0], b[1], b_mate[0] - b[0], b_mate[1] - b[1], angles='xy', scale_units='xy', scale=1, color='blue',
                  label='Original Vector b -> b_mate')

        # Plot the new, offset vector
        ax.quiver(new_b[0], new_b[1], new_b_mate[0] - new_b[0], new_b_mate[1] - new_b[1], angles='xy', scale_units='xy',
                  scale=1, color='red', label='Offset Vector')

        # Plot the offset vector intersection points
        if L_flight_offset_intersections:
            ix, iy = zip(*L_flight_offset_intersections)
            ax.scatter(ix, iy, color='red', zorder=5, label='Intersections')

    # Plot the boundary box
    min_x, max_x, min_y, max_y = boundaries
    ax.plot([min_x, max_x, max_x, min_x, min_x], [min_y, min_y, max_y, max_y, min_y], 'k--', label='Boundary Box')

    # Plot the points b, b_mate, and a
    ax.plot([b[0], b_mate[0], a[0]], [b[1], b_mate[1], a[1]], 'ro', label='Points b, b_mate, a', markersize=10)

    # Annotate the points
    ax.text(b[0], b[1], f'b', fontsize=16, color='darkblue')
    ax.text(b_mate[0], b_mate[1], 'b_mate', fontsize=16, color='darkblue')
    ax.text(a[0], a[1], "a", fontsize=16, color='darkblue')

    # Plot the polygon
    x_coords, y_coords = polygon.get_coords()
    ax.plot(x_coords, y_coords, 'b-', marker='o', label='Polygon')
    ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'b-')

    # Plot the path
    path_x, path_y = path[:, 0], path[:, 1]
    ax.plot(path_x, path_y, 'g-', marker='x', label='Path', linewidth=3)

    # Highlight the start and end points of the path
    ax.plot(path_x[0], path_y[0], 'go', markersize=10, label='Start Point')  # Start point
    ax.plot(path_x[-1], path_y[-1], 'ro', markersize=10, label='End Point')  # End point

    plt.legend()
    plt.grid(True)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()