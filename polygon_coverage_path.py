import numpy as np
from Polygon import Polygon, Vertex
import matplotlib.pyplot as plt

def create_vector_between_vertices(v1, v2):
    """ Create a vector from vertex v1 to vertex v2 """
    return v2.v - v1.v  # v is the NumPy array containing [x, y] coordinates


def compute_offset_vector(b, b_mate, a, d):
    """
    Compute a vector parallel to the vector from b to b_mate, but offset perpendicularly towards point a.

    Parameters:
    b: Vertex, the start point of the original vector
    b_mate: Vertex, the end point of the original vector
    a: Vertex, the point towards which the vector should be offset
    dx: float, the distance by which the vector should be offset

    Returns:
    new_b: NumPy array, the new starting point of the offset vector
    new_b_mate: NumPy array, the new ending point of the offset vector
    """
    # Calculate the original vector from b to b_mate
    vector_b_bmate = create_vector_between_vertices(b, b_mate)

    # Calculate the vector from b to a
    vector_b_a = create_vector_between_vertices(b, a)

    # Find the perpendicular direction from b-b_mate towards a
    perp_vector = np.array([-vector_b_bmate[1], vector_b_bmate[0]])

    # Normalize the perpendicular vector and calculate direction towards a
    perp_vector_normalized = perp_vector / np.linalg.norm(perp_vector)
    direction_to_a = np.sign(np.dot(perp_vector_normalized, vector_b_a))  # +1 or -1 depending on which side a is

    # Offset the original vector by dx in the direction of the perpendicular vector towards a
    offset = perp_vector_normalized * d * direction_to_a

    # New starting and ending points for the offset vector
    new_b = b.v + offset
    new_b_mate = b_mate.v + offset

    return new_b, new_b_mate

def extend_vector_to_boundary(v1, v2, boundaries):
    """
    Extend the vector from b to b_mate to intersect with the boundary box.

    Parameters:
    v1: Vertex, the start point of the original vector
    v2: Vertex, the end point of the original vector
    boundaries: NumPy array [min_x, max_x, min_y, max_y] representing the boundary box

    Returns:
    extended_v1: NumPy array, the new start point of the extended vector
    extended_v2: NumPy array, the new end point of the extended vector
    """
    # Extract the boundary box values
    min_x, max_x, min_y, max_y = boundaries

    # Calculate the direction vector from b to b_mate
    direction = v2 - v1

    # Calculate the scaling factors needed to reach the boundary box
    if direction[0] != 0:  # Avoid division by zero for vertical lines
        t_min_x = (min_x - v1[0]) / direction[0]
        t_max_x = (max_x - v1[0]) / direction[0]
    else:
        t_min_x = t_max_x = float('inf')  # No intersection with vertical lines

    if direction[1] != 0:  # Avoid division by zero for horizontal lines
        t_min_y = (min_y - v1[1]) / direction[1]
        t_max_y = (max_y - v1[1]) / direction[1]
    else:
        t_min_y = t_max_y = float('inf')  # No intersection with horizontal lines

    # Find the smallest positive scaling factor to stretch the vector to the boundary
    t_min = max(min(t_min_x, t_max_x), min(t_min_y, t_max_y))
    t_max = min(max(t_min_x, t_max_x), max(t_min_y, t_max_y))

    # Scale the vector to the boundaries
    extended_v1 = v1 + direction * t_min
    extended_v2 = v1 + direction * t_max

    return extended_v1, extended_v2


def get_path(poly, dx, b_index, b_mate_index, a_index, boundaries):
    """
    :param P:
        Polygon P, using Polygon class
    :param dx:
        Path width (0.1 = 100 meters)
    :param b_index:
        Starting point index
    :param b_mate_index:
        b's counterclockwise neighbour index
    :param a_index:
        b's diametral antipodal point index
    :param boundaries:
        P's boundaries
    :return path:
        A path which fully covers P
    """

    # Getting the three points as vertices from the polygon
    b = poly.vertices[b_index]
    b_mate = poly.vertices[b_mate_index]
    a = poly.vertices[a_index]

    # First pass is offset from polygon edge with half path width
    delta_init = dx / 2

    L_flight = create_vector_between_vertices(b, b_mate)
    L_flight_offset = compute_offset_vector(b, b_mate, a, delta_init)
    plot_offset_vector_with_computation(b, b_mate, a, delta_init, boundaries, poly)

    #TODO: Create path as list of vertices



    return 0


def plot_offset_vector_with_computation(b, b_mate, a, d, boundaries, polygon):
    """
    Plot the original vector from b to b_mate, the offset vector, the boundary box, and the polygon.

    Parameters:
    b: Vertex, the start point of the original vector
    b_mate: Vertex, the end point of the original vector
    a: Vertex, the point towards which the vector should be offset
    d: float, the distance by which the vector should be offset
    boundaries: array, representing the boundary box [min_x, max_x, min_y, max_y]
    polygon: Polygon, the polygon object to be plotted
    """
    # Compute the new offset vector
    new_b, new_b_mate = compute_offset_vector(b, b_mate, a, d)
    new_b, new_b_mate = extend_vector_to_boundary(new_b, new_b_mate, boundaries)

    print(b)
    print(new_b)

    #new_b1, new_b_mate1 = compute_offset_vector(new_b, new_b_mate, a, d)
    #new_b1, new_b_mate1 = extend_vector_to_boundary(new_b1, new_b_mate1, boundaries)

    # Plot the original vector
    fig, ax = plt.subplots()
    ax.quiver(b.x, b.y, b_mate.x - b.x, b_mate.y - b.y, angles='xy', scale_units='xy', scale=1, color='blue',
              label='Original Vector b -> b_mate')

    # Plot the new, offset vector
    ax.quiver(new_b[0], new_b[1], new_b_mate[0] - new_b[0], new_b_mate[1] - new_b[1], angles='xy', scale_units='xy',
              scale=1, color='red', label='Offset Vector')

    # Plot the second offset vector
    #ax.quiver(new_b1[0], new_b1[1], new_b_mate1[0] - new_b1[0], new_b_mate1[1] - new_b1[1], angles='xy', scale_units='xy',
    #          scale=1, color='red', label='Offset Vector 2')

    # Plot the boundary box
    min_x, max_x, min_y, max_y = boundaries
    ax.plot([min_x, max_x, max_x, min_x, min_x], [min_y, min_y, max_y, max_y, min_y], 'k--', label='Boundary Box')

    # Plot the points b, b_mate, and a
    ax.plot([b.x, b_mate.x, a.x], [b.y, b_mate.y, a.y], 'go', label='Points b, b_mate, a')

    # Annotate the points
    ax.text(b.x, b.y, f'b ({b.x}, {b.y})', fontsize=12, color='blue')
    ax.text(b_mate.x, b_mate.y, f'b_mate ({b_mate.x}, {b_mate.y})', fontsize=12, color='blue')
    ax.text(a.x, a.y, f'a ({a.x}, {a.y})', fontsize=12, color='green')

    # Plot the polygon
    x_coords, y_coords = polygon.get_coords()
    ax.plot(x_coords, y_coords, 'b-', marker='o', label='Polygon')
    ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'b-')

    plt.legend()
    plt.grid(True)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()
