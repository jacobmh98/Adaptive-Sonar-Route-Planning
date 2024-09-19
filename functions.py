import numpy as np
import matplotlib.pyplot as plt
from Polygon import *

def compute_concave_vertices(P):
    """ Function to compute the concave vertices in a polygon """
    concave_vertices = []

    for i in range(P.number_vertices):
        # Find the adjacent vertices (ccw order)
        v_left = P.vertices[(i - 1) % P.number_vertices]
        v = P.vertices[i]
        v_right = P.vertices[(i + 1) % P.number_vertices]

        # Computing the concave judgement matrix
        S_vi = np.linalg.det(np.array([[v_left.x, v_left.y, 1],
                                       [v.x, v.y, 1],
                                       [v_right.x, v_right.y, 1]]))

        # Test if the vertex is concave and add it to the list if true
        if S_vi < 0:
            concave_vertices.append(v)

    return concave_vertices

def project_onto_direction(vertices, direction):
    """ Function to project points onto a direction vector """
    projections = np.dot(vertices.T, direction)
    return np.max(projections) - np.min(projections)

def compute_width_sum(P):
    """ Function to compute width sum of a polygon """
    # Convert vertices to a numpy array for easier manipulation
    vertices = P.vertices_matrix()

    #print(f'{vertices.shape=}')

    # Define directions to project onto (e.g., x and y axes)
    directions = [
        np.array([1, 0]),  # X-axis
        np.array([0, 1]),  # Y-axis
    ]

    # Compute width sum
    width_sum = 0
    for direction in directions:
        width_sum += project_onto_direction(vertices, direction)

    return width_sum

def plot_results(split_polygons, cv):
    """ Create a figure with a grid of sub-plots """

    cols = 5
    rows = int(np.ceil(len(split_polygons) / cols))


    fig, ax = plt.subplots(rows, cols)

    r = 0
    c = 0

    for (P1, P2) in split_polygons:
        P1_coords = P1.vertices_matrix()
        P2_coords = P2.vertices_matrix()
        ax[r, c].plot(P1_coords[0, :], P1_coords[1, :], 'b-')
        ax[r, c].plot([P1_coords[0, :][-1], P1_coords[0, :][0]], [P1_coords[1, :][-1], P1_coords[1, :][0]], 'b-')
        ax[r, c].plot(P2_coords[0, :], P2_coords[1, :], 'r-')
        ax[r, c].plot([P2_coords[0, :][-1], P2_coords[0, :][0]], [P2_coords[1, :][-1], P2_coords[1, :][0]], 'r-')
        ax[r, c].set_title(f'{cv=}')
        ax[r, c].axis('equal')

        # ax[r, c].quiver(cv.x, cv.y, vec[0], vec[1], angles='xy', scale_units='xy', scale=1, color='r', width=0.015)
        # ax[plot_r, c].quiver(cv.x, cv.y, v_dir2[0], v_dir2[1], angles='xy', scale_units='xy', scale=1, color='g')
        c += 1

        if c != 0 and c % cols == 0:
            r += 1
            c = 0
    fig.tight_layout()
    plt.show()
    """
    r = -1
    fig, axs = plt.subplots(3, 5, figsize=(12, 8))

    for c, (P1, P2) in enumerate(split_polygons):
        if c % cols == 0:
            r += 1

        P1_xcoords, P1_ycoords = P1.get_coords()
        P2_xcoords, P2_ycoords = P2.get_coords()

        axs[r, c % cols].plot(P1_xcoords, P1_ycoords, 'b-', marker='o')
        axs[r, c % cols].plot([P1_xcoords[-1], P1_xcoords[0]], [P1_ycoords[-1], P1_ycoords[0]], 'b-')
        axs[r, c % cols].plot(P2_xcoords, P2_ycoords, 'r-', marker='o')
        axs[r, c % cols].plot([P2_xcoords[-1], P2_xcoords[0]], [P2_ycoords[-1], P2_ycoords[0]], 'r-')
        axs[r, c % cols].set_title(f'D({cv_index},{c}) = {np.round(D_i[c], 1)}')

    plt.tight_layout()
    """

def distance(v1, v2):
        """ Computes the Euclidean distance between two numpy vectors v1 and v2 """
        return np.linalg.norm(v1 - v2)

def points_are_equal(P1, P2, epsilon=1e-2):
    """ Checks if two points P1 and P2 are approximately equal within a tolerance epsilon. """
    # Check if the distance between the points is smaller than epsilon
    return np.all(np.abs(P1 - P2) < epsilon)

def split_polygon_single(e2, intersection_p, cv):
    """ Split a polygon based on a single intersection point """
    v0 = Vertex(0, intersection_p[0, 0], intersection_p[1, 0])
    P1_vertices = [v0]
    v_next = e2.v_to
    v_index = 1
    while v_next != cv:
        P1_vertices.append(Vertex(v_index, v_next.x, v_next.y))
        v_index += 1
        v_next = v_next.next

    vn = Vertex(v_index, v_next.x, v_next.y)
    P1_vertices.append(vn)

    v0 = Vertex(0, cv.x, cv.y)
    P2_vertices = [v0]
    v_next = cv.next
    v_index = 1

    while v_next != e2.v_to:
        P2_vertices.append(Vertex(v_index, v_next.x, v_next.y))
        v_index += 1
        v_next = v_next.next

    vn = Vertex(v_index, intersection_p[0, 0], intersection_p[1, 0])
    P2_vertices.append(vn)

    P1 = Polygon(P1_vertices)
    P2 = Polygon(P2_vertices)
    return P1, P2

def split_polygon_mult_dirs(e2_1, e2_2, intersection_p1, intersection_p2):
    v0 = Vertex(0, intersection_p1[0, 0], intersection_p1[1, 0])
    P1_vertices = [v0]
    v_next = e2_1.v_to
    v_index = 1
    while v_next != e2_2.v_to:
        P1_vertices.append(Vertex(v_index, v_next.x, v_next.y))
        v_index += 1
        v_next = v_next.next

    vn = Vertex(v_index, intersection_p2[0, 0], intersection_p2[1, 0])
    P1_vertices.append(vn)

    v0 = Vertex(0, intersection_p2[0, 0], intersection_p2[1, 0])
    P2_vertices = [v0]
    v_next = e2_2.v_to
    v_index = 1

    while v_next != e2_1.v_to:
        P2_vertices.append(Vertex(v_index, v_next.x, v_next.y))
        v_index += 1
        v_next = v_next.next

    vn = Vertex(v_index, intersection_p1[0, 0], intersection_p1[1, 0])
    P2_vertices.append(vn)

    P1 = Polygon(P1_vertices)
    P2 = Polygon(P2_vertices)
    return P1, P2

def compute_best_t_value(intersection_directions, dir):
    """ Compute the t-value that gives the shortest distance to the intersection point """
    # Positive direction
    best_t = None
    index = -1

    if dir:
        for i, t in enumerate(intersection_directions):
            if best_t is None and t >= 0:
                best_t = t
                index = i
            elif t >= 0 and t < best_t:
                best_t = t
                index = i
    # Negative direction
    else:
        for i, t in enumerate(intersection_directions):
            if best_t is None and t < 0:
                best_t = t
                index = i
            elif t < 0 and t > best_t:
                best_t = t
                index = i
    return index