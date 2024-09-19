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

def plot_results(split_polygons, cv, Di):
    """ Create a figure with a grid of sub-plots """

    cols = 5
    rows = int(np.ceil(len(split_polygons) / cols))


    fig, ax = plt.subplots(rows, cols)

    r = 0
    c = 0
    count = 0

    for (P1, P2) in split_polygons:
        P1_coords = P1.vertices_matrix()
        P2_coords = P2.vertices_matrix()
        ax[r, c].plot(P1_coords[0, :], P1_coords[1, :], 'b-')
        ax[r, c].plot([P1_coords[0, :][-1], P1_coords[0, :][0]], [P1_coords[1, :][-1], P1_coords[1, :][0]], 'b-')
        ax[r, c].plot(P2_coords[0, :], P2_coords[1, :], 'r-')
        ax[r, c].plot([P2_coords[0, :][-1], P2_coords[0, :][0]], [P2_coords[1, :][-1], P2_coords[1, :][0]], 'r-')
        ax[r, c].set_title(f'D{cv},{count} = {np.round(Di[count], 1)}')
        ax[r, c].axis('equal')
        count += 1

        # ax[r, c].quiver(cv.x, cv.y, vec[0], vec[1], angles='xy', scale_units='xy', scale=1, color='r', width=0.015)
        # ax[plot_r, c].quiver(cv.x, cv.y, v_dir2[0], v_dir2[1], angles='xy', scale_units='xy', scale=1, color='g')
        c += 1

        if c != 0 and c % cols == 0:
            r += 1
            c = 0
    fig.tight_layout()
    plt.show()

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

def compute_intersection(vec, cv, e2):
    """ Computes the points of intersection (if any) between a vector from a point cv and an edge e2 """
    vx = vec[0, 0]
    vy = vec[1, 0]

    x0 = cv.x
    y0 = cv.y

    x1 = e2.v_from.x
    y1 = e2.v_from.y

    x2 = e2.v_to.x
    y2 = e2.v_to.y

    s_denominator = (y1 * vx - y2 * vx - vy * x1 + vy * x2)

    if s_denominator == 0:
        s = 0
    else:
        s = - ((y0 * vx - y1 * vx - vy * x0 + vy * x1) / s_denominator)
    t = - ((s * x1 - s * x2 + x0 - x1) / vx)

    # Test if the line e intersects the edge e2
    if 0 <= s <= 1:
        # Compute the coordinates of the intersection point
        intersection_point =  cv.get_array() + t * vec

        # Ignore the intersection that happens in the adjacent vertices
        if points_are_equal(intersection_point, cv.get_array()) or \
                points_are_equal(intersection_point, cv.prev.get_array()) or \
                points_are_equal(intersection_point, cv.next.get_array()):
            return None, None
        return intersection_point, t
    return None, None

