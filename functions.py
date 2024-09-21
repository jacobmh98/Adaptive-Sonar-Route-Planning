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

#TODO this is incorrect and needs to be fixed
def compute_polygon_width(P):
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

def plot_results2(P, P1, P2, depth, cv, edge, Dij):
    fig, ax = plt.subplots(1, 4)

    P1_coords = P1.vertices_matrix()
    P2_coords = P2.vertices_matrix()
    P_coords = P.vertices_matrix()

    for v in P.vertices:
        ax[0].text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex

    ax[0].plot(P_coords[0, :], P_coords[1, :], color='black', marker='o')
    ax[0].plot(P_coords[0, :], P_coords[1, :], 'k-')
    ax[0].plot(P_coords[0, :], P_coords[1, :], 'k-')
    ax[0].plot([P_coords[0, :][-1], P_coords[0, :][0]], [P_coords[1, :][-1], P_coords[1, :][0]], 'k-')
    #ax[0].plot(P2_coords[0, :], P2_coords[1, :], 'r-')
    #ax[0].plot([P2_coords[0, :][-1], P2_coords[0, :][0]], [P2_coords[1, :][-1], P2_coords[1, :][0]], 'r-')
    ax[0].set_title(f'P')
    ax[0].axis('equal')

    #ax[1].plot(P1_coords[0, :], P1_coords[1, :], color='blue', marker='o')
    ax[1].plot(P1_coords[0, :], P1_coords[1, :], 'b-o')
    ax[1].plot([P1_coords[0, :][-1], P1_coords[0, :][0]], [P1_coords[1, :][-1], P1_coords[1, :][0]], 'b-o')
    ax[1].plot(P2_coords[0, :], P2_coords[1, :], 'r-o')
    ax[1].plot([P2_coords[0, :][-1], P2_coords[0, :][0]], [P2_coords[1, :][-1], P2_coords[1, :][0]], 'r-o')
    ax[1].set_title(f'P1 & P2')
    ax[1].axis('equal')

#    ax[2].plot(P1_coords[0, :], P1_coords[1, :], color='black', marker='o')
    ax[2].plot(P1_coords[0, :], P1_coords[1, :], 'b-o')
    ax[2].plot([P1_coords[0, :][-1], P1_coords[0, :][0]], [P1_coords[1, :][-1], P1_coords[1, :][0]], 'b-o')
    ax[2].set_title(f'P1')
    ax[2].axis('equal')

    #ax[3].plot(P2_coords[0, :], P2_coords[1, :], color='black', marker='o')
    ax[3].plot(P2_coords[0, :], P2_coords[1, :], 'r-o')
    ax[3].plot([P2_coords[0, :][-1], P2_coords[0, :][0]], [P2_coords[1, :][-1], P2_coords[1, :][0]], 'r-o')
    ax[3].set_title(f'P2')
    ax[3].axis('equal')

    print(f'{depth=}')
    print(f'\t{cv=}')
    print(f'\t{edge=}')
    print(f'\tD_ij={np.round(Dij, 1)}')
    #print(f'\tP1 = {P1.vertices}')
    #print(f'\tP2 = {P2.vertices}')

    fig.tight_layout()
    #mng = plt.get_current_fig_manager()
    #mng.full_screen_toggle()

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
                points_are_equal(intersection_point, cv.next.get_array()) or \
                points_are_equal(vec / np.linalg.norm(vec), (e2.v_to.get_array() - e2.v_from.get_array()) / np.linalg.norm(e2.v_to.get_array() - e2.v_from.get_array())):
            return None, None

        """if cv.index == e2.v_to.index or \
                cv.index == e2.v_from.index or \
            cv.prev.index == e2.v_to.index or \
                cv.next.index == e2.v_from.index:
            return None, None"""
        return intersection_point, t
    return None, None

def find_min_value_matrix(D):
    """ Find the minimum value as well as its indices (i,j) in a matrix """
    min_v = np.inf
    min_indices = (-1, -1)

    for i in range(D.shape[0]):
        for j in range(D.shape[1]):
            if D[i, j] < min_v:
                min_v = D[i, j]
                min_indices = (i, j)
    return min_v, min_indices

def is_valid_polygon(P):
    """ Test if a polygon is a valid """
    # TODO think of more criteria of a valid polygon
    if len(P.vertices) < 3:
        return False
    return True

def split_polygon(P, depth=0):
    # Compute the concave vertices
    P.concave_vertices = compute_concave_vertices(P)
    ncc = len(P.concave_vertices)
    n = len(P.vertices)

    # Base case: if the polygon is convex, return it
    if ncc == 0:
        return [P]

    if depth == 1:
        return []

    #print(f'concave vertices = {P.concave_vertices}')

    # Initialize the width sum matrix
    D = np.empty((ncc, n))
    D_polygons = []

    # Go through each concave vertex
    for i, cv in enumerate(P.concave_vertices):
        print(f"checking for {cv.index=} with coord = ({cv.x}, {cv.y})")
        split_polygons = []

        # Check lines which passes the concave vertex i and parallels edge e
        for j, e in enumerate(P.edges):
            intersection_points = []
            intersection_edges = []
            intersection_directions = []

            print(f'\tchecking edge {e}')

            # Define a vector from the vertices in edge e
            vec = e.v_to.get_array() - e.v_from.get_array()
            # vec = -vec

            # Go through each edge in the P
            for e2 in P.edges:
                if e == e2:
                    continue

                # Compute intersection with edge e2 (if any)
                ip, t = compute_intersection(vec, cv, e2)
                if ip is not None:
                    print(f'\t\t{e} intersects {e2} at ({ip[0,0]}, {ip[1,0]})), {t=}')
                    intersection_points.append(ip)
                    intersection_edges.append(e2)
                    intersection_directions.append(t)

            min_index = np.argmin(np.abs(intersection_directions))

            P1, P2 = split_polygon_single(intersection_edges[min_index], intersection_points[min_index], cv)

            # Compute the width sum of P1 and P2
            D[i, j] = compute_polygon_width(P1) + compute_polygon_width(P2)

            split_polygons.append((P1, P2))

        plot_results(split_polygons, cv.index, D[i, :])
        D_polygons.append(split_polygons)

    # Select the best split of the polygon (lowest width sum)
    D_ij, (cv, edge) = find_min_value_matrix(D)
    P1, P2 = D_polygons[cv][edge]

    print(f'{D_ij=}')
    print(f'i = {cv}')
    print(f'j = {edge}')

    #plot_results2(P, P1, P2, depth, cv, edge, D_ij)
    """
    # Recursively split both sub-polygons if the polygons are valid
    result1 = []
    result2 = []
    if is_valid_polygon(P1):
        result1 = split_polygon(P1, depth + 1)

    if is_valid_polygon(P2):
        result2 = split_polygon(P2, depth + 1)


    # Combine both lists into one and return it
    return result1 + result2
    """
    return None