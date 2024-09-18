import numpy as np
import matplotlib.pyplot as plt

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

def plot_results(split_polygons, D_i, cv_index):
    """ Create a figure with a grid of sub-plots """
    rows = 3
    cols = 5

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

def distance(v1, v2):
        """ Computes the Euclidean distance between two numpy vectors v1 and v2 """
        return np.linalg.norm(v1 - v2)

def extend_vectors_to_boundary_box(v_dir1, v_dir2, cv, min_x, max_x, min_y, max_y):
    """ Extend a vector and its negation to the boundary box of a polygon """
    if v_dir1[0] > 0 and v_dir1[1] < 0:  # positive x, negative y
        t1_limx = (max_x - cv.x) / v_dir1[0]
        t1_limy = (min_y - cv.y) / v_dir1[1]
        t1 = np.min(np.array([t1_limx, t1_limy]))

        t2_limx = (min_x - cv.x) / v_dir2[0]
        t2_limy = (max_y - cv.y) / v_dir2[1]
        t2 = np.min(np.array([t2_limx, t2_limy]))

        v_dir1 = v_dir1 * t1
        v_dir2 = v_dir2 * t2
    elif v_dir1[0] > 0 and v_dir1[1] == 0:  # positive x, neutral y
        t1 = (max_x - cv.x) / v_dir1[0]
        t2 = (min_x - cv.x) / v_dir2[0]

        v_dir1 = v_dir1 * t1
        v_dir2 = v_dir2 * t2
    elif v_dir1[0] > 0 and v_dir1[1] > 0:  # positive x, positive y
        t1_limx = (max_x - cv.x) / v_dir1[0]
        t1_limy = (max_y - cv.y) / v_dir1[1]
        t1 = np.min(np.array([t1_limx, t1_limy]))

        t2_limx = (min_x - cv.x) / v_dir2[0]
        t2_limy = (min_y - cv.y) / v_dir2[1]
        t2 = np.min(np.array([t2_limx, t2_limy]))

        v_dir1 = v_dir1 * t1
        v_dir2 = v_dir2 * t2
    elif v_dir1[0] == 0 and v_dir1[1] > 0:  # neutral x, positive y
        t1 = (max_y - cv.x) / v_dir1[0]
        t2 = (min_y - cv.x) / v_dir2[0]

        v_dir1 = v_dir1 * t1
        v_dir2 = v_dir2 * t2
    elif v_dir1[0] < 0 and v_dir1[1] > 0:  # negative x, positive y
        t1_limx = (min_x - cv.x) / v_dir1[0]
        t1_limy = (max_y - cv.y) / v_dir1[1]
        t1 = np.min(np.array([t1_limx, t1_limy]))

        t2_limx = (max_x - cv.x) / v_dir2[0]
        t2_limy = (min_y - cv.y) / v_dir2[1]
        t2 = np.min(np.array([t2_limx, t2_limy]))

        v_dir1 = v_dir1 * t1
        v_dir2 = v_dir2 * t2
    elif v_dir1[0] < 0 and v_dir1[1] == 0:  # negative x, neutral y
        t1 = (min_x - cv.x) / v_dir1[0]
        t2 = (max_x - cv.x) / v_dir2[0]

        v_dir1 = v_dir1 * t1
        v_dir2 = v_dir2 * t2
    elif v_dir1[0] < 0 and v_dir1[0] < 0:  # negative x, negative y
        t1_limx = (min_x - cv.x) / v_dir1[0]
        t1_limy = (min_y - cv.y) / v_dir1[1]
        t1 = np.min(np.array([t1_limx, t1_limy]))

        t2_limx = (max_x - cv.x) / v_dir2[0]
        t2_limy = (max_y - cv.y) / v_dir2[1]
        t2 = np.min(np.array([t2_limx, t2_limy]))

        v_dir1 = v_dir1 * t1
        v_dir2 = v_dir2 * t2
    elif v_dir1[0] == 0 and v_dir1[1] < 0:  # neutral x, negative y
        t1 = (min_y - cv.x) / v_dir1[0]
        t2 = (max_y - cv.x) / v_dir2[0]

        v_dir1 = v_dir1 * t1
        v_dir2 = v_dir2 * t2

    return v_dir1.reshape(-1, 1), v_dir2.reshape(-1, 1)

def points_are_equal(P1, P2, epsilon=1e-2):
    """ Checks if two points P1 and P2 are approximately equal within a tolerance epsilon. """
    # Check if the distance between the points is smaller than epsilon
    return np.all(np.abs(P1 - P2) < epsilon)