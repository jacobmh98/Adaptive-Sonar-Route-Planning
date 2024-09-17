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
