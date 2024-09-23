import numpy as np
import matplotlib.pyplot as plt

def dist(p1, p2):
    """ Distance function to calculate distance between two points
    :param p1: point one
    :param p2: point two
    :return: float, distance between p1 and p2
    """
    return np.linalg.norm(np.array(p1) - np.array(p2))

def compute_antipodal_pairs(polygon):
    """
    Computes antipodal pairs for a given polygon.

    Args:
        polygon: Polygon object with a list of vertices.

    Returns:
        A list of tuples representing antipodal vertex pairs.
    """
    vertices = polygon.vertices
    n = len(vertices)

    # Antipodal pairs will be stored as tuples of indices (i, j)
    antipodal_pairs = []

    # Start with the vertex farthest from vertex 0
    j = 1
    while dist(vertices[0].v, vertices[(j + 1) % n].v) > dist(vertices[0].v, vertices[j].v):
        j += 1

    # Now loop over each vertex i
    for i in range(n):
        antipodal_pairs.append((i, j))

        # Move the caliper to the next vertex and check the antipodal condition
        # Keep rotating the caliper to find antipodal pairs
        while dist(vertices[i].v, vertices[(j + 1) % n].v) > dist(vertices[i].v, vertices[j].v):
            j = (j + 1) % n
            antipodal_pairs.append((i, j))

    return antipodal_pairs


def filter_and_remove_redundant_pairs(polygon, antipodal_pairs):
    """ Filter out neighboring antipodal pairs and remove redundant pairs (i, j) and (j, i).

    Args:
        antipodal_pairs: List of tuples representing antipodal pairs.
        polygon: Polygon object to check neighboring vertices.

    Returns:
        A filtered list of unique and non-neighboring antipodal pairs.
    """
    n = polygon.number_vertices
    unique_pairs = set()

    for i, j in antipodal_pairs:
        # Filter out neighboring pairs
        if abs(i - j) % n != 1:
            # Add the pair if its reverse doesn't exist
            if (j, i) not in unique_pairs:
                unique_pairs.add((i, j))

    return list(unique_pairs)


def get_diametric_antipodal_pair_index(polygon, antipodal_pairs, b):
    """ Find the diametric antipodal pair for vertex b, considering only its antipodal pairs.

    Args:
        antipodal_pairs: List of tuples representing antipodal pairs.
        b: Vertex index for which to find the furthest antipodal pair.
        polygon: Polygon object with vertices.

    Returns:
        The index of the furthest antipodal vertex for vertex b.
    """
    max_distance = -1
    diametric_antipode = None

    # Loop through the antipodal pairs and find the furthest point for vertex b
    for (i, j) in antipodal_pairs:
        if i == b:
            distance = dist(polygon.vertices[b].v, polygon.vertices[j].v)
            if distance > max_distance:
                max_distance = distance
                diametric_antipode = j
        elif j == b:
            distance = dist(polygon.vertices[b].v, polygon.vertices[i].v)
            if distance > max_distance:
                max_distance = distance
                diametric_antipode = i

    return diametric_antipode

def plot_antipodal_points(polygon, antipodal_vertices):
    """
    Plot the polygon and highlight the antipodal points.

    Args:
        polygon: Polygon object with vertices.
        antipodal_vertices: List of tuples representing antipodal vertex pairs.
    """
    # Get the x and y coordinates of the vertices
    x_coords, y_coords = polygon.get_coords()

    # Plot the polygon (ensure the polygon closes by connecting last and first point)
    plt.plot(x_coords + [x_coords[0]], y_coords + [y_coords[0]], 'b-', marker='o')

    # Plot vertex indices for reference
    for v in polygon.vertices:
        plt.text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='red')

    # Plot the antipodal pairs
    for (i, j) in antipodal_vertices:
        xk = [polygon.vertices[i].x, polygon.vertices[j].x]
        yk = [polygon.vertices[i].y, polygon.vertices[j].y]
        plt.plot(xk, yk, linestyle='--', marker='o', color=[0.7, 0.7, 0.7])

    # Display the plot
    plt.grid()
    plt.show()
