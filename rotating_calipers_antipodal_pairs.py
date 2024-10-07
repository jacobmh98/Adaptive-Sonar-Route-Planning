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
    """ Computes antipodal pairs for a given polygon.

    :param polygon: Polygon object with a list of vertices.
    :return: List of tuples representing antipodal vertex pairs.
    """
    vertices = polygon.vertices
    n = len(vertices)

    # Antipodal pairs will be stored as tuples of indices (i, j)
    antipodal_pairs = []

    # Start with the vertex farthest from vertex 0
    j = 1
    while dist(vertices[0].v, vertices[(j + 1) % n].v) > dist(vertices[0].v, vertices[j].v):
        j += 1

    # Loop over each vertex i
    for i in range(0,n):
        antipodal_pairs.append((i, j))

        # Move the caliper to the next vertex and check the antipodal condition
        # Keep rotating the caliper to find antipodal pairs
        while dist(vertices[i].v, vertices[(j + 1) % n].v) > dist(vertices[i].v, vertices[j].v):
            j = (j + 1) % n
            antipodal_pairs.append((i, j))

    return antipodal_pairs


def filter_and_remove_redundant_pairs(polygon, antipodal_pairs):
    """
    Filter out neighboring antipodal pairs and remove redundant pairs (i, j) and (j, i).

    For a triangle (3 vertices), neighboring points along the hypotenuse are not removed.

    :param polygon: Polygon object to check neighboring vertices.
    :param antipodal_pairs: List of tuples representing antipodal pairs.
    :return: Filtered list of unique and non-neighboring antipodal pairs.
    """
    n = polygon.number_vertices
    unique_pairs = set()

    # Special case for triangles (3 vertices)
    if n == 3:
        # Get lengths of each edge to identify the hypotenuse
        edges = [
            (0, 1, np.linalg.norm(np.array(polygon.vertices[0].v) - np.array(polygon.vertices[1].v))),
            (1, 2, np.linalg.norm(np.array(polygon.vertices[1].v) - np.array(polygon.vertices[2].v))),
            (2, 0, np.linalg.norm(np.array(polygon.vertices[2].v) - np.array(polygon.vertices[0].v)))
        ]
        # Find the hypotenuse, the edge with the longest length
        hypotenuse = max(edges, key=lambda edge: edge[2])
        hypotenuse_indices = (hypotenuse[0], hypotenuse[1])

        # Keep the hypotenuse neighbors, but continue filtering for others
        for i, j in antipodal_pairs:
            if i == j:  # Avoid adding pairs where both points are the same
                continue

            # Allow neighbors along the hypotenuse
            if (i, j) == hypotenuse_indices or (j, i) == hypotenuse_indices:
                unique_pairs.add((i, j))
            else:
                # Check if i and j are neighbors (for non-triangle cases)
                if abs(i - j) != 1 and abs(i - j) != n - 1:
                    # Add the pair if its reverse doesn't exist
                    if (j, i) not in unique_pairs:
                        unique_pairs.add((i, j))

    # General case for polygons with more than 3 vertices
    else:
        for i, j in antipodal_pairs:
            if i == j:  # Avoid adding pairs where both points are the same
                continue

            # Check if i and j are neighbors, including wrapping around the polygon
            if abs(i - j) == 1 or abs(i - j) == n - 1:
                continue  # Skip neighboring pairs

            # Add the pair if its reverse doesn't exist
            if (j, i) not in unique_pairs:
                unique_pairs.add((i, j))

    return list(unique_pairs)


def get_diametric_antipodal_pair_index(polygon, antipodal_pairs, b):
    """ Find the diametric antipodal pair for vertex b, considering only its antipodal pairs.

    :param antipodal_pairs: List of tuples representing antipodal pairs.
    :param b: Vertex index for which to find the furthest antipodal pair.
    :param polygon: Polygon object with vertices.
    :return: Index of the furthest antipodal vertex for vertex b.
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


def filter_diametric_antipodal_pairs(polygon, antipodal_pairs):
    """ Filter the antipodal pairs to keep only the diametric antipodal pairs for each vertex.

    :param polygon: Polygon object with vertices.
    :param antipodal_pairs: List of tuples representing antipodal pairs.

    :returns: List of tuples representing only the diametric antipodal pairs.
    """
    diametric_pairs = set()  # Using a set to avoid duplicates

    # Loop through all vertices of the polygon
    for b in range(polygon.number_vertices):
        # Find the diametric antipodal pair for vertex b
        diametric_pair = get_diametric_antipodal_pair_index(polygon, antipodal_pairs, b)

        if diametric_pair is not None:
            # Add the pair (b, diametric_pair) in sorted order to avoid (b, a) and (a, b) duplicates
            sorted_pair = tuple(sorted([b, diametric_pair]))
            diametric_pairs.add(sorted_pair)

    #plot_antipodal_points(polygon, diametric_pairs)

    # Convert the set to a list for the final output
    return list(diametric_pairs)


def get_diametric_antipodal_point_index(diametric_antipodal_pairs, b):
    """ Given a point index b, find and return its diametric antipodal point a from the diametric antipodal pairs.

    :param diametric_antipodal_pairs: List of tuples representing diametric antipodal pairs.
    :param b: The index of the point for which to find the diametric antipodal pair.
    :returns: The index of the diametric antipodal point for b, or None if not found.
    """
    for (i, j) in diametric_antipodal_pairs:
        if i == b:
            return j  # Return the corresponding antipodal point
        elif j == b:
            return i  # Return the corresponding antipodal point

    # If no pair is found for the given point
    return None

def plot_antipodal_points(polygon, antipodal_vertices):
    """ Plot the polygon and highlight the antipodal points.

    :param polygon: Polygon object with vertices.
    :param antipodal_vertices: List of tuples representing antipodal vertex pairs.
    """
    # Get the x and y coordinates of the vertices
    x_coords, y_coords = polygon.get_coords()

    # Plot the polygon (ensure the polygon closes by connecting last and first point)
    plt.plot(x_coords + [x_coords[0]], y_coords + [y_coords[0]], 'k-', marker='o')

    # Plot vertex indices for reference
    for v in polygon.vertices:
        plt.text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='blue')

    # Plot the antipodal pairs
    for (i, j) in antipodal_vertices:
        xk = [polygon.vertices[i].x, polygon.vertices[j].x]
        yk = [polygon.vertices[i].y, polygon.vertices[j].y]
        plt.plot(xk, yk, linestyle='--', marker='o', color=[0.7, 0.7, 0.7])

    # Display the plot
    #plt.grid()
    plt.show()