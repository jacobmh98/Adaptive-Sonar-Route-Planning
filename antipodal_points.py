import numpy as np
import matplotlib.pyplot as plt


def signed_triangle_area(v1, v2, v3):
    """ Compute signed area of triangle created from the three vertices v1, v2, and v3

    :return: float: Signed area of the triangle created by the three vertices.
    """
    return (v2[0] - v1[0]) * (v3[1] - v1[1]) - (v3[0] - v1[0]) * (v2[1] - v1[1])


def simple_polygon_orientation(polygon):
    """ Determine vertex order for a simple polygon.

    :param polygon : Polygon, a polygon object with vertices
    :return: float, positive if counterclockwise, negative if clockwise, 0 for degenerate cases.
    """
    V = polygon.vertices_matrix().T
    n = V.shape[0]

    if n < 3:
        return 0

    x = V[:, 0]
    y = V[:, 1]
    ymin = np.min(y)
    y_idx = np.where(y == ymin)[0]

    if y_idx.size == 1:
        idx = y_idx[0]
    else:
        idx = y_idx[np.argmax(x[y_idx])]

    if idx == 0:
        return signed_triangle_area(V[-1, :], V[0, :], V[1, :])
    elif idx == n - 1:
        return signed_triangle_area(V[-2, :], V[-1, :], V[0, :])
    else:
        return signed_triangle_area(V[idx - 1, :], V[idx, :], V[idx + 1, :])


def antipodal_vertice_pairs(polygon):
    """ Find antipodal vertex pairs for a given polygon.

    :param polygon : Polygon, a polygon object.
    :return pq : ndarray, Mx2 array representing pairs of antipodal vertices in the polygon.
    """
    V = polygon.vertices_matrix().T
    n = V.shape[0]

    # Ensure the polygon is in counterclockwise order
    if simple_polygon_orientation(polygon) < 0:
        V = np.flipud(V)

    # Helper functions
    area = lambda i, j, k: signed_triangle_area(V[i, :], V[j, :], V[k, :])
    next_vertex = lambda i: (i + 1) % n

    # Initialize variables
    p = n - 1
    p0 = next_vertex(p)
    q = next_vertex(p)
    pp, qq = [], []  # pp stores points, qq stores the antipodal points

    # Find antipodal pairs
    while area(p, next_vertex(p), next_vertex(q)) >= area(p, next_vertex(p), q):
        q = next_vertex(q)
    q0 = q

    while q != p0:
        p = next_vertex(p)
        pp.append(p)
        qq.append(q)

        while area(p, next_vertex(p), next_vertex(q)) >= area(p, next_vertex(p), q):
            q = next_vertex(q)
            if [p, q] != [q0, p0]:
                pp.append(p)
                qq.append(q)
            else:
                break

        if area(p, next_vertex(p), next_vertex(q)) == area(p, next_vertex(p), q):
            if [p, q] != [q0, n - 1]:
                pp.append(p)
                qq.append(next_vertex(q))
                break
            else:
                break

    pq = np.column_stack((pp, qq))
    return pq


def remove_parallel_antipodal_vertices(polygon, pq):
    """ Removing antipodal points if they are parallel neighbors.

    :param polygon: Polygon
    :param pq: List of antipodal points in polygon
    :return: List of antipodal points with parallel neighbors removed
    """
    remainder = []
    n = polygon.number_vertices

    for pair in pq:
        v0 = polygon.vertices[pair[0]]
        v1 = polygon.vertices[pair[1]]

        # Checking if pair is neighbors
        if (pair[0] + 1 == pair[1]) or (pair[0] - 1 == pair[1]) or (pair[0] == 0 and pair[1] == n - 1) or (
                pair[1] == 0 and pair[0] == n - 1):
            if not (v0.x == v1.x or v0.y == v1.y):
                remainder.append(pair)
        else:
            remainder.append(pair)

    return np.array(remainder)


def get_diametral_antipodal_point(polygon, pq, b):
    """ Diametral point B is the point for which the Euclidean distance d(A,B) is maximized.

    :param polygon: Polygon
    :param pq: List of antipodal points from the polygon
    :param b: Index of point for which its diametral point a is sought after
    :return: Index of b's diametral point a
    """
    dist = 0
    diametral_pair = pq[0]

    for pair in pq:
        # Check if either point in antipodal pair is a
        if pair[0] == b or pair[1] == b:
            # Calculate dist between the two points in the pair
            v0 = polygon.vertices[pair[0]]
            v1 = polygon.vertices[pair[1]]
            new_dist = np.linalg.norm(v0.v - v1.v)

            # Update pair if a new further distance is found
            if new_dist > dist:
                dist = new_dist
                diametral_pair = pair

    if diametral_pair[0] == b:
        return diametral_pair[1]
    else:
        return diametral_pair[0]


def plot_antipodal_points(polygon, antipodal_vertices):
    """ Plot the polygon and highlight the antipodal points """
    x_coords, y_coords = polygon.get_coords()

    plt.plot(x_coords, y_coords, 'b-', marker='o')  # Connect back to the first point to close the polygon
    plt.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'b-')

    for v in polygon.vertices:
        plt.text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='red')

    for k in range(antipodal_vertices.shape[0]):
        xk = [polygon.vertices[antipodal_vertices[k, 0]].x, polygon.vertices[antipodal_vertices[k, 1]].x]
        yk = [polygon.vertices[antipodal_vertices[k, 0]].y, polygon.vertices[antipodal_vertices[k, 1]].y]
        plt.plot(xk, yk, linestyle='--', marker='o', color=[0.7, 0.7, 0.7])

    plt.grid()
    plt.show()
