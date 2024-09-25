import numpy as np
import matplotlib.pyplot as plt

def signed_triangle_area(v1, v2, v3):
    """
    Get signed area of triangle from three vertices v1, v2, and v3

    Returns:
    float: Signed area of the triangle created by the three vertices.
    """
    return (v2[0] - v1[0]) * (v3[1] - v1[1]) - (v3[0] - v1[0]) * (v2[1] - v1[1])

def simple_polygon_orientation(V):
    """
    Determine vertex order for a simple polygon.

    Parameters:
    V : ndarray
        Px2 array of (x, y) vertex coordinates.

    Returns:
    float: Positive if counterclockwise, negative if clockwise, 0 for degenerate cases.
    """
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

def antipodal_vertice_pairs(S):
    """
    Find antipodal vertex pairs for a given polygon.

    Parameters:
    S : ndarray
        Px2 array of (x, y) vertex coordinates for the polygon.

    Returns:
    pq : ndarray
        Mx2 array representing pairs of antipodal vertices in S.
    """
    n = S.shape[0]

    # Remove duplicate last vertex if the polygon is closed
    if np.array_equal(S[0, :], S[-1, :]):
        S = S[:-1, :]
        n -= 1

    # Ensure the polygon is in counterclockwise order
    if simple_polygon_orientation(S) < 0:
        S = np.flipud(S)

    # Helper functions
    area = lambda i, j, k: signed_triangle_area(S[i, :], S[j, :], S[k, :])
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

    # Reorder pairs if the original polygon was clockwise
    if simple_polygon_orientation(S) < 0:
        pp = [n - 1 - i for i in pp]
        qq = [n - 1 - i for i in qq]

    pq = np.column_stack((pp, qq))
    return pq

def remove_parallel_antipodal_vertices(P, pq):
    """
    Removing anitpodal points if they are parallel neighbours
    :param P:
        Polygon
    :param pq:
        List of antipodal points in P
    :return remainder:
        List of antipodal points in P with parallel neighbours removed
    """
    pq = np.array(pq)
    remainder = []
    n = P.shape[0] - 1  # Number of points in P, -1 for array index

    for pair in pq:
        # Checking if pair is neighbours
        if (pair[0] + 1 == pair[1]) or (pair[0] - 1 == pair[1]):
            if not (P[pair[0]][0] == P[pair[1]][0] or P[pair[0]][1] == P[pair[1]][1]):
                remainder.append(pair)

        # Edge cases if pair has first or last point in polygon
        elif (pair[0] == 0 and pair[1] == n) or (pair[0] == n and pair[1] == 0):
            if not (P[pair[0]][0] == P[pair[1]][0] or P[pair[0]][1] == P[pair[1]][1]):
                remainder.append(pair)
        else:
            remainder.append(pair)

    return np.array(remainder)

def get_diametral_antipodal_point(P, pq, a):
    """
    diametral point B is the point for which the Euclidean distance d(A,B) is maximized.

    :param P:
        Convex polygon
    :param pq:
        List of antipodal points from P
    :param a:
        Index of point for which its diametral point b is sought after
    :return b:
        Index of a's diametral point b
    """

    dist = 0
    diametral_pair = pq[0]

    for pair in pq:

        # Check if either point in antipodal pair is a
        if pair[0] == a or pair[1] == a:

            # Calculate dist between the two points in the pair
            new_dist = np.linalg.norm(P[pair[0]] - P[pair[1]])

            # Update pair if a new further distance is found
            if new_dist > dist:
                dist = new_dist
                diametral_pair = pair

    # Return b from the diametral pair found
    if diametral_pair[0] == a:
        return diametral_pair[1]
    else:
        return diametral_pair[0]

# get_path helper functions
def create_vector(p1, p2):
    """ Create a line in the form of a vector
    :param p1:
    :param p1, p2:
        2D points represented as numpy arrays
    """

    return p2-p1

def normalize_vector(v):
    """ Returns the unit vector of the input vector v"""
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def offset_line_towards_point(vector, fixed_offset, point_a):
    """
    Offsets the extended vector by a fixed amount towards a new point 'a'.

    :param vector: List containing the start and end points of the extended vector
    :param fixed_offset: The distance by which to offset the vector
    :param point_a: The point towards which the vector should be offset
    :return: The new extended vector after offsetting
    """
    point_min, point_max = vector

    # Find the midpoint of the extended vector
    midpoint = (point_min + point_max) / 2

    # Calculate the direction from the midpoint to point a
    direction_to_a = point_a - midpoint

    # Normalize the direction vector
    direction_to_a_unit = normalize_vector(direction_to_a)

    # Offset the vector by moving it along the direction to 'a'
    offset_vector = direction_to_a_unit * fixed_offset

    # Apply the offset to both points of the extended vector
    new_point_min = point_min + offset_vector
    new_point_max = point_max + offset_vector

    return new_point_min, new_point_max

def find_extended_points(p1, p2, boundaries):
    """
    Extends the vector defined by p1 and p2 to intersect the provided boundaries.
    formula: (x,y)=(x1,y1 )+t⋅(dx,dy)

    :param p1: First point of the vector (numpy array)
    :param p2: Second point of the vector (numpy array)
    :param boundaries: List of form [min_x, max_x, min_y, max_y]
    """
    vector = create_vector(p1, p2)
    dx, dy = vector

    # Calculate t values for x and y bounds
    t_min_x = (boundaries[0] - p1[0]) / dx if dx != 0 else -np.inf
    t_max_x = (boundaries[1] - p1[0]) / dx if dx != 0 else np.inf
    t_min_y = (boundaries[2] - p1[1]) / dy if dy != 0 else -np.inf
    t_max_y = (boundaries[3] - p1[1]) / dy if dy != 0 else np.inf

    # Find the minimum and maximum t values within the boundaries
    t_min = max(min(t_min_x, t_max_x), min(t_min_y, t_max_y))
    t_max = min(max(t_min_x, t_max_x), max(t_min_y, t_max_y))

    # Calculate the extended points
    point_min = p1 + t_min * vector
    point_max = p1 + t_max * vector

    return point_min, point_max


def get_path(P, dx, b, b_mate, a, boundaries):
    path = []
    d_init = dx / 2
    b = P[b]
    b_mate = P[b_mate]
    a = P[a]

    L_flight = create_vector(b, b_mate)
    point_min, point_max = find_extended_points(b, b_mate, boundaries)
    extended_L_flight = create_vector(point_min, point_max)

    # Offsetting vector by d_init distance towards point a
    new_point_min, new_point_max = offset_line_towards_point(L_flight, d_init, a)
    extended_L_flight_offset = create_vector(new_point_min, new_point_min)

    plot_offset_vector([point_min, point_max], [new_point_min, new_point_max], a)
    return 0

def plot_offset_vector(original_vector, offset_vector, point_a):
    """ Plots the original vector, offset vector, and point a for visualization """
    point_min, point_max = original_vector
    new_point_min, new_point_max = offset_vector

    plt.figure()

    # Plot original vector
    plt.plot([point_min[0], point_max[0]], [point_min[1], point_max[1]], 'g-', label='Original Vector')

    # Plot offset vector
    plt.plot([new_point_min[0], new_point_max[0]], [new_point_min[1], new_point_max[1]], 'r-', label='Offset Vector')

    # Plot point 'a'
    plt.scatter(point_a[0], point_a[1], color='green', label='Point A')

    plt.legend()
    plt.grid(True)
    plt.title("Original Vector and Offset Vector")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")

    plt.show()



def plot_vector(p1, p2, vector, boundaries):
    plt.figure()
    plt.scatter(*p1, color='blue', label='Point 1')
    plt.scatter(*p2, color='green', label='Point 2')
    plt.plot([vector[0][0], vector[1][0]], [vector[0][1], vector[1][1]], color='red', label='Extended Vector')
    plt.xlim(boundaries[0], boundaries[1])
    plt.ylim(boundaries[2], boundaries[3])
    plt.axhline(0, color='black',linewidth=0.5)
    plt.axvline(0, color='black',linewidth=0.5)
    plt.grid(True)
    plt.show()