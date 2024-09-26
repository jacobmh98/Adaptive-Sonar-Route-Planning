import numpy as np

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

    return 0