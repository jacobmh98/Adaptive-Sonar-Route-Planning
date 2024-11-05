def create_line(p1, p2):
    """Create a line in the form of ax + by = c."""
    a = p2[1] - p1[1]
    b = p1[0] - p2[0]
    c = a * p1[0] + b * p1[1]
    return a, b, c


def offset_line(line, offset):
    """Offset a line by a perpendicular distance."""
    a, b, c = line
    # Calculate the perpendicular distance offset
    norm = np.sqrt(a ** 2 + b ** 2)
    c_offset = c + offset * norm
    return a, b, c_offset


def find_intersection(line1, line2):
    """Find the intersection of two lines represented by ax + by = c."""
    a1, b1, c1 = line1
    a2, b2, c2 = line2
    determinant = a1 * b2 - a2 * b1
    if determinant == 0:
        return None  # Lines are parallel
    x = (b2 * c1 - b1 * c2) / determinant
    y = (a1 * c2 - a2 * c1) / determinant
    return (x, y)


def intersect_edges(L_flight, edges):
    """Find intersections of a line with polygon edges."""
    intersections = []
    for edge in edges:
        # Create line representation for the current edge
        line_edge = create_line(edge[0], edge[1])
        intersection = find_intersection(L_flight, line_edge)
        if intersection:
            x, y = intersection
            # Check if the intersection point is within the segment limits
            if min(edge[0][0], edge[1][0]) <= x <= max(edge[0][0], edge[1][0]) and \
                    min(edge[0][1], edge[1][1]) <= y <= max(edge[0][1], edge[1][1]):
                intersections.append(intersection)
    if len(intersections) >= 2:
        # Sort intersections by their order along the flight line direction
        intersections.sort(key=lambda point: (point[0], point[1]))
        return intersections[0], intersections[1]
    else:
        return None, None


def check_and_connect(path, ip1, ip2):
    """Add intersection points to the path in the correct order."""
    if not path:
        # Initialize path with the first set of intersection points
        path.append(ip1)
        path.append(ip2)
    else:
        # Connect the points in the correct order
        if np.linalg.norm(np.array(path[-1]) - np.array(ip1)) < np.linalg.norm(np.array(path[-1]) - np.array(ip2)):
            path.append(ip1)
            path.append(ip2)
        else:
            path.append(ip2)
            path.append(ip1)
    return path


def get_path(P, dx, b_index, bmate_index, a_index):
    """Compute the Edge-Vertex Path (EVP) for the given polygon."""
    # Store edges as pairs of endpoints (vertices in counterclockwise order)
    edges = [(P[i], P[(i + 1) % len(P)]) for i in range(len(P))]

    # Define the vertices
    b = P[b_index]
    bmate = P[bmate_index]
    a = P[a_index]

    # Initial offset
    offset_init = dx / 2
    # Create initial flight line
    L_flight = create_line(b, bmate)
    # Offset the flight line
    L_flight = offset_line(L_flight, offset_init)

    # Initialize path
    path = []

    # Iterate and intersect the line with the polygon
    while True:
        ip1, ip2 = intersect_edges(L_flight, edges)
        if ip1 and ip2:
            path = check_and_connect(path, ip1, ip2)
            # Move to the next parallel line
            L_flight = offset_line(L_flight, dx)
        else:
            break

    return path