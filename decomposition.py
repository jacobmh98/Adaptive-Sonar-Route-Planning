import copy

from Polygon import *
from scipy.spatial import ConvexHull
from global_variables import *

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
            # Check the angle is not 180 deg
            vec1 = v_left.get_array() - v.get_array()
            vec2 = v_right.get_array() - v.get_array()

            dot_product = dot(vec1, vec2)
            neg_mag = -np.linalg.norm(vec1) * np.linalg.norm(vec2)

            if not (dot_product - epsilon <= neg_mag <= dot_product + epsilon):
                concave_vertices.append(v)

    return concave_vertices

def get_center_of_polygon(P):
    """ Compute the center of a polygon"""
    x_coords, y_coords = P.get_coords()

    return np.sum(x_coords) / len(x_coords), np.sum(y_coords) / len(y_coords)

def distance(v1, v2):
    """ Computes the Euclidean distance between two numpy vectors v1 and v2 """
    return np.linalg.norm(v1 - v2)

def float_equality(actual_v, desired_v):
    return np.abs(actual_v - desired_v) < epsilon

def points_are_equal(P1, P2, epsilon=epsilon):
    """ Checks if two points P1 and P2 are approximately equal within a tolerance epsilon. """
    # Check if the distance between the points is smaller than epsilon
    return np.all(np.abs(P1 - P2) < epsilon)

def split_polygon_single(e2, intersection_p, cv):
    """ Split a polygon based on a single intersection point """

    # Creating the 1st sub-polygon
    v0 = Vertex(0, intersection_p[0, 0], intersection_p[1, 0])
    v0.edge_from_v_is_hard = e2.is_hard_edge

    P1_vertices = [v0]
    v_next = e2.v_to
    v_index = 1

    while v_next != cv:
        v = Vertex(v_index, v_next.x, v_next.y)
        v.edge_from_v_is_hard = v_next.edge_from_v_is_hard
        P1_vertices.append(v)
        v_index += 1
        v_next = v_next.next

    vn = Vertex(v_index, v_next.x, v_next.y)
    P1_vertices.append(vn)

    # Creating the 2nd sub-polygon
    v0 = Vertex(0, cv.x, cv.y)
    v0.edge_from_v_is_hard = cv.edge_from_v_is_hard
    P2_vertices = [v0]
    v_next = cv.next
    v_index = 1

    while v_next != e2.v_to:
        v = Vertex(v_index, v_next.x, v_next.y)
        v.edge_from_v_is_hard = v_next.edge_from_v_is_hard
        P2_vertices.append(v)
        v_index += 1
        v_next = v_next.next

    vn = Vertex(v_index, intersection_p[0, 0], intersection_p[1, 0])
    P2_vertices.append(vn)

    P1 = Polygon(P1_vertices)
    P2 = Polygon(P2_vertices)

    #plot_obstacles([P1], [], False)
    #plot_obstacles([P2], [], False)

    #return remove_collinear_vertices(P1), remove_collinear_vertices(P2)
    return P1, P2

def compute_intersection2(e, cv, e2):
    #print(f'{cv=}, {e}, {e2}')
    # (x, y) for the concave vertex
    x0 = cv.x
    y0 = cv.y

    # Coordinates for edge 1 translated to the concave vertex
    x1 = e.v_from.x
    y1 = e.v_from.y

    x2 = e.v_to.x
    y2 = e.v_to.y

    # Coordinates for edge 2
    x3 = e2.v_from.x
    y3 = e2.v_from.y

    x4 = e2.v_to.x
    y4 = e2.v_to.y

    if (y1 - y2)*x3 + (y2 - y1)*x4 + (x1 - x2)*(y4 - y3) == 0:
        return None, None

    if (y4 - y3)*x1 + (-y4 + y3)*x2 + (y1 - y2)*(x3 - x4) == 0:
        return None, None

    t = (((-y4 + y0)*x3 + (-y0 + y3)*x4 + x0*(y4 - y3))/
             ((y1 - y2)*x3 + (y2 - y1)*x4 + (x1 - x2)*(y4 - y3)))

    u =  ((y0 - y3)*x1 + (-y0 + y3)*x2 - (y1 - y2)*(x0 - x3))/((y4 - y3)*x1 + (-y4 + y3)*x2 + (y1 - y2)*(x3 - x4))

    ip = np.array([[x3], [y3]]) + u * np.array([[x4 - x3], [y4 - y3]])

    """# Ignore the intersection that happens in the adjacent vertices
    if points_are_equal(ip, cv.get_array()) or \
            points_are_equal(ip, cv.prev.get_array()) or \
            points_are_equal(ip, cv.next.get_array()):
        return None, None

    # Ignore perpendicular lines (intersection not possible)
    vec = e.v_to.get_array() - e.v_from.get_array()
    if points_are_equal(vec / np.linalg.norm(vec), (e2.v_to.get_array() - e2.v_from.get_array()) / np.linalg.norm(
            e2.v_to.get_array() - e2.v_from.get_array())) or \
            points_are_equal(-vec / np.linalg.norm(-vec),
                             (e2.v_to.get_array() - e2.v_from.get_array()) / np.linalg.norm(
                                 e2.v_to.get_array() - e2.v_from.get_array())):
        return None, None"""

    if e2.v_from.index == cv.prev.index and e2.v_to.index == cv.index:
        return None, None

    if e2.v_from.index == cv.index and e2.v_to.index == cv.next.index:
        return None, None
    #print(f'\t{t=}, {u=}')
    #print(f'\t{ip=}')

    if u == 0 or u ==1 and (e.prev == e2 or e.next == e2):
        return None, None

    if 0 <= u <= 1:
        # Returning the (x, y) coordinates of the intersection point
        #print(f'intersection cv={cv.index}, {e}, t={np.round(t, 1)}, {e2}, u={np.round(u, 1)}')
        return ip, t




    return None, None

def compute_intersection(vec, e, cv, e2):
    """ Computes the points of intersection (if any) between a vector from a point cv and an edge e2 """
    # Handle vertical or horizontal lines
    epsilon2 = 0#epsilon

    vx = vec[0, 0] + epsilon2
    vy = vec[1, 0] + epsilon2

    x0 = cv.x
    y0 = cv.y

    x1 = e2.v_from.x
    y1 = e2.v_from.y

    x2 = e2.v_to.x
    y2 = e2.v_to.y

    s = - ((y0 * vx - y1 * vx - vy * x0 + vy * x1) / (y1 * vx - y2 * vx - vy * x1 + vy * x2))
    t = - ((s * x1 - s * x2 + x0 - x1) / vx)

    # Test if the vector intersects the edge e2
    if 0 <= s <= 1:
        # Compute the coordinates of the intersection point
        intersection_point =  cv.get_array() + t * vec

        # Ignore the intersection that happens in the adjacent vertices
        if points_are_equal(intersection_point, cv.get_array()) or \
                points_are_equal(intersection_point, cv.prev.get_array()) or \
                points_are_equal(intersection_point, cv.next.get_array()):
            return None, None

        # Ignore perpendicular lines (intersection not possible)
        if points_are_equal(vec / np.linalg.norm(vec), (e2.v_to.get_array() - e2.v_from.get_array()) / np.linalg.norm(e2.v_to.get_array() - e2.v_from.get_array())) or \
                points_are_equal(-vec / np.linalg.norm(-vec), (e2.v_to.get_array() - e2.v_from.get_array()) / np.linalg.norm(e2.v_to.get_array() - e2.v_from.get_array())):
            return None, None

        return intersection_point, t
    return None, None

def find_min_value_matrix(D):
    """ Find the minimum value as well as its indices (i,j) in a 2D matrix """
    min_v = np.inf
    min_indices = (-1, -1)

    for i in range(D.shape[0]):
        for j in range(D.shape[1]):
            if D[i, j] < min_v:
                min_v = D[i, j]
                min_indices = (i, j)
    return min_v, min_indices

def remove_equal_points(P):
    vertices = []

    for i, v in enumerate(P.vertices):
        if i == 0:
            vertices.append(P.vertices[0])
            continue

        add_v = True

        for v2 in vertices:
            if points_are_equal(v.get_array(), v2.get_array()):
                add_v = False
                break

        if add_v:
            vertices.append(v)

    return Polygon(vertices)

def is_well_formed(polygon):
    polygon = remove_collinear_vertices(copy.deepcopy(polygon))

    # 1. Check if the polygon has at least 3 vertices
    if len(polygon.vertices) < 3:
        return False, "Polygon must have at least 3 vertices."

    # 2. Check if vertices are distinct
    for i, v in enumerate(polygon.vertices):
        for j, v2 in enumerate(polygon.vertices):
            if j <= i:
                continue

            if points_are_equal(v.get_array(), v2.get_array()):
                return False, "Polygon has duplicate vertices."

    # 3. Check if the polygon is in counterclockwise order
    if not is_ccw(polygon):
        return False, "Vertices are not in counterclockwise order."

    # 4. Check for edge intersections (except consecutive edges)
    if has_self_intersections(polygon):
        return False, "Has self intersections"

    return True, "Polygon is well-formed."

def is_ccw(polygon):
    # Use the signed area to determine if the vertices are in ccw order
    total = 0
    vertices = polygon.vertices
    for i in range(len(vertices)):
        v1 = vertices[i]
        v2 = vertices[(i + 1) % len(vertices)]
        total += (v2.x - v1.x) * (v2.y + v1.y)
    return total < 0

def has_self_intersections(polygon):
    edges = polygon.edges
    # Compare each edge with every other edge (ignoring consecutive edges)
    for i, e in enumerate(edges):
        for j, e2 in enumerate(edges):
            if i == j:
                break

            # Coordinates for edge 1
            x1 = e.v_from.x
            y1 = e.v_from.y

            x2 = e.v_to.x
            y2 = e.v_to.y

            # Coordinates for edge 2
            x3 = e2.v_from.x
            y3 = e2.v_from.y

            x4 = e2.v_to.x
            y4 = e2.v_to.y

            denominator = np.round(x1 * y3 - x1 * y4 - x2 * y3 + x2 * y4 - x3 * y1 + x3 * y2 + x4 * y1 - x4 * y2, 3)

            if denominator == 0:
                continue

            t = np.round((x1*y3 - x1*y4 - x3*y1 + x3*y4 + x4*y1 - x4*y3) / denominator, 3)

            u = np.round(-(x1*y2 - x1*y3 - x2*y1 + x2*y3 + x3*y1 - x3*y2) / denominator, 3)
            if 0 < t< 1 and 0 < u< 1:
                #print(f'{e} intersects {e2}')
                #print(f"SELF INTERSECTING t={t}, u={u}")
                return True

            if ((t == 0 or t == 1) and 0 < u < 1) or ((u == 0 or u == 1) and 0 < t < 1):
                #print(f"SELF INTERSECTING (EDGE CASE) {t=} {u=}")
                #print(f'{e} and {e2}')
                return True

    return False

def remove_collinear_vertices(P):
    """ Remove all collinear vertices in a polygon within some error """
    vertices = []

    for v in P.vertices:
        x1, y1 = v.prev.get_array().flatten()
        x2, y2 = v.get_array().flatten()
        x3, y3 = v.next.get_array().flatten()

        # Check the angle is not 180 deg
        vec1 = v.prev.get_array() - v.get_array()
        vec2 = v.next.get_array() - v.get_array()

        dot_product = dot(vec1, vec2)
        neg_mag = -np.linalg.norm(vec1) * np.linalg.norm(vec2)

        if not (dot_product - epsilon <= neg_mag <= dot_product + epsilon):
            new_v = Vertex(len(vertices), v.x, v.y)
            new_v.edge_from_v_is_hard = v.edge_from_v_is_hard
            vertices.append(new_v)

    return Polygon(vertices)

def cross(v1, v2):
    """ Cross product between two vectors """
    return np.cross(v1.flatten(), v2.flatten())

def split_polygon(P, depth=0):
    # Compute the concave vertices
    P.concave_vertices = compute_concave_vertices(P)
    ncc = len(P.concave_vertices)
    n = len(P.vertices)
    #save_polygon(P, depth, n)

    # Base case: if the polygon is convex, return it
    if ncc == 0:
        print(f'\treturning from {depth - 1}')
        return [P]
    print(f'{depth=}')
    #print(f'{n=}, {ncc=}')
    #print(f'concave vertices = {P.concave_vertices}')

    # Initialize the width sum matrix
    D = np.empty((ncc, n))
    D_polygons = []

    # Go through each concave vertex
    for i, cv in enumerate(P.concave_vertices):
        #print(f"checking for {cv.index=} with coord = ({cv.x}, {cv.y})")
        split_polygons = []

        # Check lines which passes the concave vertex i and parallels edge e
        for j, e in enumerate(P.edges):
            intersection_points = []
            intersection_edges = []
            intersection_directions = []
            intersection_normals = []
            intersection_legal = []

            #print(f'\tchecking edge {e}')

            # Define a vector from the vertices in edge e
            vec = e.v_to.get_array() - e.v_from.get_array()

            # Go through each edge in the P
            for e2 in P.edges:
                if e == e2:
                    continue
                #print(f'\t\twith {e2}')
                # Define the vector from the vertices in edge e2
                vec2 = e2.v_to.get_array() - e2.v_from.get_array()

                # Compute intersection with edge e2 (if any)
                #ip, t = compute_intersection(vec, e, cv, e2)
                ip, t = compute_intersection2(e, cv, e2)

                if ip is not None:
                    intersection_points.append(ip)
                    intersection_edges.append(e2)
                    intersection_directions.append(t)
                    intersection_normals.append(cross(vec, vec2))

                    if t * cross(vec, vec2) < 0:
                        intersection_legal.append(False)
                        #print(f'\t\t\tINVALID {t=}, normal={cross(vec,vec2)}, t*normal={t*cross(vec, vec2)}')
                    else:
                        intersection_legal.append(True)
            # Handle invalid intersections

            # Get the index of the intersection with minimum distance
            if len(intersection_directions) == 0:
                D[i, j] = np.inf
                split_polygons.append((None, None))
                continue

            min_index = np.argmin(np.abs(intersection_directions))
            intersection_points = np.array(intersection_points)
            intersection_edges = np.array(intersection_edges)
            intersection_directions = np.array(intersection_directions)
            intersection_normals = np.array(intersection_normals)

            # Check if the intersection is legal
            if intersection_normals[min_index] * intersection_directions[min_index] >= 0:
                P1, P2 = split_polygon_single(intersection_edges[min_index], intersection_points[min_index], cv)

                #print(f'\tselecting {intersection_points[min_index].flatten()}')
                # Remove collinear vertices form each sub-polygon
                #P1 = remove_collinear_vertices(P1)
                #P2 = remove_collinear_vertices(P2)

                if is_well_formed(P1)[0] and is_well_formed(P2)[0]:

                    min_width_P1 = min_polygon_width(P1.vertices_matrix())
                    min_width_P2 = min_polygon_width(P2.vertices_matrix())

                    D[i, j] = min_width_P1 + min_width_P2 #min_polygon_width(P1.vertices_matrix()) + min_polygon_width(P2.vertices_matrix())
                    split_polygons.append((P1, P2))
                    continue
            else:
                if intersection_directions[min_index] >= 0:
                    legal_mask = intersection_directions < 0
                else:
                    legal_mask = intersection_directions >= 0

                intersection_points_legal_dir = intersection_points[legal_mask]
                intersection_edges_legal_dir = intersection_edges[legal_mask]
                intersection_directions_legal = intersection_directions[legal_mask]
                intersection_normals_legal = intersection_normals[legal_mask]

                if len(intersection_directions_legal) == 0:
                    D[i, j] = np.inf
                    split_polygons.append((None, None))
                    continue
                min_index = np.argmin(np.abs(intersection_directions_legal))

                if intersection_normals_legal[min_index] * intersection_directions_legal[min_index] < 0:
                    P1, P2 = split_polygon_single(intersection_edges_legal_dir[min_index], intersection_points_legal_dir[min_index], cv)

                    # Remove collinear vertices form each sub-polygon
                    #P1 = remove_collinear_vertices(P1)
                    #P2 = remove_collinear_vertices(P2)

                    # Compute the width sum of P1 and P2
                    if is_well_formed(P1)[0] and is_well_formed(P2)[0]:

                        min_width_P1 = min_polygon_width(P1.vertices_matrix())

                        min_width_P2 = min_polygon_width(P2.vertices_matrix())

                        D[i, j] = min_width_P1 + min_width_P2
                        split_polygons.append((P1, P2))
                        continue

            D[i, j] = np.inf
            split_polygons.append((None, None))
            #print('did not append')

        #plot_results(split_polygons, depth, cv.index, D[i, :])
        D_polygons.append(split_polygons)

    # Select the best split of the polygon (lowest width sum)
    D_ij, (cv, edge) = find_min_value_matrix(D)

    P1, P2 = D_polygons[cv][edge]

    #plot_obstacles([P1], [], True)
    #plot_obstacles([P2], [], True)

    # Recursively split both sub-polygons if the polygons are valid
    result1 = []
    result2 = []

    result1 = split_polygon(P1, depth + 1)
    result2 = split_polygon(P2, depth + 1)

    # Combine both lists into one and return it
    return result1 + result2

def point_line_distance(point, line_point1, line_point2):
    """ Function to compute the perpendicular distance between a point and a line """
    numerator = np.abs((line_point2[1] - line_point1[1]) * point[0] -
                       (line_point2[0] - line_point1[0]) * point[1] +
                       line_point2[0] * line_point1[1] - line_point2[1] * line_point1[0])
    denominator = distance(line_point1, line_point2)
    return numerator / denominator

def min_polygon_width(vertices):
    """ Function to compute the minimum width using Rotating Calipers algorithm """
    # Compute the convex hull of the polygon
    hull = ConvexHull(vertices.T)  # Use the transpose of the vertices for the ConvexHull

    # Extract the vertices that make up the convex hull
    hull_points = vertices[:, hull.vertices]

    min_width = float('inf')

    n = hull_points.shape[1]

    # Apply rotating calipers to find the minimum distance between parallel lines
    for i in range(n):
        p1 = hull_points[:, i]
        p2 = hull_points[:, (i + 1) % n]

        # The edge we're rotating about is (p1, p2)
        max_distance = 0

        # For each other point, find its perpendicular distance to the line (p1, p2)
        for j in range(n):
            if j != i and j != (i + 1) % n:
                p = hull_points[:, j]
                d = point_line_distance(p, p1, p2)
                max_distance = max(max_distance, d)

        # The minimum width is the smallest of these maximum distances
        min_width = min(min_width, max_distance)

    return min_width

def dot(v1, v2):
    """ Dot product between two vertical vectors """
    return v1.flatten() @ v2.flatten()

def polygons_are_adjacent(P1, P2, i, j):
    """ Determine if two polygons share an edge either by complete or partial adjacency"""
    for e in P1.edges:
        for e2 in P2.edges:
            # Check for complete adjacency between the polygons
            if (points_are_equal(e.v_from.get_array(), e2.v_from.get_array()) and points_are_equal(e.v_to.get_array(), e2.v_to.get_array())) or \
                    (points_are_equal(e.v_from.get_array(), e2.v_to.get_array()) and points_are_equal(e.v_to.get_array(), e2.v_from.get_array())):
                print(f'complete overlap between {j} and {j}')
                return True

            # Define the vector for each edge
            vec1 = (e.v_to.get_array()) - e.v_from.get_array()
            vec2 = (e2.v_to.get_array()) - e2.v_from.get_array()

            dot_product = dot(vec1, vec2)
            neg_mag = -np.linalg.norm(vec1) * np.linalg.norm(vec2)

            # Check if the vectors are collinear
            if dot_product - epsilon_xl <= neg_mag <= dot_product + epsilon_xl:
                # Parametrize the line from vec1: l(t) = P + t v
                print(f'P{i} collinear with P{j}')

                # Check for partial adjacency between the polygons (if the projected intervals overlap)
                t1 = 0
                t2 = dot(e.v_to.get_array() - e.v_from.get_array(), vec1) / dot(vec1, vec1)
                s1 = dot(e2.v_from.get_array() - e.v_from.get_array(), vec1) / dot(vec1, vec1)
                s2 = dot(e2.v_to.get_array() - e.v_from.get_array(), vec1) / dot(vec1, vec1)

                # Check for partial adjacency between the polygons (if the projected intervals overlap)
                if max(t1, t2) > min(s1, s2) and max(s1, s2) > min(t1, t2):
                    print(f'\toverlap between P{i} and P{j} edges {e} and {e2}')

                    if not (vec1[0] == 0 and vec2[0] == 0):
                        # Compute the intersection with the y-axis for each line
                        t1 = - e.v_from.x / vec1[0]
                        e_intersects_y = e.v_from.y + t1 * vec1[1]

                        t2 = - e2.v_from.x / vec2[0]
                        e2_intersects_y = e2.v_from.y + t2 * vec2[1]

                        # Check if the two lines intersects y in the same point
                        if points_are_equal(e_intersects_y, e2_intersects_y, epsilon_xl):
                            print(f'\t\tshared edge between P{i} and P{j}')
                            return True
                    else:
                        # Rotation angle in radians
                        theta = np.radians(5)  # Example: Rotate 45 degrees

                        # Rotation matrix
                        R = np.array([
                            [np.cos(theta), -np.sin(theta)],
                            [np.sin(theta), np.cos(theta)]
                        ])

                        rotated_vec1 = np.dot(R, vec1)
                        rotated_vec2 = np.dot(R, vec2)

                        # Compute the intersection with the y-axis for each line
                        t1 = - e.v_from.x / rotated_vec1[0]
                        e_intersects_y = e.v_from.y + t1 * rotated_vec1[1]

                        t2 = - e2.v_from.x / rotated_vec2[0]
                        e2_intersects_y = e2.v_from.y + t2 * rotated_vec2[1]

                        if abs(abs(e_intersects_y - e2_intersects_y) - abs(e.v_from.y - e2.v_from.y)) < epsilon:
                            print(f'\t\tshared edge between P{i} and P{j}')
                            return True

    return False

# Theory used in report
def polygons_are_adjacent2(P1, P2, i, j):
    for i2, e in enumerate(P1.edges):
        vec1 = e.v_to.get_array() - e.v_from.get_array()

        for j2, e2 in enumerate(P2.edges):
            """if (points_are_equal(e.v_from.get_array(), e2.v_from.get_array(), epsilon_large) and points_are_equal(e.v_to.get_array(), e2.v_to.get_array()), epsilon_large) or \
                    (points_are_equal(e.v_from.get_array(), e2.v_to.get_array(), epsilon_large) and points_are_equal(e.v_to.get_array(), e2.v_from.get_array(), epsilon_large)):
                print(f'complete overlap between {i} and {j}')
                return True
"""
            vec2 = e2.v_to.get_array() - e2.v_from.get_array()

            #print(f'cross {i2} x {j2} = {np.abs(cross(vec1, vec2))}')
            # Test if the edges are collinear
            if np.abs(cross(vec1, vec2)) < epsilon:
                print(f'P{i} edge {i2} collinear with P{j} edge {j2}')

                # Projecting the endpoints onto the common line form between the edges
                proj_1 = dot(e.v_from.get_array(), vec1 ) / dot(vec1, vec1)
                proj_2 = dot(e.v_to.get_array(), vec1) / dot(vec1, vec1)
                proj_3 = dot(e2.v_from.get_array(), vec1) / dot(vec1, vec1)
                proj_4 = dot(e2.v_to.get_array(), vec1) / dot(vec1, vec1)

                if max(proj_1, proj_2) > min(proj_3, proj_4) and max(proj_3, proj_4) > min(proj_1, proj_2):

                    print(f'adjacent between P{i} and P{j}')
                    return True

    return False

def find_shared_edge(P1, P2):
    """ Return the shared edge for P1 and P2 if they have a complete adjacent edge """
    for e in P1.edges:
        for e2 in P2.edges:
            # Check if P_i and P_j share an edge completely
            if (points_are_equal(e.v_from.get_array(), e2.v_from.get_array()) and points_are_equal(
                    e.v_to.get_array(), e2.v_to.get_array())) or \
                    (points_are_equal(e.v_from.get_array(), e2.v_to.get_array()) and points_are_equal(
                        e.v_to.get_array(), e2.v_from.get_array())):

                return e, e2
    return None

def optimize_polygons(sub_polygons):
    """ Optimize the sub-polygons by combining them when possible (they share an edge that can be removed while keeping the polygon convex """
    merged = True

    while merged:
        merged = False

        # Go through each sub-polygon and compare with the other sub-polygons
        for i, P_i in enumerate(sub_polygons):
            for j, P_j in enumerate(sub_polygons):
                if j <= i:
                    continue

                shared_edges = find_shared_edge(P_i, P_j)

                if shared_edges is not None:
                    combined_polygon_vertices = []

                    #print(f'\t\tP_{i} and P_{j}')#, complete overlap between  {shared_edges[0]} and {shared_edges[1]}')
                    e, e2 = shared_edges

                    while e.next != shared_edges[0]:
                        combined_polygon_vertices.append(Vertex(len(combined_polygon_vertices), e.next.v_from.x, e.next.v_from.y))
                        e = e.next

                    while e2.next != shared_edges[1]:
                        combined_polygon_vertices.append(Vertex(len(combined_polygon_vertices), e2.next.v_from.x, e2.next.v_from.y))
                        e2 = e2.next

                    P = Polygon(combined_polygon_vertices)

                    if len(compute_concave_vertices(P)) == 0:
                        sub_polygons[i] = P
                        sub_polygons.pop(j)
                        merged = True
                    break
            if merged:
                break

    for i in range(len(sub_polygons)):
        sub_polygons[i].set_index(i)

    return sub_polygons

def plot_obstacles(sub_polygons, obstacles, include_points=True):
    fig, ax = plt.subplots(1, 1)
    count = 0
    for i, p in enumerate(sub_polygons):
        for e in p.edges:
            if e.is_hard_edge:
                ax.plot([e.v_from.x, e.v_to.x], [e.v_from.y, e.v_to.y], f'r-', marker='o' if include_points else None)
            else:
                ax.plot([e.v_from.x, e.v_to.x], [e.v_from.y, e.v_to.y], f'k-', marker='o' if include_points else None)

        if include_points:
            for v in p.vertices:
                plt.text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex
                count += 1
        c_x, c_y = get_center_of_polygon(p)
        ax.text(c_x - 0.1, c_y, f'P{i}', color='r', fontsize=7)

    for o in obstacles:
        x_coords, y_coords = o.get_coords()

        ax.plot(x_coords, y_coords, f'k-', marker='o' if include_points else None)
        ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], f'k-')

        if include_points:
            for v in o.vertices:
                plt.text(v.x, v.y, f'{count + v.index}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex

    ax.set_aspect('equal')

def sum_of_widths(sub_polygons):
    sum = 0

    for p in sub_polygons:
        sum += min_polygon_width(p.vertices_matrix())

    return sum