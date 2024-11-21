import numpy as np


def intermediate_path_distance(points, end_point):
    """ Calculate the total distance from a list of points to an end point.

    :param points: List of numpy arrays representing the path.
    :param end_point: Numpy array representing the endpoint to which the distance is calculated.
    :return: Float representing the total distance traveled through the list of points to the end point.
    """
    path_distance = 0.0

    for i in range(len(points) - 1):
        distance = np.linalg.norm(points[i + 1] - points[i])
        path_distance += distance
    path_distance += np.linalg.norm(points[-1] - end_point)

    return path_distance


def lines_intersect(p1, p2, p3, p4, eps=1e-6):
    """ Check if two line segments (p1 to p2 and p3 to p4) intersect.

    :param p1: Numpy array representing the first endpoint of the first line segment.
    :param p2: Numpy array representing the second endpoint of the first line segment.
    :param p3: Numpy array representing the first endpoint of the second line segment.
    :param p4: Numpy array representing the second endpoint of the second line segment.
    :param eps: Float tolerance for determining if points are collinear (default is 1e-6).
    :return: Bool indicating True if the line segments intersect, False otherwise.
    """

    def orientation(a, b, c):
        val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
        return 0 if abs(val) < eps else (1 if val > 0 else 2)

    def on_segment(p, q, r):
        return (min(p[0], r[0]) - eps <= q[0] <= max(p[0], r[0]) + eps and
                min(p[1], r[1]) - eps <= q[1] <= max(p[1], r[1]) + eps)

    o1, o2, o3, o4 = orientation(p1, p2, p3), orientation(p1, p2, p4), orientation(p3, p4, p1), orientation(p3, p4, p2)
    return (o1 != o2 and o3 != o4) or (o1 == 0 and on_segment(p1, p3, p2) or
                                       o2 == 0 and on_segment(p1, p4, p2) or
                                       o3 == 0 and on_segment(p3, p1, p4) or
                                       o4 == 0 and on_segment(p3, p2, p4))


def is_point_on_edge(point, edge, epsilon=1e-6):
    """ Check if a point lies on the line segment defined by edge_start and edge_end.

    :param point: Numpy array representing the point to check.
    :param edge: Tuple containing the starting point and the ending point of the edge.
    :param epsilon: Float tolerance for checking if the point is on the segment (default is 1e-6).
    :return: Bool indicating True if the point is on the edge, False otherwise.
    """
    edge_start, edge_end = edge

    vector_a = edge_end - edge_start
    vector_b = point - edge_start
    cross_product = np.abs(np.cross(vector_a, vector_b))

    if cross_product > epsilon:
        return False

    min_x, max_x = min(edge_start[0], edge_end[0]), max(edge_start[0], edge_end[0])
    min_y, max_y = min(edge_start[1], edge_end[1]), max(edge_start[1], edge_end[1])
    return (min_x - epsilon <= point[0] <= max_x + epsilon and
            min_y - epsilon <= point[1] <= max_y + epsilon)


def is_point_on_hard_edge(point, hard_edges):
    """ Check if a point is located on any of the hard edges.

    :param point: Numpy array representing the point to check.
    :param hard_edges: List of hard edges, where each hard edge is a tuple of (start_point, end_point).
    :return: Tuple containing a boolean indicating if the point is on a hard edge and the hard edge itself, or None if not found.
    """
    for hard_edge_start, hard_edge_end in hard_edges:
        if is_point_on_edge(point, (hard_edge_start, hard_edge_end)):
            return True, (hard_edge_start, hard_edge_end)
    return False, None


def find_closest_hard_edge_intersection(start_point, end_point, hard_edges, polygon, ignore_edges=None, ignore_points=None):
    """ Check if a line from start_point to end_point intersects with any hard edges,
    prioritizing edges that are part of the given polygon and ignoring specified edges.

    :param start_point: Numpy array representing the starting point of the line segment.
    :param end_point: Numpy array representing the ending point of the line segment.
    :param hard_edges: List of hard edges, where each hard edge is a tuple of (start_point, end_point).
    :param polygon: Polygon object in which to check for edges.
    :param ignore_edges: (Optional) List of edges to ignore, each represented as a tuple of (start_point, end_point).
    :param ignore_points: (Optional) List of points to ignore when checking for intersections.
    :return: Tuple or None representing the closest intersecting edge that is part of the polygon, or None if no intersection occurs.
    """

    closest_hard_edge = None
    min_distance = float('inf')

    # Convert ignore_edges to a set of tuples for faster lookup
    if ignore_edges is not None:
        ignore_edges_set = set((tuple(edge[0]), tuple(edge[1])) for edge in ignore_edges)

    # Create a set of edges in the polygon for quick lookup
    polygon_edges_set = {(tuple(edge.v_from.get_array().flatten()), tuple(edge.v_to.get_array().flatten())) for edge in polygon.edges}

    for hard_edge in hard_edges:
        # Check if hard_edge is a tuple or Edge object
        if isinstance(hard_edge, tuple):
            hard_edge_start = np.array(hard_edge[0])
            hard_edge_end = np.array(hard_edge[1])
        else:
            hard_edge_start = hard_edge.v_from.get_array().flatten()
            hard_edge_end = hard_edge.v_to.get_array().flatten()

        current_edge = (tuple(hard_edge_start), tuple(hard_edge_end))

        # Skip edges to ignore if specified
        if ignore_edges is not None and current_edge in ignore_edges_set:
            continue

        # Check if the start point is on the current hard edge and should be ignored
        if ignore_points is not None and any(np.array_equal(start_point, point) for point in ignore_points):
            if is_point_on_edge(start_point, (hard_edge_start, hard_edge_end)):
                continue  # Ignore this edge as the start point is on it

        # Check for intersection with the current hard edge
        if lines_intersect(start_point, end_point, hard_edge_start, hard_edge_end):
            # Check if the current edge is part of the polygon
            if current_edge in polygon_edges_set:
                return hard_edge_start, hard_edge_end  # Return immediately if this edge is part of the polygon

            # If the edge is not part of the polygon, calculate the distance
            midpoint = (hard_edge_start + hard_edge_end) / 2
            distance = np.linalg.norm(start_point - midpoint)
            if distance < min_distance:
                min_distance = distance
                closest_hard_edge = (hard_edge_start, hard_edge_end)

    # Returning the closest edge found or None if no intersection occurs
    return closest_hard_edge


def cross_product_2d(v1, v2):
    """ Calculate the cross product of two 2D vectors.

    :param v1: Numpy array representing the first vector.
    :param v2: Numpy array representing the second vector.
    :return: Float representing the cross product of the two vectors.
    """
    return v1[0] * v2[1] - v1[1] * v2[0]


def is_line_moving_into_polygon(start_point, end_point, hard_edge, polygon):
    """ Determine if the line from start_point to end_point is moving into the polygon defined by hard_edge.

    :param start_point: Numpy array representing the starting point of the line segment.
    :param end_point: Numpy array representing the ending point of the line segment.
    :param hard_edge: Tuple representing the hard edge as (start_point, end_point).
    :param polygon: Polygon object to check against.
    :return: Bool indicating True if the line is moving into the polygon, False otherwise.
    """
    hard_edge_start, hard_edge_end = hard_edge

    # Create 2D vectors from the points
    line_vector = (end_point[0] - start_point[0], end_point[1] - start_point[1])
    hard_edge_vector = (hard_edge_end[0] - hard_edge_start[0], hard_edge_end[1] - hard_edge_start[1])

    # Calculate centroid
    centroid = polygon.calculate_centroid()  # Ensure this returns a 2D tuple (x, y)

    # Create vector from hard edge start to centroid
    centroid_vector = (centroid[0] - hard_edge_start[0], centroid[1] - hard_edge_start[1])

    # Calculate the cross products
    cross_product_line = cross_product_2d(hard_edge_vector, line_vector)
    cross_product_centroid = cross_product_2d(hard_edge_vector, centroid_vector)

    # Determine the direction of movement
    return (cross_product_line > 0) and (cross_product_centroid > 0)


def find_adjacent_region_edge(edge, last_point, region):
    """ Find the adjacent edge in the region polygon that shares the last point
        and is not the same as the current hard edge.

    :param edge: Tuple representing the current hard edge as (start_point, end_point).
    :param last_point: Numpy array representing the last point in the path, which is a vertex in the current hard edge.
    :param region: Polygon object that contains all edges to check against.
    :return: Tuple or None representing the adjacent edge that shares the last point, or None if not found.
    """
    edge_start, edge_end = edge

    # Check if the last point matches either end of the current hard edge
    if np.array_equal(last_point, edge_end) or np.array_equal(last_point, edge_start):
        # Iterate over all edges in the region
        for edge in region.edges:
            edge_start = edge.v_from.get_array().flatten()
            edge_end = edge.v_to.get_array().flatten()

            # Check if this edge shares the last point and is not the same as the current hard edge
            if (np.array_equal(last_point, edge_start) or np.array_equal(last_point, edge_end)) and \
               not (np.array_equal(edge_start, edge_start) and np.array_equal(edge_end, edge_end)):
                return edge_start, edge_end  # Return the adjacent edge as a tuple

    return None  # Return None if no adjacent edge is found


def has_connected_region_hard_edge(point, hard_edges, region, ignore_edge=None):
    """ Check if there are any hard edges in the specified polygon (region) that share the given point,
    ignoring the specified edge if it's hard.

    :param point: Numpy array representing the point to check.
    :param hard_edges: List of hard edges, where each hard edge is a tuple of (start_point, end_point).
    :param region: The Polygon object containing the edges to check against.
    :param ignore_edge: A tuple representing an edge to ignore when checking for hard edges.
    :return: True if any edge that shares the point is a hard edge (excluding ignore_edge), False otherwise.
    """

    # Get the edges of the specified region
    x_coords, y_coords = region.get_coords()  # Get the coordinates of the polygon

    # Iterate through each edge of the region
    for i in range(len(x_coords)):
        edge_start = (x_coords[i], y_coords[i])
        edge_end = (x_coords[(i + 1) % len(x_coords)], y_coords[(i + 1) % len(x_coords)])  # Wrap around
        print(f'edge: {edge_start}, {edge_end}')
        # Check if the edge shares the given point
        if np.array_equal(point, edge_start) or np.array_equal(point, edge_end):

            if ignore_edge is not None and (np.array_equal(edge_start, ignore_edge[0]) and np.array_equal(edge_end, ignore_edge[1])):
                continue

            # Check if the current edge is a hard edge
            for hard_edge in hard_edges:
                if (np.array_equal(edge_start, hard_edge[0]) and np.array_equal(edge_end, hard_edge[1])) or \
                   (np.array_equal(edge_start, hard_edge[1]) and np.array_equal(edge_end, hard_edge[0])):
                    print(f"Found hard edge: {hard_edge} at point {point}")  # Debugging line
                    return True  # Found a hard edge at the point

    return False  # No hard edge found at the point


def find_adjacent_hard_edge(hard_edge, point, hard_edges, region):
    """ Find an adjacent hard edge that shares a vertex with the specified point.

    :param hard_edge: Tuple representing the hard edge as (start_point, end_point).
    :param point: Numpy array representing the point to check against the hard edges.
    :param hard_edges: List of hard edges.
    :param region: Polygon object containing the hard edges.
    :return: Tuple or None representing the adjacent hard edge that shares the specified point, or None if not found.
    """
    hard_edge_start, hard_edge_end = hard_edge

    for edge in hard_edges:
        # Check if the edge is not the same as the original hard edge
        if not (np.array_equal(edge[0], hard_edge_start) and np.array_equal(edge[1], hard_edge_end)):
            edge_start, edge_end = edge
            # Check if the given point matches either end of the adjacent edge
            if np.array_equal(point, edge_start) or np.array_equal(point, edge_end):
                return edge  # Return the adjacent hard edge

    # Edge case where the region is not fully enclosed in hard edges, so no hard edge is found
    edge = find_adjacent_region_edge(hard_edge, point, region)

    return edge


def find_optimal_path(left_path, right_path, end_point):
    """ Determine the optimal path between two paths (left and right) based on length and number of turns.

    :param left_path: List representing the first path as a list of points.
    :param right_path: List representing the second path as a list of points.
    :param end_point: Numpy array representing the endpoint to which the paths will be compared.
    :return: List representing the optimal path based on the criteria specified (shortest path).
    """
    # Choosing the shortest path (first by number of turns, then by distance)
    if len(left_path) == len(right_path):
        left_dist = intermediate_path_distance(left_path, end_point)
        right_dist = intermediate_path_distance(right_path, end_point)
        return left_path if left_dist < right_dist else right_path

    elif len(left_path) < len(right_path):
        return left_path
    else:
        return right_path


def find_polygons_of_hard_edge(hard_edge, polygons):
    """ Find all polygons that contain the given hard edge.

    :param hard_edge: Tuple representing the hard edge as (start_point, end_point).
    :param polygons: List of Polygon objects.
    :return: List of polygon objects that contain the hard edge.
    """
    matching_polygons = []

    for poly in polygons:
        x_coords, y_coords = poly.get_coords()  # Get the coordinates of the polygon

        # Iterate through each edge of the polygon
        for i in range(len(x_coords)):
            # Define the current edge
            edge_start = (x_coords[i], y_coords[i])
            edge_end = (x_coords[(i + 1) % len(x_coords)], y_coords[(i + 1) % len(x_coords)])  # Wrap around

            current_hard_edge = (edge_start, edge_end)

            # Check if the current edge matches the input hard edge in either order
            if (np.array_equal(current_hard_edge[0], hard_edge[0]) and np.array_equal(current_hard_edge[1], hard_edge[1])) or \
               (np.array_equal(current_hard_edge[0], hard_edge[1]) and np.array_equal(current_hard_edge[1], hard_edge[0])):
                matching_polygons.append(poly)  # Add the polygon to the list if found
                break  # No need to check other edges for this polygon

    return matching_polygons  # Return the list of matching polygons


def shorter_path(start_point, end_point, start_hard_edge, start_on_hard_edge, poly, polygons, region, hard_edges_list):
    """ Check if a shorter path exists from start_point to end_point while avoiding hard edges.

    :param start_point: Numpy array representing the starting point of the path.
    :param end_point: Numpy array representing the ending point of the path.
    :param start_hard_edge: Tuple representing the hard edge at the starting point.
    :param start_on_hard_edge: Bool indicating if the start point is on a hard edge.
    :param poly: Polygon object associated with the starting point.
    :param polygons: List of all polygons.
    :param region: Polygon object representing the region to check against.
    :param hard_edges_list: List of hard edges to avoid.
    :return: Bool indicating True if a shorter path can be found, False otherwise.
    """
    # Do a better check later
    if not start_on_hard_edge:
        return False

    # First check is to see if the line from start to end is moving inside the start hard edge's polygon
    if is_line_moving_into_polygon(start_point, end_point, start_hard_edge, poly):

        # Next we check for any intersecting edges
        intersecting_edge = find_closest_hard_edge_intersection(start_point, end_point, hard_edges_list, poly, [start_hard_edge])

        # If an edge is found to intersect new line, then check if this line is inside that polygon (if inside both polygons, it is assumed to be clear)
        if intersecting_edge:
            hard_edge_polys = find_polygons_of_hard_edge(intersecting_edge, polygons)

            # Check if a line can be drawn from start to end and be inside any of the poly's
            inside = False
            for current_poly in hard_edge_polys:
                if is_line_moving_into_polygon(end_point, start_point, intersecting_edge, current_poly):
                    inside = True

            if inside:
                next_hard_edge = find_adjacent_hard_edge(intersecting_edge, end_point, hard_edges_list, region)

                start_poly_check = find_closest_hard_edge_intersection(start_point, end_point, hard_edges_list, poly, [start_hard_edge, intersecting_edge, next_hard_edge])

                # If no intersecting edges found, then there is a clear path from the given start to the end point
                if not start_poly_check:
                    return True
    return False


def is_point_on_region_edge(point, region, eps=1e-6):
    """ Check if a given point lies on any of the edges of the specified polygon (region).

    :param point: Numpy array representing the point to check.
    :param region: Polygon object that contains the edges to check against.
    :param eps: Tolerance for determining if the point lies on an edge (default is 1e-6).
    :return: True if the point lies on any edge of the polygon, False otherwise.
    """
    # Get the edges of the specified region
    x_coords, y_coords = region.get_coords()  # Get the coordinates of the polygon

    # Iterate through each edge of the region
    for i in range(len(x_coords)):
        edge_start = np.array([x_coords[i], y_coords[i]])
        edge_end = np.array([x_coords[(i + 1) % len(x_coords)], y_coords[(i + 1) % len(x_coords)]])  # Wrap around

        # Check if the point lies on the edge using the is_point_on_edge logic
        if is_point_on_edge(point, (edge_start, edge_end), eps):
            return True  # The point lies on the edge

    return False  # The point does not lie on any edge of the polygon


def has_adjacent_hard_edge(point, hard_edge, hard_edges):
    """ Check if any adjacent edges to the specified hard edge are hard edges
    and return the adjacent hard edge that is connected to the given point.

    :param point: Numpy array representing the point that lies on the hard edge.
    :param hard_edge: A tuple representing the hard edge as (start_point, end_point).
    :param hard_edges: List of hard edges, where each hard edge is a tuple of (start_point, end_point).
    :return: Tuple (True, adjacent_hard_edge) if any adjacent edge is a hard edge, else (False, None).
    """
    edge_start, edge_end = hard_edge

    # Iterate through all hard edges to check for adjacency
    for edge in hard_edges:
        # Ignore the current hard edge itself
        if not (np.array_equal(edge[0], edge_start) and np.array_equal(edge[1], edge_end)) and \
           not (np.array_equal(edge[0], edge_end) and np.array_equal(edge[1], edge_start)):
            # Check if the edge shares the specified point
            if (np.array_equal(edge[0], point) or np.array_equal(edge[1], point)):
                return True, edge  # Return the adjacent hard edge connected to the point

    return False, None  # No adjacent hard edge found


def follow_and_create_path(temp_path, end_point, intersecting_edge, start_on_hard_edge, start_is_clear, curr_poly, prev_poly, polygons, region, hard_edges_list, direction):
    """ Follow and create a path around the hard edges, adjusting based on direction.

    :param temp_path: List representing the current path being constructed.
    :param end_point: Numpy array representing the endpoint to which the path is aimed.
    :param intersecting_edge: Tuple representing the hard edge currently being intersected.
    :param start_on_hard_edge: Bool indicating if the start point is on a hard edge.
    :param start_is_clear: Bool indicating if the path from the start point is clear.
    :param curr_poly: The current Polygon
    :param prev_poly: Polygon object representing the previous polygon before the current one.
    :param polygons: List of all polygons in the region.
    :param region: Polygon object representing the region to check against.
    :param hard_edges_list: List of hard edges to avoid.
    :param direction: String indicating the direction to follow ('right' or 'left').
    :return: List representing the updated path after avoiding hard edges.
    """
    current_hard_edge = intersecting_edge
    first_intersecting_edge = intersecting_edge
    counter = 0

    # Computing optimal free path
    while True:
        new_start_point = temp_path[-1]

        # If start point is on a hard edge and is clear, we ignore the starting edge (it is not crossed)
        if start_on_hard_edge and start_is_clear:
            start_on_hard_edge = False  # Only starts there once
            intersecting_edge = find_closest_hard_edge_intersection(new_start_point, end_point, hard_edges_list, prev_poly, [first_intersecting_edge], [new_start_point])

        else:
            # Checking if new start point is on the region edge
            if is_point_on_region_edge(new_start_point, region):
                # Checking if adjacent edge in direction is hard, if so, start point must be counted as intersection
                has_adjacent, adjacent_hard_edge = has_adjacent_hard_edge(new_start_point, current_hard_edge, hard_edges_list)

                if has_adjacent:
                    # Check if the line from new start to end is moving inside the end polygon
                    if is_line_moving_into_polygon(new_start_point, end_point, current_hard_edge, curr_poly):
                        intersecting_edge = find_closest_hard_edge_intersection(new_start_point, end_point, hard_edges_list, prev_poly, [current_hard_edge], [new_start_point, end_point])

                    else:
                        intersecting_edge = find_closest_hard_edge_intersection(new_start_point, end_point, hard_edges_list, prev_poly, [current_hard_edge], [new_start_point])

                # If no adjacent then the start point is not counted as an intersection
                else:
                    intersecting_edge = find_closest_hard_edge_intersection(new_start_point, end_point, hard_edges_list, prev_poly, [current_hard_edge], [new_start_point])

            else:
                intersecting_edge = find_closest_hard_edge_intersection(new_start_point, end_point, hard_edges_list, prev_poly, [current_hard_edge], [new_start_point])

        # Check if a free path is found then stop searching
        if not intersecting_edge:
            break

        # Incase a path is never found, should not happen
        if counter > 50:
            #print(f"No path found when going {direction}")
            break

        # Finding the next hard edge to follow around the obstacle, using prev point to get edge, so always in correct direction
        current_hard_edge = find_adjacent_hard_edge(current_hard_edge, new_start_point, hard_edges_list, region)
        if not current_hard_edge:
            current_hard_edge = intersecting_edge

        # Which next point to append depends on direction
        if direction == 'right':
            if shorter_path(temp_path[0], current_hard_edge[0], first_intersecting_edge, start_on_hard_edge, prev_poly, polygons, region, hard_edges_list):
                temp_path = [temp_path[0], current_hard_edge[0]]

            else:
                temp_path.append(current_hard_edge[0])
        else:
            if shorter_path(temp_path[0], current_hard_edge[1], first_intersecting_edge, start_on_hard_edge, prev_poly, polygons, region, hard_edges_list):
                temp_path = [temp_path[0], current_hard_edge[1]]

            else:
                temp_path.append(current_hard_edge[1])
        counter += 1

    return temp_path


def avoid_hard_edges(start_point, end_point, current_polygon, polygons, region, hard_edges_list):
    """ Determine a path around hard edges from start_point to end_point within the current polygon.

    :param start_point: Numpy array representing the starting point of the path.
    :param end_point: Numpy array representing the ending point of the path.
    :param current_polygon: Polygon object containing the starting point.
    :param polygons: List of all polygons in the region.
    :param region: Polygon object representing the region to check against.
    :param hard_edges_list: List of hard edges to avoid.
    :return: List representing a series of intermediate points forming a clear path around hard edges.
    """

    previous_polygon = polygons[polygons.index(current_polygon) - 1]

    # Check if the start point is on a hard edge
    start_on_hard_edge, start_hard_edge = is_point_on_hard_edge(start_point, hard_edges_list)

    start_is_clear = False

    # Start point is on a hard edge
    if start_on_hard_edge:
        print("Starting on hard edge")

        # Check if the line from the start point to the end point is moving into polygon and not crossing the hard edge
        if is_line_moving_into_polygon(start_point, end_point, start_hard_edge, previous_polygon):
            start_is_clear = True
            print("Start is clear")

            # Check for intersections with hard edges, ignoring the edge where the start point lies
            intersecting_edge = find_closest_hard_edge_intersection(start_point, end_point, hard_edges_list, previous_polygon, [start_hard_edge])
            if not intersecting_edge:
                return []
            else:
                # Check if end point is on a hard edge
                on_hard_edge, end_hard_edge = is_point_on_hard_edge(end_point, hard_edges_list)

                if on_hard_edge:
                    # Checking if line is moving inside either of the polygons
                    in_first = is_line_moving_into_polygon(start_point, end_point, intersecting_edge, previous_polygon)
                    in_last = is_line_moving_into_polygon(end_point, start_point, intersecting_edge, current_polygon)

                    if in_first or in_last:
                        return []

         # Line from start to end is moving into obstacle
        else:
            print("Start not clear")
            # The intersecting edge is the start edge, as it get crossed
            intersecting_edge = start_hard_edge
            print(start_hard_edge)

    # Not starting on a hard edge
    else:
        #print("Not starting on hard edge")
        intersecting_edge = find_closest_hard_edge_intersection(start_point, end_point, hard_edges_list, previous_polygon)

        if intersecting_edge:
            # Check if end point is on a hard edge
            on_hard_edge, end_hard_edge = is_point_on_hard_edge(end_point, hard_edges_list)

            if on_hard_edge:
                # Checking if line is moving inside either of the polygons
                in_first = is_line_moving_into_polygon(start_point, end_point, intersecting_edge, previous_polygon)
                in_last = is_line_moving_into_polygon(end_point, start_point, intersecting_edge, current_polygon)

                if in_first or in_last:
                    print("Moving inside, Clear path")
                    return []

        else:
            print("Clear path")
            return []

    # Computing path around hard edges in both directions
    right_temp_path = follow_and_create_path([start_point, intersecting_edge[0]], end_point, intersecting_edge,
                                             start_on_hard_edge, start_is_clear, current_polygon, previous_polygon, polygons, region, hard_edges_list,
                                             direction='right')
    left_temp_path = follow_and_create_path([start_point, intersecting_edge[1]], end_point, intersecting_edge,
                                            start_on_hard_edge, start_is_clear, current_polygon, previous_polygon, polygons, region, hard_edges_list,
                                            direction='left')
    intermediate_points = find_optimal_path(left_temp_path, right_temp_path, end_point)

    return intermediate_points