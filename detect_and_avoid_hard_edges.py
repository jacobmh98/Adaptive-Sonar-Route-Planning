from turtledemo.penrose import start

import numpy as np
import Polygon  # Ensure this module is correctly defined with 3D centroid support

def calculate_total_distance(points, end_point):
    total_distance = 0.0

    for i in range(len(points) - 1):
        distance = np.linalg.norm(points[i + 1] - points[i])
        total_distance += distance
    total_distance += np.linalg.norm(points[-1] - end_point)

    return total_distance


def compute_normal_vector(hard_edge_start, hard_edge_end):
    edge_vector = hard_edge_end - hard_edge_start
    normal_vector = np.array([-edge_vector[1], edge_vector[0], 0.0], dtype=float)
    normal_vector /= np.linalg.norm(normal_vector)
    return normal_vector


def lines_intersect(p1, p2, p3, p4, epsilon=1e-6):
    def orientation(a, b, c):
        val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
        return 0 if abs(val) < epsilon else (1 if val > 0 else 2)

    def on_segment(p, q, r, epsilon=1e-6):
        return (min(p[0], r[0]) - epsilon <= q[0] <= max(p[0], r[0]) + epsilon and
                min(p[1], r[1]) - epsilon <= q[1] <= max(p[1], r[1]) + epsilon)

    o1, o2, o3, o4 = orientation(p1, p2, p3), orientation(p1, p2, p4), orientation(p3, p4, p1), orientation(p3, p4, p2)
    return (o1 != o2 and o3 != o4) or (o1 == 0 and on_segment(p1, p3, p2, epsilon) or
                                       o2 == 0 and on_segment(p1, p4, p2, epsilon) or
                                       o3 == 0 and on_segment(p3, p1, p4, epsilon) or
                                       o4 == 0 and on_segment(p3, p2, p4, epsilon))


def is_point_on_segment(point, seg_start, seg_end, epsilon=1e-6):
    vector_a = seg_end - seg_start
    vector_b = point - seg_start
    cross_product = np.abs(np.cross(vector_a, vector_b))

    if cross_product > epsilon:
        return False

    min_x, max_x = min(seg_start[0], seg_end[0]), max(seg_start[0], seg_end[0])
    min_y, max_y = min(seg_start[1], seg_end[1]), max(seg_start[1], seg_end[1])
    return (min_x - epsilon <= point[0] <= max_x + epsilon and
            min_y - epsilon <= point[1] <= max_y + epsilon)


def check_line_intersects_hard_edges(start_point, end_point, hard_edges, ignore_edges=None):
    """
    Check if a line from start_point to end_point intersects with any hard edges,
    ignoring specified edges.

    Parameters:
    start_point (numpy array): The starting point of the line segment.
    end_point (numpy array): The ending point of the line segment.
    hard_edges (list): List of hard edges, where each hard edge is a tuple of (start_point, end_point).
    ignore_edges (list, optional): A list of edges to ignore, each represented as a tuple of (start_point, end_point).

    Returns:
    tuple or None: The closest intersecting edge, or None if no intersection occurs.
    """
    closest_edge = None
    min_distance = float('inf')

    # Convert ignore_edges to a set of tuples for faster lookup
    if ignore_edges is not None:
        ignore_edges_set = set((tuple(edge[0]), tuple(edge[1])) for edge in ignore_edges)

    for hard_edge_start, hard_edge_end in hard_edges:
        current_edge = (tuple(hard_edge_start), tuple(hard_edge_end))

        # Skip edges to ignore if specified
        if ignore_edges is not None and current_edge in ignore_edges_set:
            continue

        # Check for intersection with the current hard edge
        if lines_intersect(start_point, end_point, hard_edge_start, hard_edge_end):
            midpoint = (hard_edge_start + hard_edge_end) / 2
            distance = np.linalg.norm(start_point - midpoint)
            if distance < min_distance:
                min_distance = distance
                closest_edge = (hard_edge_start, hard_edge_end)

    return closest_edge


def check_line_intersects_excluding_start_point_edges(start_point, end_point, hard_edges, current_hard_edge, prev_poly):
    closest_edge = None
    min_distance = float('inf')
    for hard_edge_start, hard_edge_end in hard_edges:
        # Ignore this edge if it contains the start point
        if is_point_on_segment(start_point, hard_edge_start, hard_edge_end):
            #if is_line_moving_into_polygon(start_point, end_point, current_hard_edge, prev_poly):
            #    print("here")
            continue

        # Check if the line between start_point and end_point intersects with the hard edge
        if lines_intersect(start_point, end_point, hard_edge_start, hard_edge_end):
            midpoint = (hard_edge_start + hard_edge_end) / 2
            distance = np.linalg.norm(start_point - midpoint)
            if distance < min_distance:
                min_distance = distance
                closest_edge = (hard_edge_start, hard_edge_end)

    return closest_edge


def is_point_on_hard_edge(point, hard_edges):
    for hard_edge_start, hard_edge_end in hard_edges:
        if is_point_on_segment(point, hard_edge_start, hard_edge_end):
            return True, (hard_edge_start, hard_edge_end)  # Return the hard edge as well
    return False, None


def calculate_cross_product_2d(v1, v2):
    """Calculate the cross product of two 2D vectors."""
    return v1[0] * v2[1] - v1[1] * v2[0]


def is_line_moving_into_polygon(start_point, end_point, hard_edge, polygon):
    hard_edge_start, hard_edge_end = hard_edge

    # Create 2D vectors from the points
    line_vector = (end_point[0] - start_point[0], end_point[1] - start_point[1])
    hard_edge_vector = (hard_edge_end[0] - hard_edge_start[0], hard_edge_end[1] - hard_edge_start[1])

    # Calculate centroid
    centroid = polygon.calculate_centroid()  # Ensure this returns a 2D tuple (x, y)
    print(f'Centroid: {centroid}')
    # Create vector from hard edge start to centroid
    centroid_vector = (centroid[0] - hard_edge_start[0], centroid[1] - hard_edge_start[1])

    # Calculate the cross products
    cross_product_line = calculate_cross_product_2d(hard_edge_vector, line_vector)
    cross_product_centroid = calculate_cross_product_2d(hard_edge_vector, centroid_vector)

    # Determine the direction of movement
    return (cross_product_line > 0) and (cross_product_centroid > 0)


def find_adjacent_edge(current_hard_edge, last_point, region):
    """
    Find the adjacent edge in the region polygon that shares the last point
    and is not the same as the current hard edge.

    Parameters:
    current_hard_edge (tuple): The current hard edge as a tuple of (start_point, end_point).
    last_point (numpy array): The last point in the path, which is a vertex in the current hard edge.
    region (Polygon): The polygon object that contains all edges to check against.

    Returns:
    tuple or None: The adjacent edge that shares the last point, or None if not found.
    """
    current_edge_start, current_edge_end = current_hard_edge

    # Check if the last point matches either end of the current hard edge
    if np.array_equal(last_point, current_edge_end) or np.array_equal(last_point, current_edge_start):
        # Iterate over all edges in the region
        for edge in region.edges:
            edge_start = edge.v_from.get_array().flatten()
            edge_end = edge.v_to.get_array().flatten()

            # Check if this edge shares the last point and is not the same as the current hard edge
            if (np.array_equal(last_point, edge_start) or np.array_equal(last_point, edge_end)) and \
               not (np.array_equal(edge_start, current_edge_start) and np.array_equal(edge_end, current_edge_end)):
                return (edge_start, edge_end)  # Return the adjacent edge as a tuple

    return None  # Return None if no adjacent edge is found


def find_adjacent_hard_edge(hard_edge, point, hard_edges, region):
    hard_edge_start, hard_edge_end = hard_edge

    for edge in hard_edges:
        # Check if the edge is not the same as the original hard edge
        if not (np.array_equal(edge[0], hard_edge_start) and np.array_equal(edge[1], hard_edge_end)):
            edge_start, edge_end = edge
            # Check if the given point matches either end of the adjacent edge
            if np.array_equal(point, edge_start) or np.array_equal(point, edge_end):
                return edge  # Return the adjacent hard edge

    # Edge case where the region is not fully enclosed in hard edges, so no hard edge is found
    edge = find_adjacent_edge(hard_edge, point, region)

    return edge


def find_optimal_path(left_path, right_path, end_point):
    # Choosing the shortest path (first by number of turns, then by distance)
    if len(left_path) == len(right_path):
        left_dist = calculate_total_distance(left_path, end_point)
        right_dist = calculate_total_distance(right_path, end_point)
        return left_path if left_dist < right_dist else right_path

    elif len(left_path) < len(right_path):
        return left_path
    else:
        return right_path


def find_polygons_of_hard_edge(hard_edge, polygons):
    """
    Find all polygons that contain the given hard edge.

    Parameters:
    hard_edge (tuple): A tuple representing the hard edge as (start_point, end_point).
    polygons (list): A list of Polygon objects.

    Returns:
    list: A list of polygon objects that contain the hard edge.
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
    # Do a better check later
    if not start_on_hard_edge:
        return False

    # First check is to see if the line from start to end is moving inside the start hard edge's polygon
    if is_line_moving_into_polygon(start_point, end_point, start_hard_edge, poly):

        # Next we check for any intersecting edges
        intersecting_edge = check_line_intersects_hard_edges(start_point, end_point, hard_edges_list, [start_hard_edge])

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

                start_poly_check = check_line_intersects_hard_edges(start_point, end_point, hard_edges_list, [start_hard_edge, intersecting_edge, next_hard_edge])

                # If no intersecting edges found, then there is a clear path from the given start to the end point
                if not start_poly_check:
                    return True

    return False

def follow_and_create_path(temp_path, end_point, start_hard_edge, start_on_hard_edge, prev_poly, polygons, region, hard_edges_list, direction):
    print(direction)
    intersecting_edge = start_hard_edge
    current_hard_edge = start_hard_edge
    start_point = temp_path[0]
    print(f"starting point: {temp_path[0]}")
    print(f"Ending point: {end_point}")
    print(f"Start hard edge: {start_hard_edge}")

    if is_line_moving_into_polygon(start_point, end_point, start_hard_edge, prev_poly):

        print("Moving into clear")
        #intersecting_edge = check_line_intersects_hard_edges(start_point, end_point, hard_edges_list, [start_hard_edge])
    else:
        print("moving into unclear")

    while True:
        # Checking for a free path
        intersecting_edge = check_line_intersects_excluding_start_point_edges(temp_path[-1], end_point,
                                                                              hard_edges_list, current_hard_edge,
                                                                              prev_poly)

        print(f'Next edge: {intersecting_edge}')
        # Check if a free path is found then stop searching
        if not intersecting_edge:
            break

        # Finding the next hard edge to follow around the obstacle, using prev point to get edge, so always in correct direction
        current_hard_edge = find_adjacent_hard_edge(current_hard_edge, temp_path[-1], hard_edges_list, region)

        # Which next point to append depends on direction
        if direction == 'right':
            if shorter_path(temp_path[0], current_hard_edge[0], start_hard_edge, start_on_hard_edge, prev_poly, polygons, region, hard_edges_list):
                temp_path = [temp_path[0], current_hard_edge[0]]
            else:
                temp_path.append(current_hard_edge[0])
        else:
            if shorter_path(temp_path[0], current_hard_edge[1], start_hard_edge, start_on_hard_edge, prev_poly, polygons, region, hard_edges_list):
                temp_path = [temp_path[0], current_hard_edge[1]]
            else:
                temp_path.append(current_hard_edge[1])


    return temp_path


def avoid_hard_edges(start_point, end_point, current_polygon, polygons, region, hard_edges_list):
    intermediate_points = []
    current_poly_index = polygons.index(current_polygon)
    prev_poly = polygons[current_poly_index - 1]

    # Check if the start point is on a hard edge
    start_on_hard_edge, start_hard_edge = is_point_on_hard_edge(start_point, hard_edges_list)

    # Start point is on a hard edge
    if start_on_hard_edge:

        # Check if the line from the start point to the end point is moving into polygon and not crossing the hard edge
        if is_line_moving_into_polygon(start_point, end_point, start_hard_edge, prev_poly):
            print("Going into clear area")

         # Line from start to end is moving
        else:
            print("Going into unclear area")

    # Not starting on a hard edge
    else:
        print("Point not on hard edge")


    return intermediate_points

"""
            # Check for intersections with hard edges, ignoring the edge where the start point lies
            intersecting_edge = check_line_intersects_hard_edges(start_point, end_point, hard_edges_list,
                                                                 [start_hard_edge])
            if intersecting_edge:
                right_temp_path = follow_and_create_path([start_point, intersecting_edge[0]], end_point, start_hard_edge, start_on_hard_edge, prev_poly, polygons, region, hard_edges_list, direction='right')
                left_temp_path = follow_and_create_path([start_point, intersecting_edge[1]], end_point, start_hard_edge, start_on_hard_edge, prev_poly, polygons, region, hard_edges_list,direction='left')
                intermediate_points = find_optimal_path(left_temp_path, right_temp_path, end_point)

        else:
            print("Going into unclear area")
            intersecting_edge = check_line_intersects_hard_edges(start_point, end_point, hard_edges_list)

            if intersecting_edge:
                right_temp_path = follow_and_create_path([start_point, intersecting_edge[0]], end_point, start_hard_edge, start_on_hard_edge, prev_poly, polygons, region, hard_edges_list, direction='right')
                left_temp_path = follow_and_create_path([start_point, intersecting_edge[1]], end_point, start_hard_edge, start_on_hard_edge, prev_poly, polygons, region, hard_edges_list,direction='left')
                intermediate_points = find_optimal_path(left_temp_path, right_temp_path, end_point)

    # Starting point not on hard edge
    else:
        print("Point not on hard edge")
        intersecting_edge = check_line_intersects_hard_edges(start_point, end_point, hard_edges_list)
        print(f'intersecting edge: {intersecting_edge}')
        # if an edge is found it is not a clear path
        if intersecting_edge:
            right_temp_path = follow_and_create_path([start_point, intersecting_edge[0]], end_point, intersecting_edge, start_on_hard_edge, prev_poly, polygons, region,
                                                     hard_edges_list, direction='right')
            left_temp_path = follow_and_create_path([start_point, intersecting_edge[1]], end_point, intersecting_edge, start_on_hard_edge, prev_poly, polygons, region,
                                                    hard_edges_list, direction='left')
            intermediate_points = find_optimal_path(left_temp_path, right_temp_path, end_point)

    return intermediate_points
"""