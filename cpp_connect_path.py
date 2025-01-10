import numpy as np
import reroute_main

def compute_total_distance(path):
    total_distance = 0.0
    # Loop through each consecutive pair of points and compute the distance
    for i in range(len(path) - 1):
        total_distance += np.linalg.norm(path[i + 1] - path[i])
    return total_distance


def find_nearest_vertex_to_point(poly, point):
    nearest_vertex = None
    min_distance = float('inf')

    # Loop over all vertices in the polygon
    for vertex in poly.vertices:
        dist = np.linalg.norm(np.array(vertex.v) - np.array(point))

        if dist < min_distance:
            min_distance = dist
            nearest_vertex = vertex

    return nearest_vertex


def find_nearest_to_vertex(poly, first_intersection, last_intersection):
    # Combine the two points from both intersections into a single list
    intersection_points = [first_intersection[0], first_intersection[1], last_intersection[0], last_intersection[1]]

    nearest_intersection_point = None
    min_distance = float('inf')
    nearest_intersection = None

    # Loop over all vertices in the polygon
    for vertex in poly.vertices:
        for i, intersection_point in enumerate(intersection_points):
            dist = np.linalg.norm(np.array(vertex.v) - intersection_point)

            if dist < min_distance:
                nearest_vertex = vertex
                min_distance = dist
                nearest_intersection_point = intersection_point
                nearest_intersection = 0 if i < 2 else 1  # 0 for first intersection, 1 for last
    return nearest_intersection_point, nearest_intersection


def find_nearest_to_point(point, first_intersection, last_intersection):
    # Combine the two points from both intersections into a single list
    intersection_points = [first_intersection[0], first_intersection[1], last_intersection[0], last_intersection[1]]

    nearest_intersection_point = None
    min_distance = float('inf')
    nearest_intersection = None

    # Loop over all intersection points
    for i, intersection_point in enumerate(intersection_points):
        dist = np.linalg.norm(np.array(point) - intersection_point)

        if dist < min_distance:
            min_distance = dist
            nearest_intersection_point = intersection_point
            nearest_intersection = 0 if i < 2 else 1  # 0 for first intersection, 1 for last

    return nearest_intersection_point, nearest_intersection


def add_intersection_points_to_path(path, intersection):
    last_point = path[-1]  # The last point in the current path

    # Unpack the intersection
    p1, p2 = intersection

    # Calculate the distance between the last point and each point in the intersection
    dist_to_p1 = np.linalg.norm(last_point - p1)
    dist_to_p2 = np.linalg.norm(last_point - p2)

    # Add the points in the correct order based on their distance to the last point
    if dist_to_p1 <= dist_to_p2:
        path.append(p1)
        path.append(p2)
    else:
        path.append(p2)
        path.append(p1)

    return path


def compute_path(nearest_intersection_point, nearest_intersection_index,intersections):
    path = [nearest_intersection_point]  # Initialize path with the nearest intersection point

    # Getting index of nearest intersection in intersection list (can only be first or last element)
    if nearest_intersection_index != 0:
        nearest_intersection_index = len(intersections) - 1

    # Add the second point from the nearest intersection
    if np.array_equal(np.array(intersections[nearest_intersection_index][0]), np.array(nearest_intersection_point)):
        path.append(intersections[nearest_intersection_index][1])
    else:
        path.append(intersections[nearest_intersection_index][0])

    # Go through the intersection list in normal order (first to last)
    if nearest_intersection_index == 0:
        for intersection in intersections[1:]:  # Iterate through remaining intersections and add to the first path
            path = add_intersection_points_to_path(path, intersection)

    else:  # Iterate through the intersection list backwards
        for intersection in reversed(intersections[:-1]):
            path = add_intersection_points_to_path(path, intersection)

    return path


def connect_first_path(next_poly, intersections):
    # Find nearest vertex in next_poly to either first or last intersection
    nearest_intersection_point, nearest_intersection_index = find_nearest_to_vertex(next_poly, intersections[0], intersections[-1])

    # Computing the path
    path = compute_path(nearest_intersection_point, nearest_intersection_index,intersections)

    return path[::-1] # For first path, the start point is the nearest point in next polygon, so must be reversed


def connect_middle_path(polygons, total_intersections, i, path):
    last_path_point = path[-1]
    intersections = total_intersections[i]
    nearest_intersection_point, nearest_intersection_index = find_nearest_to_point(last_path_point, intersections[0], intersections[-1])

    # Computing the path
    path = compute_path(nearest_intersection_point, nearest_intersection_index, intersections)

    # Check for better path in opposite direction
    opposite_path = connect_first_path(polygons[i+1], intersections)

    # Finding the path's start and end points
    start_point = last_path_point  # Must always start at last path's end
    end_point1 = find_nearest_vertex_to_point(polygons[i+1], path[-1]).v.flatten()
    end_point2 = find_nearest_vertex_to_point(polygons[i+1], opposite_path[-1]).v.flatten()

    # Appending start and end points to both paths and calculating the total distances
    check_middle = [start_point] + path + [end_point1]
    check_opposite = [start_point] + opposite_path + [end_point2]

    # Checking total distances travelled in each path
    dist1 = compute_total_distance(check_middle)
    dist2 = compute_total_distance(check_opposite)

    if dist1 <= dist2:
        return path
    else:
        return opposite_path


def connect_last_path(path, intersections):
    last_path_point = path[-1]
    nearest_intersection_point, nearest_intersection_index = find_nearest_to_point(last_path_point, intersections[0], intersections[-1])

    # Computing the path
    path = compute_path(nearest_intersection_point, nearest_intersection_index, intersections)

    return path


def connect_solo_path(intersections):  # Any point can be used as first point for the optimal path in solo paths
    solo_path = [intersections[0][0],intersections[0][1]] # Manually adding first intersection
    for intersection in intersections[1:]:  # Iterate through remaining intersections and add to the first path
        solo_path = add_intersection_points_to_path(solo_path, intersection)

    return solo_path


def extract_hard_edges(polygons):
    # Extracting indices of hard edge vertices in each polygon
    hard_vertice_index_list = []
    for poly in polygons:
        sub_list = []
        for vertex in poly.vertices:
            if vertex.edge_from_v_is_hard:
                sub_list.append(vertex.index)
        hard_vertice_index_list.append(sub_list)

    # Extracting all hard edges
    all_hard_edges = []
    for poly_index, poly in enumerate(polygons):
        if poly_index < len(hard_vertice_index_list):
            edges = hard_vertice_index_list[poly_index]
            x_coords, y_coords = poly.get_coords()
            for edge_index in edges:
                start_idx = edge_index
                end_idx = (edge_index + 1) % len(x_coords)

                hard_edge_start = np.array([x_coords[start_idx], y_coords[start_idx]])
                hard_edge_end = np.array([x_coords[end_idx], y_coords[end_idx]])

                all_hard_edges.append((hard_edge_start, hard_edge_end))

    return all_hard_edges


def connect_path(polygons, total_intersections, region, obstacles):
    path = np.empty((0, 2))  # Main path
    transit_flags = []  # List to store flags for each point in the transit path
    transit_path = np.empty((0,2))
    hard_region_edges = extract_hard_edges(polygons)
    hard_obstacles = [obstacle for obstacle in obstacles if obstacle.is_hard_obstacle]
    added_extra_points = 0

    for i, poly in enumerate(polygons):
        current_path = []

        # Determine the current path based on the polygon index
        if len(polygons) == 1:
            # Single polygon case
            current_path = connect_solo_path(total_intersections[i])
        elif i == 0:
            # First polygon case
            current_path = connect_first_path(polygons[i + 1], total_intersections[i])
        elif i < len(polygons) - 1:
            # Middle polygons
            current_path = connect_middle_path(polygons, total_intersections, i, path)
        elif i == len(polygons) - 1:
            # Last polygon case
            current_path = connect_last_path(path, total_intersections[i])

        # Handle intermediate points between polygons
        reroute = True
        if i > 0 and (hard_obstacles or hard_region_edges) and reroute:  # Not needed if no hard obstacles or region edges present
            print(f"\nGoing from {i - 1} to {i}")
            last_path_point = path[-1]  # Last point of the current path
            current_first_point = current_path[0]  # First point of the next path

            # Compute intermediate points to avoid obstacles and region hard edges
            intermediate_points = reroute_main.hard_edges_rerouting(
                last_path_point, current_first_point, region, hard_obstacles, i, polygons)

            # Append intermediate points and mark as transit
            for point in intermediate_points:
                path = np.vstack([path, point])
                transit_flags.append("transit")

        # Append the current path to the main path
        if current_path:

            # Add a new point between the two existing points for paths with just 2 points
            if len(current_path) == 2:
                # Compute the midpoint between the two points
                #midpoint = (current_path[0] + current_path[1]) / 2.0

                # Insert the midpoint into the array at position 1
                current_path = np.insert(current_path, 1, current_path[0], axis=0)
                added_extra_points += 1

            # Add flags for the current path
            for idx, point in enumerate(current_path):
                if idx == 0 or idx == len(current_path) - 1:  # First and last path points are marked as transit points
                    transit_flags.append("transit")
                else:
                    transit_flags.append(None)

            path = np.vstack([path, current_path])

    return path, transit_flags, added_extra_points


def remove_duplicate_points_preserve_order(path, flags):
    """
    Removes duplicate points from a 2D NumPy array while preserving the original order,
    and removes corresponding flags for the removed points.

    :param path: A 2D NumPy array where each row is a point [x, y].
    :param flags: A list or array of flags corresponding to each point in the path.
    :return: A tuple (unique_path, unique_flags), where:
             - unique_path is a 2D NumPy array with duplicates removed, order preserved.
             - unique_flags is a list of flags corresponding to the unique points.
    """
    seen = set()
    unique_path = []
    unique_flags = []

    for point, flag in zip(path, flags):
        # Convert the point to a tuple (hashable) for uniqueness
        point_tuple = tuple(point)
        if point_tuple not in seen:
            seen.add(point_tuple)
            unique_path.append(point)
            unique_flags.append(flag)

    return np.array(unique_path), unique_flags


def connect_path_for_tsp(start_pair, intersections):
    """ Create a TSP path starting from a given pair and connecting all intersections in sequence.

    :param start_pair: Tuple, (start, end) pair to initiate the path
    :param intersections: List of intersection points
    :return: Complete path connecting intersections in order
    """
    path = [start_pair[0], start_pair[1]]
    for intersection in intersections[1:]:
        path = add_intersection_points_to_path(path, intersection)
    return path