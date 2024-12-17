import numpy as np
import coverage_plots
import detect_and_avoid_hard_edges
from detect_and_avoid_hard_edges import avoid_hard_edges


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

def connect_first_path(next_poly, intersections):
    # Find nearest vertex in next_poly to either first or last intersection
    nearest_intersection_point, nearest_intersection_index = find_nearest_to_vertex(next_poly, intersections[0], intersections[-1])

    first_path = [nearest_intersection_point]  # Initialize path with the nearest intersection point

    # Getting index of nearest intersection in intersection list (can only be first or last element)
    if nearest_intersection_index != 0:
        nearest_intersection_index = len(intersections) - 1

    # Add the second point from the nearest intersection
    if np.array_equal(np.array(intersections[nearest_intersection_index][0]), np.array(nearest_intersection_point)):
        first_path.append(intersections[nearest_intersection_index][1])
    else:
        first_path.append(intersections[nearest_intersection_index][0])

    # Go through the intersection list in normal order (first to last)
    if nearest_intersection_index == 0:
        for intersection in intersections[1:]:  # Iterate through remaining intersections and add to the first path
            first_path = add_intersection_points_to_path(first_path, intersection)

    else:  # Iterate through the intersection list backwards
        for intersection in reversed(intersections[:-1]):
            first_path = add_intersection_points_to_path(first_path, intersection)

    return first_path[::-1] # For first path, the start point is the nearest point in next polygon, so must be reversed

def connect_middle_path(polygons, total_intersections, i, path):
    last_path_point = path[-1]
    intersections = total_intersections[i]
    nearest_intersection_point, nearest_intersection_index = find_nearest_to_point(last_path_point, intersections[0], intersections[-1])
    middle_path = [nearest_intersection_point]

    # Getting index of nearest intersection in intersection list (can only be first or last element)
    if nearest_intersection_index != 0:
        nearest_intersection_index = len(intersections) - 1

    # Add the second point from the nearest intersection
    if np.array_equal(np.array(intersections[nearest_intersection_index][0]), np.array(nearest_intersection_point)):
        middle_path.append(intersections[nearest_intersection_index][1])
    else:
        middle_path.append(intersections[nearest_intersection_index][0])

    # Go through the intersection list in normal order (first to last)
    if nearest_intersection_index == 0:
        for inter in intersections[1:]:  # Iterate through remaining intersections and add to the first path
            middle_path = add_intersection_points_to_path(middle_path, inter)

    else:  # Iterate through the intersection list backwards
        for inter in reversed(intersections[:-1]):
            middle_path = add_intersection_points_to_path(middle_path, inter)

    # Check for better path in opposite direction
    opposite_path = connect_first_path(polygons[i+1], intersections)

    # Finding the path's start and end points
    start_point = last_path_point  # Must always start at last path's end
    end_point1 = find_nearest_vertex_to_point(polygons[i+1], middle_path[-1]).v.flatten()
    end_point2 = find_nearest_vertex_to_point(polygons[i+1], opposite_path[-1]).v.flatten()

    # Appending start and end points to both paths and calculating the total distances
    check_middle = [start_point] + middle_path + [end_point1]
    check_opposite = [start_point] + opposite_path + [end_point2]

    #coverage_plots.multi_poly_plot(polygons[i], polygons, np.array(check_middle))
    #coverage_plots.multi_poly_plot(polygons[i], polygons, np.array(check_opposite))

    # Checking total distances travelled in each path
    dist1 = compute_total_distance(check_middle)
    dist2 = compute_total_distance(check_opposite)

    if dist1 <= dist2:
        #print("middle")
        return middle_path
    else:
        #print("opposite")
        return opposite_path

def connect_last_path(path, intersections):
    last_path_point = path[-1]

    nearest_intersection_point, nearest_intersection_index = find_nearest_to_point(last_path_point, intersections[0], intersections[-1])
    last_path = [nearest_intersection_point]

    # Getting index of nearest intersection in intersection list (can only be first or last element)
    if nearest_intersection_index != 0:
        nearest_intersection_index = len(intersections) - 1

    # Add the second point from the nearest intersection
    if np.array_equal(np.array(intersections[nearest_intersection_index][0]), np.array(nearest_intersection_point)):
        last_path.append(intersections[nearest_intersection_index][1])
    else:
        last_path.append(intersections[nearest_intersection_index][0])

    # Go through the intersection list in normal order (first to last)
    if nearest_intersection_index == 0:
        for intersection in intersections[1:]:  # Iterate through remaining intersections and add to the first path
            last_path = add_intersection_points_to_path(last_path, intersection)

    else:  # Iterate through the intersection list backwards
        for intersection in reversed(intersections[:-1]):
            last_path = add_intersection_points_to_path(last_path, intersection)

    return last_path

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

def connect_path(polygons, total_intersections, region):
    path = np.empty((0, 2))
    transit_flags = []  # List to store transit flags

    for i, poly in enumerate(polygons):
        current_path = []
        current_transit_flags = []

        # In case of just 1 polygon, then the optimal intersections create the optimal path
        if len(polygons) == 1:
            current_path = connect_solo_path(total_intersections[i])
            current_transit_flags = ["transit" if j == 0 or j == len(current_path) - 1 else None for j in range(len(current_path))]

        # First poly case, has no start point, so start at closest point in next polygon's intersections and reverse its path
        elif i == 0 and len(polygons) > 1:
            current_path = connect_first_path(polygons[i + 1], total_intersections[i])
            current_transit_flags = ["transit" if j == 0 or j == len(current_path) - 1 else None for j in range(len(current_path))]

        # Going through the middle polygons
        elif i < len(polygons) - 1:
            current_path = connect_middle_path(polygons, total_intersections, i, path)
            current_transit_flags = ["transit" if j == 0 or j == len(current_path) - 1 else None for j in range(len(current_path))]

        # Last polygon is simple, just start at the end of the path so far for optimal path
        elif i == len(polygons) - 1:
            current_path = connect_last_path(path, total_intersections[i])
            current_transit_flags = ["transit" if j == 0 or j == len(current_path) - 1 else None for j in range(len(current_path))]

        avoid_hard_edges_bool = False
        if avoid_hard_edges_bool:
            hard_edges = extract_hard_edges(polygons)

            if i > 0 and len(hard_edges) > 0:
                last_path_point = path[-1]
                current_first_point = current_path[0]
                intermediate_points = detect_and_avoid_hard_edges.avoid_hard_edges(
                    last_path_point, current_first_point, poly, polygons, region, hard_edges)

                if intermediate_points:
                    # Append the intermediate points to the path
                    for point in intermediate_points:
                        path = np.vstack([path, point])
                        transit_flags.append(None)  # Intermediate points are not transits

        # Should never be an empty path, but just in case check to avoid error
        if len(current_path) > 0:
            path = np.vstack([path, current_path])
            transit_flags.extend(current_transit_flags)

    return path, transit_flags
