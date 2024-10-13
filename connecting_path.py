import numpy as np
import coverage_plots

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
    if all(intersections[nearest_intersection_index][0][i] == nearest_intersection_point[i] for i in range(len(nearest_intersection_point))):
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

# TODO: Add logic to check distance from end of path to next start point, and check if moving start is beneficial (ie. start at good last point instead)
def connect_middle_path(polygons, total_intersections, i, path):
    last_path_point = path[-1]
    intersections = total_intersections[i]

    nearest_intersection_point, nearest_intersection_index = find_nearest_to_point(last_path_point, intersections[0], intersections[-1])
    middle_path = [nearest_intersection_point]

    # Getting index of nearest intersection in intersection list (can only be first or last element)
    if nearest_intersection_index != 0:
        nearest_intersection_index = len(intersections) - 1

    # Add the second point from the nearest intersection
    if all(intersections[nearest_intersection_index][0][i] == nearest_intersection_point[i] for i in
           range(len(nearest_intersection_point))):
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

    # Check for better path
    opposite_path = connect_first_path(polygons[i+1], intersections)

    # Finding path's start and end points
    start_point = last_path_point
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
    if all(intersections[nearest_intersection_index][0][i] == nearest_intersection_point[i] for i in
           range(len(nearest_intersection_point))):
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

def connect_solo_path(intersections):
    solo_path = [intersections[0][0],intersections[0][1]] # Manually adding first intersection
    for intersection in intersections[1:]:  # Iterate through remaining intersections and add to the first path
        solo_path = add_intersection_points_to_path(solo_path, intersection)
    return solo_path

def connect_path(polygons, total_intersections):

    path = np.empty((0,2))

    for i, poly in enumerate(polygons):
        current_path = []

        # In case of just 1 polygon, then the optimal intersections create the optimal path
        if len(polygons) == 1:
            current_path = connect_solo_path(total_intersections[i])

        # First poly edge case, has no start point, so start at closest point in next polygon intersection and reverse its path
        elif i == 0 and len(polygons) > 1:
            current_path = connect_first_path(polygons[i+1], total_intersections[i])

        # Going through the middle polygons
        elif i < len(polygons) - 1:
            current_path = connect_middle_path(polygons, total_intersections, i, path)

        # Last polygon is simple, just start at the end of the path so far
        elif i == len(polygons) - 1:
            current_path = connect_last_path(path, total_intersections[i])

        # Should never be an empty path, but just in case check to avoid error
        if len(current_path) > 0:
            path = np.vstack([path, current_path])

    return path