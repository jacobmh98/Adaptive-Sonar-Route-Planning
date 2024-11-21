import numpy as np

def compute_path_distance(path):
    """Computes the total distance traveled along a path.

    :param path: List of points representing the path, where each point is a tuple (x, y) or a numpy array.
    :return: Float representing the total distance traveled along the path.
    """

    if len(path) < 2:
        return 0.0  # No distance if the path has fewer than 2 points

    total_distance = 0.0

    for i in range(len(path) - 1):
        point_a = np.array(path[i])
        point_b = np.array(path[i + 1])
        total_distance += np.linalg.norm(point_b - point_a)

    return total_distance


def line_intersection(p1, p2, q1, q2, epsilon=1e-9):
    """Checks if line segment (p1, p2) intersects (q1, q2).

    :param p1: Tuple representing the start of the first line segment.
    :param p2: Tuple representing the end of the first line segment.
    :param q1: Tuple representing the start of the second line segment.
    :param q2: Tuple representing the end of the second line segment.
    :param epsilon: Float tolerance for floating-point comparisons.
    :return: Boolean indicating whether the segments intersect.
    """

    r = np.array(p2) - np.array(p1)
    s = np.array(q2) - np.array(q1)
    r_cross_s = np.cross(r, s)

    if abs(r_cross_s) < epsilon:
        return False  # The lines are parallel or collinear

    q1_p1 = np.array(q1) - np.array(p1)
    t = np.cross(q1_p1, s) / r_cross_s
    u = np.cross(q1_p1, r) / r_cross_s

    return 0 <= t <= 1 and 0 <= u <= 1


def find_obstacle_intersections(start_point, end_point, obstacles, epsilon=1e-9):
    """Finds all edges that intersect with a line segment or contain its endpoints.

    :param start_point: Tuple representing the starting point of the line.
    :param end_point: Tuple representing the ending point of the line.
    :param obstacles: List of obstacles, where each obstacle has edges to check.
    :param epsilon: Float tolerance for floating-point comparisons.
    :return: List of intersected edges.
    """

    intersected_edges = []

    # Iterate through all obstacles
    for obstacle in obstacles:
        for edge in obstacle.obstacle_edges():
            v1, v2 = edge

            # Check if the edge intersects the line
            if line_intersection(start_point, end_point, v1, v2):
                intersected_edges.append(edge)
            else:
                # Check if start_point or end_point lies on the edge
                edge_vector = np.array(v2) - np.array(v1)
                start_vector = np.array(start_point) - np.array(v1)
                end_vector = np.array(end_point) - np.array(v1)

                # Check collinearity and within bounds
                start_on_edge = (
                    abs(np.cross(edge_vector, start_vector)) < epsilon and
                    0 <= np.dot(start_vector, edge_vector) <= np.dot(edge_vector, edge_vector)
                )
                end_on_edge = (
                    abs(np.cross(edge_vector, end_vector)) < epsilon and
                    0 <= np.dot(end_vector, edge_vector) <= np.dot(edge_vector, edge_vector)
                )

                if start_on_edge or end_on_edge:
                    intersected_edges.append(edge)

    return intersected_edges


def find_closest_intersected_edge(start_point, intersected_edges, polygon, epsilon=1e-9):
    """Finds the closest intersected edge to the polygon's centroid, based on edge center.

    :param start_point: Tuple representing the start point of the line.
    :param intersected_edges: List of edges, where each edge is a tuple ((x1, y1), (x2, y2)).
    :param polygon: Polygon object containing vertices to compute the centroid.
    :param epsilon: Float tolerance for floating-point comparisons.
    :return: Tuple representing the closest intersected edge.
    """

    closest_edge = None
    min_distance = float('inf')

    # Calculate the centroid of the polygon
    x_coords, y_coords = zip(*[(v.x, v.y) for v in polygon.vertices])
    centroid = (sum(x_coords) / len(x_coords), sum(y_coords) / len(y_coords))

    for edge in intersected_edges:
        if len(edge) < 2:  # Skip invalid edges
            continue

        v1, v2 = edge

        # Compute the midpoint of the edge
        edge_midpoint = ((v1[0] + v2[0]) / 2, (v1[1] + v2[1]) / 2)

        # Calculate the distance from the edge midpoint to the polygon's centroid
        centroid_distance = np.linalg.norm(np.array(edge_midpoint) - np.array(centroid))

        # Check if the start point lies on this edge
        edge_vector = np.array(v2) - np.array(v1)
        point_vector = np.array(start_point) - np.array(v1)
        cross_product = np.cross(edge_vector, point_vector)
        dot_product = np.dot(point_vector, edge_vector)
        edge_length_squared = np.dot(edge_vector, edge_vector)

        if abs(cross_product) < epsilon and 0 <= dot_product <= edge_length_squared:
            return edge  # Return immediately if the start point lies on this edge

        # Update the closest edge if this one is closer
        if centroid_distance < min_distance:
            min_distance = centroid_distance
            closest_edge = edge

    return closest_edge


def filter_intersected_edges(previous_edge, intersected_edges, obstacles):
    """Filters out the previous edge and single intersected edges per obstacle.

    :param previous_edge: Tuple representing the previously intersected edge.
    :param intersected_edges: List of edges intersected by the line segment.
    :param obstacles: List of obstacles, where each obstacle has a method to provide its edges.
    :return: List of filtered intersected edges.
    """

    # Step 1: Remove the previous edge
    filtered_edges = [edge for edge in intersected_edges if edge != previous_edge]

    # Step 2: Create a mapping of obstacles to their intersected edges
    obstacle_intersection_map = {id(obstacle): [] for obstacle in obstacles}

    for edge in filtered_edges:
        for obstacle in obstacles:
            if edge in obstacle.obstacle_edges():
                obstacle_intersection_map[id(obstacle)].append(edge)
                break

    # Step 3: Remove edges that are the only intersected edge from their obstacle
    final_filtered_edges = []
    for edge in filtered_edges:
        for obstacle in obstacles:
            if edge in obstacle.obstacle_edges() and len(obstacle_intersection_map[id(obstacle)]) > 1:
                final_filtered_edges.append(edge)
                break

    return final_filtered_edges


def are_edges_from_same_obstacle(prev_edge, inter_edge, obstacles):
    """Checks if the previous edge and intersected edge are from the same obstacle.

    :param prev_edge: Tuple representing the previous edge ((x1, y1), (x2, y2)).
    :param inter_edge: Tuple representing the intersected edge ((x1, y1), (x2, y2)).
    :param obstacles: List of obstacles, where each obstacle has a method to provide its edges.
    :return: Boolean indicating whether the two edges are from the same obstacle.
    """
    for obstacle in obstacles:
        obstacle_edges = obstacle.obstacle_edges()
        if prev_edge in obstacle_edges and inter_edge in obstacle_edges:
            return True
    return False


def find_best_path_direction(left_temp_path, right_temp_path):
    """Determines the best path direction between left and right paths.

    :param left_temp_path: List of points representing the left path.
    :param right_temp_path: List of points representing the right path.
    :return: The best path (list of points) with fewer turns or shorter distance.
    """
    if len(left_temp_path) == len(right_temp_path):
        if compute_path_distance(left_temp_path) <= compute_path_distance(right_temp_path):
            print("left smaller distance")
            return left_temp_path
        else:
            print("right smaller distance")
            return right_temp_path

    elif len(left_temp_path) <= len(right_temp_path):
        print("left less turns")
        return left_temp_path

    else:
        print("right less turns")
        return right_temp_path


def follow_and_create_path(temp_path, start_point, end_point, prev_intersecting_edge, obstacles, coming_from_polygon, direction):
    """Follows and creates a path around obstacles in the specified direction.

    :param temp_path: List of points representing the current path.
    :param end_point: Tuple representing the endpoint of the path.
    :param prev_intersecting_edge: Tuple representing the last intersecting edge.
    :param obstacles: List of obstacles containing edges to avoid.
    :param direction: String indicating the direction to follow ('left' or 'right').
    :return: List of points representing the updated path.
    """

    intersecting_edge = prev_intersecting_edge
    counter = 0
    best_alternate_path = [0] * 10000
    print(f"Direction: {direction}")

    while True:
        # Current start point for this iteration
        new_start_point = temp_path[-1]
        print(f"new start point: {new_start_point}")

        # Checking for new intersections from new start point to the original end point
        intersected_edges = find_obstacle_intersections(new_start_point, end_point, obstacles)
        print(f"next intersected edges: {intersected_edges}")

        # Filter edges to remove those that are using the new start point as vertex in them
        filtered_edges = filter_intersected_edges(intersecting_edge, intersected_edges, obstacles)
        print(f"Next filtered edges: {filtered_edges}")

        if len(filtered_edges) < 2:
            #print(f"Clear path found going {direction}")
            break

        # Storing the previous intersected edge to check if new obstacle is intersected
        prev_intersecting_edge = intersecting_edge

        # Finding the closest intersected edge
        intersecting_edge = find_closest_intersected_edge(new_start_point, filtered_edges, coming_from_polygon)
        print(f"Closest intersecting edge: {intersecting_edge}")


        if direction == "left":
            #if shorter_path(start_point, new_start_point, intersecting_edge, obstacles):
            #    temp_path = []
            #else:
            temp_path.append(intersecting_edge[0])
        else:
            temp_path.append(intersecting_edge[1])

        if counter > 50:
            print(f"Max iterations reached, no clear path found going {direction}")
            break
        counter += 1

    return temp_path


def avoid_obstacles(start_point, end_point, obstacles, coming_from_polygon):
    """Finds a path avoiding obstacles between the start and end points.

    :param start_point: Tuple representing the start point.
    :param end_point: Tuple representing the end point.
    :param obstacles: List of obstacles containing edges to avoid.
    :return: List of intermediate points for the path.
    """

    print(f"Start point: {start_point}")
    print(f"End point: {end_point}")

    intersected_edges = find_obstacle_intersections(start_point, end_point, obstacles)
    print(f"initial intersected edges {intersected_edges}")

    # Obstacle vertex intersected, but does not enter the obstacle, so we count it as a clear path
    if len(intersected_edges) == 1:
        print("Obstacle Vertex intersected, Clear path")
        return []

    # No obstacle intersections found
    elif len(intersected_edges) == 0:
        print("No intersections, Clear path")
        return []

    # The path from start to end point crosses into an obstacle, and an intermediate path to reroute around it must be found
    else:
        print("Obstacle intersected")

        # Finding the closest edge to start point that was intersected
        closest_intersected_edge = find_closest_intersected_edge(start_point, intersected_edges, coming_from_polygon)
        print(f"Closest intersected edge: {closest_intersected_edge}")

        # Looking for a path around the obstacles
        left_temp_path = follow_and_create_path([start_point, closest_intersected_edge[0]], start_point, end_point, closest_intersected_edge, obstacles, coming_from_polygon, direction = 'left')
        right_temp_path = follow_and_create_path([start_point, closest_intersected_edge[1]], start_point, end_point, closest_intersected_edge, obstacles, coming_from_polygon, direction = 'right')

        # Finding the best direction around the obstacles (prioritizes fewer turns)
        intermediate_points = find_best_path_direction(left_temp_path, right_temp_path)


    return intermediate_points[1:]  # Start point in path gets appended in connect path before, so not included in intermediate path
