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


def filter_intersected_edges(prev_intersecting_edge, intersected_edges, obstacles):
    """Filters out the previous edge and single intersected edges per obstacle.

    :param prev_intersecting_edge: Tuple representing the previous intersected edge.
    :param intersected_edges: List of edges intersected by the line segment.
    :param obstacles: List of obstacles, where each obstacle has a method to provide its edges.
    :return: List of filtered intersected edges.
    """
    # Step 1: Remove the previous edge
    filtered_edges = [edge for edge in intersected_edges if edge != prev_intersecting_edge]

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
    """
    Checks if the previous edge and intersected edge are from the same obstacle.

    :param prev_edge: Tuple representing the previous edge ((x1, y1), (x2, y2)).
    :param inter_edge: Tuple representing the intersected edge ((x1, y1), (x2, y2)).
    :param obstacles: List of obstacles, where each obstacle has a method to provide its edges.
    :return: Tuple (boolean, obstacle) where the boolean indicates whether the edges are from the same obstacle,
             and the obstacle is the corresponding obstacle object if True, or None if False.
    """
    for obstacle in obstacles:
        obstacle_edges = obstacle.obstacle_edges()
        if prev_edge in obstacle_edges and inter_edge in obstacle_edges:
            return True, obstacle
    return False, None


def find_best_path(left_temp_path, right_temp_path):
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


def is_going_into_obstacle(point1, point2, obstacle, epsilon=1e-9):
    """ Determines if the line segment between two points enters the obstacle.

    :param point1: First point of the line segment as a tuple (x, y).
    :param point2: Second point of the line segment as a tuple (x, y).
    :param obstacle: The Polygon object representing the obstacle.
    :param epsilon: Float tolerance for precision issues.
    :return: Boolean indicating whether the line segment enters the obstacle.
    """
    # Calculate the midpoint of the line segment
    midpoint = ((point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2)

    # Extract vertices of the obstacle
    vertices = obstacle.vertices
    x_mid, y_mid = midpoint
    n = len(vertices)

    # Ray-casting algorithm to determine if the midpoint is inside the polygon
    inside = False
    for i in range(n):
        v1 = vertices[i]
        v2 = vertices[(i + 1) % n]  # Wrap around to the first vertex

        x1, y1 = v1.x, v1.y
        x2, y2 = v2.x, v2.y

        # Check if the midpoint lies on the edge
        if abs((x2 - x1) * (y_mid - y1) - (y2 - y1) * (x_mid - x1)) < epsilon:  # Collinear
            if min(x1, x2) - epsilon <= x_mid <= max(x1, x2) + epsilon and \
               min(y1, y2) - epsilon <= y_mid <= max(y1, y2) + epsilon:
                return True  # Midpoint lies on an edge of the polygon

        # Check if the ray crosses the edge
        if ((y1 > y_mid) != (y2 > y_mid)) and \
                (x_mid < (x2 - x1) * (y_mid - y1) / (y2 - y1) + x1):
            inside = not inside

    return inside


def is_point_on_edges(point, edges, epsilon=1e-9):
    """ Checks if a given point lies on any edge in a list of edges.

    :param point: The point to check as a tuple (x, y).
    :param edges: List of edges, where each edge is represented as ((x1, y1), (x2, y2)).
    :param epsilon: Float tolerance for precision issues.
    :return: True if the point lies on any edge, False otherwise.
    """
    px, py = point

    for edge in edges:
        (x1, y1), (x2, y2) = edge

        # Calculate the vector from the edge's start to the point and the edge's vector
        edge_vector = (x2 - x1, y2 - y1)
        point_vector = (px - x1, py - y1)

        # Check if the point is collinear with the edge
        cross_product = abs(edge_vector[0] * point_vector[1] - edge_vector[1] * point_vector[0])
        if cross_product > epsilon:
            continue  # Not collinear

        # Check if the point is within the bounds of the edge
        dot_product = (point_vector[0] * edge_vector[0] + point_vector[1] * edge_vector[1])
        edge_length_squared = edge_vector[0]**2 + edge_vector[1]**2

        if 0 <= dot_product <= edge_length_squared:
            return True  # The point lies on this edge

    return False


def filter_initial_intersections(intersected_edges, obstacles, start_point, end_point):
    # If 0 or 1 intersections, then it is a clear path
    if len(intersected_edges) < 2:
        return intersected_edges

    # If 2 intersected edges, we check if they belong to the same obstacle, if so, we check if the line goes through that obstacle or if it is clear
    if len(intersected_edges) == 2:
        from_same, same_obstacle = are_edges_from_same_obstacle(intersected_edges[0], intersected_edges[1], obstacles)

        # Checking if the two intersected edges are from the same obstacle
        if from_same:
            print("From same obstacle")

            # Checking if start and end point are on the intersected edges
            if is_point_on_edges(start_point, intersected_edges) and is_point_on_edges(end_point, intersected_edges):
                print("Start and End on intersected edges")

                # Checking if a line from start to end is moving into or outside the obstacle
                if not is_going_into_obstacle(start_point, end_point, same_obstacle):
                    print("Not going into the obstacle")

                    # If just 2 intersections, and not going into obstacle, it is a clear path from start to end
                    return []

    return intersected_edges


def line_enters_obstacle(start_point, end_point, obstacle, epsilon=1e-9):
    """ Determines if the line segment from start_point to end_point enters the obstacle using the winding number.

    :param start_point: Tuple representing the starting point of the line segment (x, y).
    :param end_point: Tuple representing the ending point of the line segment (x, y).
    :param obstacle: The Polygon object representing the obstacle.
    :param epsilon: Float tolerance for precision issues.
    :return: True if the line segment enters the obstacle, False otherwise.
    """
    def calculate_winding_number(point, vertices):
        """Calculate the winding number for a point with respect to a polygon."""
        x, y = point
        winding_number = 0

        for i in range(len(vertices)):
            v1 = vertices[i]
            v2 = vertices[(i + 1) % len(vertices)]  # Wrap around to the first vertex
            x1, y1 = v1.x, v1.y
            x2, y2 = v2.x, v2.y

            # Calculate vectors
            vec1 = np.array([x1 - x, y1 - y])
            vec2 = np.array([x2 - x, y2 - y])

            # Calculate cross product and angle
            cross = np.cross(vec1, vec2)
            dot = np.dot(vec1, vec2)
            angle = np.arctan2(cross, dot)

            # Accumulate winding number
            winding_number += angle

        return abs(winding_number) > epsilon  # Non-zero winding number indicates the point is inside

    # Check winding numbers for both endpoints
    inside_start = calculate_winding_number(start_point, obstacle.vertices)
    inside_end = calculate_winding_number(end_point, obstacle.vertices)

    return inside_start or inside_end  # Line enters if either endpoint is inside


def shorter_path(start_point, end_point, obstacles, epsilon=1e-9):
    """ Checks if the line segment from start_point to end_point enters any obstacle.

    :param start_point: Tuple representing the starting point of the line segment (x, y).
    :param end_point: Tuple representing the ending point of the line segment (x, y).
    :param obstacles: List of obstacles, where each obstacle is a Polygon object.
    :param epsilon: Float tolerance for precision issues.
    :return: True if the line segment enters any obstacle, False otherwise.
    """
    for obstacle in obstacles:
        if line_enters_obstacle(start_point, end_point, obstacle, epsilon=epsilon):
            return False  # Line enters an obstacle

    return True  # Line does not enter any obstacle



def compute_intermediate_path(temp_path, start_point, end_point, prev_intersecting_edge, obstacles, prev_polygon, direction):
    """Follows and creates a path around obstacles in the specified direction.

    :param temp_path: List of points representing the current path.
    :param start_point: [x,y] coordinates of the start point.
    :param end_point: [x,y] coordinates of the start point.
    :param prev_intersecting_edge: Tuple representing the last intersecting edge.
    :param obstacles: List of obstacles containing edges to avoid.
    :param prev_polygon: The previous polygon (start point is end point for this polygons path)
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
            print(f"Clear path found going {direction}")
            break

        # Storing the previous intersected edge to check if new obstacle is intersected
        prev_intersecting_edge = intersecting_edge

        # Finding the closest intersected edge
        intersecting_edge = find_closest_intersected_edge(new_start_point, filtered_edges, prev_polygon)
        print(f"Closest intersecting edge: {intersecting_edge}")

        if direction == "left":
            if shorter_path(start_point, intersecting_edge[0], obstacles):
                temp_path = [temp_path[0], intersecting_edge[0]]
                print(f"Shorter left")
            else:
                temp_path.append(intersecting_edge[0])

        else:
            if shorter_path(start_point, intersecting_edge[1], obstacles):
                temp_path = [temp_path[0], intersecting_edge[1]]
                print(f"Shorter right")
            else:
                temp_path.append(intersecting_edge[1])

        if counter > 50:
            print(f"Max iterations reached, no clear path found going {direction}")
            break
        counter += 1

    print(f"Going: {direction}, path: {temp_path}")
    return temp_path


def avoid_obstacles(start_point, end_point, obstacles, prev_polygon):
    """Finds a path avoiding obstacles between the start and end points.

    :param start_point: Tuple representing the start point.
    :param end_point: Tuple representing the end point.
    :param obstacles: List of obstacles containing edges to avoid.
    :param prev_polygon: The previous polygon (start point is end point for this polygons path)
    :return: List of intermediate points for the path.
    """

    print(f"Start point: {start_point}")
    print(f"End point: {end_point}")

    intersected_edges = find_obstacle_intersections(start_point, end_point, obstacles)
    print(f"initial intersected edges {intersected_edges}")

    filtered_edges = filter_initial_intersections(intersected_edges, obstacles, start_point, end_point)
    print(f"Initial filtered intersections: {filtered_edges}")

    # Obstacle vertex intersected, but does not enter the obstacle, so we count it as a clear path
    if len(filtered_edges) < 2:
        print("No intersections, Clear path")
        return []

    # The path from start to end point crosses into an obstacle, and an intermediate path to reroute around it must be found
    else:
        print("Obstacle intersected")

        # Finding the closest edge to start point that was intersected, using this to generate new start points
        closest_intersected_edge = find_closest_intersected_edge(start_point, intersected_edges, prev_polygon)
        print(f"Closest intersected edge: {closest_intersected_edge}")

        # Computing paths going left and right direction around the obstacle
        left_temp_path = compute_intermediate_path([start_point, closest_intersected_edge[0]], start_point, end_point, closest_intersected_edge, obstacles, prev_polygon, direction ='left')
        right_temp_path = compute_intermediate_path([start_point, closest_intersected_edge[1]], start_point, end_point, closest_intersected_edge, obstacles, prev_polygon, direction ='right')

        # Finding the best direction around the obstacles (prioritizes fewer turns)
        intermediate_path = find_best_path(left_temp_path, right_temp_path)

    return intermediate_path[1:]  # Start point in path gets appended in connect path before, so not included in intermediate path