import numpy as np
from reroute_helper_functions import is_point_on_edges, find_best_path, line_intersection

def avoid_obstacles(start_point, end_point, obstacles, prev_polygon, intersected_edges):
    """Finds a path avoiding obstacles between the start and end points.

    :param start_point: Tuple representing the start point.
    :param end_point: Tuple representing the end point.
    :param obstacles: List of obstacles containing edges to avoid.
    :param prev_polygon: The previous polygon (start point is end point for this polygons path)
    :param intersected_edges: List of intersected edges (obstacle edges)
    :return: List of intermediate points for the path.
    """

    # The path from start to end point crosses into an obstacle, and an intermediate path to reroute around it must be found
    # Finding the closest edge to start point that was intersected, using this to generate new start points
    closest_intersected_edge = find_closest_intersected_edge(start_point, intersected_edges, prev_polygon)
    print(f"Closest intersected edge: {closest_intersected_edge}")

    # Computing paths going left and right direction around the obstacle
    left_temp_path = compute_intermediate_path([start_point, closest_intersected_edge[0]], start_point, end_point, closest_intersected_edge, obstacles, prev_polygon, direction ='left')
    right_temp_path = compute_intermediate_path([start_point, closest_intersected_edge[1]], start_point, end_point, closest_intersected_edge, obstacles, prev_polygon, direction ='right')

    # Finding the best direction around the obstacles (prioritizes fewer turns)
    intermediate_path = find_best_path(left_temp_path, right_temp_path)

    return intermediate_path[1:]  # Start point in path gets appended in connect path before, so not included in intermediate path


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

    closest_edge = prev_intersecting_edge
    counter = 0

    print(f"Direction: {direction}")

    while True:
        # Current start point for this iteration
        new_start_point = temp_path[-1]
        print(f"new start point: {new_start_point}")

        # Checking for new intersections from new start point to the original end point
        intersected_edges = find_obstacle_intersections(new_start_point, end_point, obstacles)
        print(f"next intersected edges: {intersected_edges}")

        # Filter edges to remove those that are using the new start point as vertex in them
        filtered_edges = filter_intersected_edges(closest_edge, intersected_edges, obstacles)
        print(f"Next filtered edges: {filtered_edges}")

        # If 1 intersection, it is an obstacle vertex, and is clear, same for 0
        if len(filtered_edges) < 2:
            print(f"Clear path found going {direction}")
            break

        # Finding the closest intersected edge
        closest_edge = find_closest_intersected_edge(new_start_point, filtered_edges, prev_polygon)
        print(f"Closest intersecting edge: {closest_edge}")

        # TODO: If new obstacle is intersected, then go left and right around this (recurse)

        if direction == "left":
            if shorter_path(start_point, closest_edge[0], obstacles):
                temp_path = [temp_path[0], closest_edge[0]]
                print(f"Shorter left")
            else:
                temp_path.append(closest_edge[0])

        else:
            if shorter_path(start_point, closest_edge[1], obstacles):
                temp_path = [temp_path[0], closest_edge[1]]
                print(f"Shorter right")
            else:
                temp_path.append(closest_edge[1])

        # Should never be reached
        if counter > 1000:
            print(f"Max iterations reached, no clear path found going {direction}")
            return []
        counter += 1

    #print(f"Going: {direction}, path: {temp_path}")
    return temp_path


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
                print(f"Line intersected: {edge}")
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
                    print(f"Start on edge: {start_on_edge}")
                    print(f"End on edge: {end_on_edge}")
                    print(f"The edge: {edge}")

    return intersected_edges


def get_obstacle_for_edge(edge, obstacles):
    """
    Finds the obstacle that contains the given edge.

    :param edge: Tuple representing the edge ((x1, y1), (x2, y2)).
    :param obstacles: List of obstacles, where each obstacle is a collection of edges.
    :return: The obstacle containing the edge.
    """
    for obstacle in obstacles:
        if edge in obstacle.edges:
            return obstacle
    return None


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
    """ Checks if the previous edge and intersected edge are from the same obstacle.

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


def find_closest_intersected_edge(start_point, intersected_edges, polygon, epsilon=1e-9):
    """
    Finds the closest intersected edge to the start point based on three distances:
    1. Distance to the closest vertex of the edge
    2. Distance to the midpoint of the edge
    3. Distance to the polygon centroid (weighted at 50%)

    :param start_point: Tuple representing the start point of the line.
    :param intersected_edges: List of edges, where each edge is a tuple ((x1, y1), (x2, y2)).
    :param polygon: Polygon object containing vertices to compute the centroid.
    :param epsilon: Float tolerance for floating-point comparisons.
    :return: Tuple representing the closest intersected edge.
    """
    closest_edge = None
    min_weighted_distance = float('inf')

    # Calculate the centroid of the polygon
    x_coords, y_coords = zip(*[(v.x, v.y) for v in polygon.vertices])
    centroid = (sum(x_coords) / len(x_coords), sum(y_coords) / len(y_coords))

    for edge in intersected_edges:
        if len(edge) < 2:  # Skip invalid edges
            continue

        v1, v2 = edge

        # Compute the midpoint of the edge
        edge_midpoint = ((v1[0] + v2[0]) / 2, (v1[1] + v2[1]) / 2)

        # Compute distances
        dist_to_vertex1 = np.linalg.norm(np.array(start_point) - np.array(v1))
        dist_to_vertex2 = np.linalg.norm(np.array(start_point) - np.array(v2))
        dist_to_closest_vertex = min(dist_to_vertex1, dist_to_vertex2)
        dist_to_midpoint = np.linalg.norm(np.array(start_point) - np.array(edge_midpoint))
        dist_to_centroid = np.linalg.norm(np.array(edge_midpoint) - np.array(centroid))

        # Compute weighted distance metric
        weighted_distance = (
            dist_to_closest_vertex +
            dist_to_midpoint +
            0.5 * dist_to_centroid
        )

        # Update the closest edge if this one is better
        if weighted_distance < min_weighted_distance:
            min_weighted_distance = weighted_distance
            closest_edge = edge

    return closest_edge


def filter_initial_intersections(intersected_edges, obstacles, start_point, end_point):
    """
    Filters the initial intersected edges based on whether they belong to the same obstacle
    and additional criteria for clear paths and single-edge obstacles.

    :param intersected_edges: List of intersected edges.
    :param obstacles: List of obstacles, where each obstacle is a collection of edges.
    :param start_point: Tuple representing the starting point of the line.
    :param end_point: Tuple representing the ending point of the line.
    :return: Filtered list of intersected edges.
    """
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
        else:
            print("Not from same obstacle")
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