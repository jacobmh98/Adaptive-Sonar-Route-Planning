import numpy as np
from reroute_helper_functions import is_point_on_edges, find_best_path, line_intersection


def avoid_region_edges(start_point, end_point, intersected_region_edges, region):

    closest_intersected_region_edge = find_closest_intersected_region_edge(start_point, intersected_region_edges)
    print(f"Closest region edge: {closest_intersected_region_edge}")

    left_temp_path = compute_intermediate_region_path([start_point, closest_intersected_region_edge[0]], start_point, end_point, closest_intersected_region_edge, region, 'left')
    right_temp_path = compute_intermediate_region_path([start_point, closest_intersected_region_edge[1]], start_point, end_point, closest_intersected_region_edge, region, 'right')
    intermediate_path = find_best_path(left_temp_path, right_temp_path)

    return intermediate_path

def compute_intermediate_region_path(temp_path, start_point, end_point, closest_intersected_region_edge, region, direction):

    print(f"Direction region: {direction}")
    counter = 0
    prev_intersected_edge = closest_intersected_region_edge

    while True:

        new_start_point = temp_path[-1]
        print(f"New start point: {new_start_point}")

        intersected_edges = find_region_hard_edge_intersections(new_start_point, end_point, region)
        print(f"New intersected edges: {intersected_edges}")

        filtered_edges = filter_intersected_region_edges(intersected_edges, new_start_point, end_point, region)
        print(f"New filtered edges: {filtered_edges}")

        if len(filtered_edges) == 0:
            return temp_path

        closest_edge = find_closest_intersected_region_edge(new_start_point, filtered_edges, prev_intersected_edge)
        prev_intersected_edge = closest_edge

        if direction == 'left':

            if shorter_path_region(start_point, closest_edge[0], region, epsilon=1e-9):
                temp_path = [temp_path[0], closest_edge[0]]
            else:
                temp_path.append(closest_edge[0])
        else:
            if shorter_path_region(start_point, closest_edge[1], region, epsilon=1e-9):
                temp_path = [temp_path[0], closest_edge[1]]
            else:
                temp_path.append(closest_edge[1])

        # Should never be reached
        if counter > 1000:
            print(f"Max iterations reached, no clear path found going {direction}")
            return []
        counter += 1


def find_closest_intersected_region_edge(start_point, intersected_edges, ignored_edge=None, epsilon=1e-9):
    """
    Finds the closest intersected edge to the start point based on three metrics:
    1. Distance to the closest vertex of the edge
    2. Distance to the midpoint of the edge
    3. Perpendicular distance from the start_point to the edge

    :param start_point: Tuple (x, y) representing the start point.
    :param intersected_edges: List of edges as tuples ((x1, y1), (x2, y2)).
    :param ignored_edge: Edge to ignore in the closest edge search (tuple ((x1, y1), (x2, y2))).
    :param epsilon: Float tolerance for floating-point comparisons.
    :return: Tuple representing the closest intersected edge.
    """
    closest_edge = None
    min_distance = float('inf')

    # If there is only one edge, return it unless it is the ignored edge
    if len(intersected_edges) == 1:
        if intersected_edges[0] != ignored_edge:
            return intersected_edges[0]
        else:
            return None  # Return None if the single edge is the ignored edge

    for edge in intersected_edges:
        if edge == ignored_edge:  # Skip the ignored edge
            continue

        v1, v2 = edge

        # Compute distances to the vertices
        dist_to_v1 = np.linalg.norm(np.array(start_point) - np.array(v1))
        dist_to_v2 = np.linalg.norm(np.array(start_point) - np.array(v2))
        closest_vertex_distance = min(dist_to_v1, dist_to_v2)

        # Compute distance to the midpoint of the edge
        edge_midpoint = ((v1[0] + v2[0]) / 2, (v1[1] + v2[1]) / 2)
        dist_to_midpoint = np.linalg.norm(np.array(start_point) - np.array(edge_midpoint))

        # Compute perpendicular distance from the point to the edge
        edge_vector = np.array(v2) - np.array(v1)
        point_vector = np.array(start_point) - np.array(v1)
        cross_product = np.abs(np.cross(edge_vector, point_vector))
        edge_length = np.linalg.norm(edge_vector)
        if edge_length > epsilon:
            perpendicular_distance = cross_product / edge_length
        else:
            perpendicular_distance = float('inf')  # Handle degenerate edges

        # Combine distances to create a weighted score
        weighted_distance = (
            0.5 * closest_vertex_distance +
            0.3 * dist_to_midpoint +
            0.2 * perpendicular_distance
        )

        # Update the closest edge if this one is better
        if weighted_distance < min_distance:
            min_distance = weighted_distance
            closest_edge = edge

    return closest_edge


def filter_intersected_region_edges(intersected_edges, start_point, end_point, region, epsilon=1e-9):
    """
    Filters intersected region edges based on whether the line segment between start_point and end_point
    is inside or outside the region, or if start_point and end_point lie on the same or adjacent edges.

    :param intersected_edges: List of intersected edges as tuples ((x1, y1), (x2, y2)).
    :param start_point: Tuple (x, y) representing the start of the line segment.
    :param end_point: Tuple (x, y) representing the end of the line segment.
    :param region: A Polygon object representing the region.
    :param epsilon: Float tolerance for floating-point comparisons.
    :return: Filtered list of intersected edges. Returns an empty list if:
             1. The line is inside the region.
             2. start_point and end_point lie on the same or adjacent intersected edges.
    """
    if len(intersected_edges) == 1:
        # Single intersected edge case
        edge = intersected_edges[0]

        # Check if start_point or end_point lies on the edge
        start_on_edge = is_point_on_edges(start_point, [edge], epsilon)
        end_on_edge = is_point_on_edges(end_point, [edge], epsilon)

        if start_on_edge or end_on_edge:
            # Use the winding algorithm to determine if the line is inside the region
            if line_enters_region(start_point, end_point, region, epsilon):
                return []  # The line is inside the region; remove the edge

    elif len(intersected_edges) > 1:
        # Multiple intersected edges case
        # Identify edges on which the start_point lies
        edges_with_start = [edge for edge in intersected_edges if is_point_on_edges(start_point, [edge], epsilon)]
        print(f"Edges with start: {edges_with_start}")
        # Identify edges on which the end_point lies
        edges_with_end = [edge for edge in intersected_edges if is_point_on_edges(end_point, [edge], epsilon)]
        print(f"Edges with end: {edges_with_end}")

        # If start_point lies on one or more edges, check if end_point lies on the same or adjacent edges
        if edges_with_start and edges_with_end:
            # Check for overlap between the two sets
            for edge_start in edges_with_start:
                if edge_start in edges_with_end:
                    # Both points lie on the same edge
                    return []

            # If end_point is on a different edge, check if the line enters the region
            if line_enters_region(start_point, end_point, region, epsilon):
                return []

    return intersected_edges


def shorter_path_region(start_point, end_point, region, epsilon=1e-9):
    """
    Checks if the line segment from start_point to end_point enters any region hard edge.

    :param start_point: Tuple representing the starting point of the line segment (x, y).
    :param end_point: Tuple representing the ending point of the line segment (x, y).
    :param region: A Polygon object representing the region to check.
    :param epsilon: Float tolerance for precision issues.
    :return: True if the line segment does NOT intersect any hard edge in the region, False otherwise.
    """
    # Get all hard edges from the region
    hard_edges = [edge for edge in region.edges if edge.is_hard_edge]

    for edge in hard_edges:
        v1 = (edge.v_from.x, edge.v_from.y)
        v2 = (edge.v_to.x, edge.v_to.y)

        # Check if the line intersects the hard edge
        if line_intersection(start_point, end_point, v1, v2, epsilon):
            return False  # Line intersects a hard edge

    return True  # Line does not intersect any hard edge


def line_enters_region(start_point, end_point, region, epsilon=1e-9):
    """
    Determines if the line segment between start_point and end_point is inside the region using the winding number algorithm.

    :param start_point: Tuple (x, y) representing the start of the line segment.
    :param end_point: Tuple (x, y) representing the end of the line segment.
    :param region: A Polygon object representing the region.
    :param epsilon: Float tolerance for floating-point comparisons.
    :return: True if the line segment is inside the region, False otherwise.
    """
    def calculate_winding_number(point, vertices): # TODO use other winding implementation for both
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

    # Check the midpoint of the line segment
    midpoint = ((start_point[0] + end_point[0]) / 2, (start_point[1] + end_point[1]) / 2)
    return calculate_winding_number(midpoint, region.vertices)


def find_region_hard_edge_intersections(start_point, end_point, region, epsilon=1e-9):
    """
    Finds all hard edges in the given region that intersect with a line segment or contain the start/end point.

    :param start_point: Tuple (x, y) representing the start of the line segment.
    :param end_point: Tuple (x, y) representing the end of the line segment.
    :param region: A Polygon object representing the region to check.
    :param epsilon: Float tolerance for floating-point comparisons.
    :return: List of intersected hard edges as tuples ((x1, y1), (x2, y2)).
    """
    intersected_hard_edges = []

    # Check all edges in the region
    for edge in region.edges:
        if edge.is_hard_edge:  # Only consider hard edges
            v1 = (edge.v_from.x, edge.v_from.y)
            v2 = (edge.v_to.x, edge.v_to.y)

            # Check if the hard edge intersects the start-end line
            if line_intersection(start_point, end_point, v1, v2, epsilon):
                intersected_hard_edges.append((v1, v2))
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
                    intersected_hard_edges.append((v1, v2))

    return intersected_hard_edges

