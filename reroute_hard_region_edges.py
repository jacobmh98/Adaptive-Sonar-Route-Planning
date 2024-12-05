import numpy as np
from reroute_helper_functions import is_point_on_edges, find_best_path, line_intersection, calculate_winding_number, \
    is_start_end_on_edge

def reroute_path_region(start_point, end_point, intersected_region_edges, region, i, polygons):
    """ Computes a path to avoid intersecting hard edges in a region.

    :param start_point: Tuple (x, y) representing the starting point of the path.
    :param end_point: Tuple (x, y) representing the ending point of the path.
    :param intersected_region_edges: List of hard edges intersected by the current path.
    :param region: The Polygon object representing the region.
    :param i: Index of current polygon
    :param polygons: List of polygons
    :return: List of points forming the intermediate path that avoids hard edges.
    """

    closest_intersected_region_edge = find_closest_intersected_region_edge(start_point, intersected_region_edges, None, polygons)
    #print(f"Closest region edge: {closest_intersected_region_edge}")

    left_temp_path, found_left_path = compute_intermediate_region_path([start_point, closest_intersected_region_edge[1]], end_point, closest_intersected_region_edge, region, i, polygons, 'left')
    right_temp_path, found_right_path = compute_intermediate_region_path([start_point, closest_intersected_region_edge[0]], end_point, closest_intersected_region_edge, region, i, polygons, 'right')
    #print(f"Found left: {found_left_path}")
    #print(f"Found right: {found_right_path}")

    if not found_left_path and not found_right_path:
        print("No path found")
        intermediate_path = []
    elif not found_left_path:
        intermediate_path = right_temp_path
    elif not found_right_path:
        intermediate_path = left_temp_path
    else:
        intermediate_path = find_best_path(left_temp_path, right_temp_path)
    return intermediate_path


def compute_intermediate_region_path(temp_path, end_point, closest_intersected_region_edge, region, i, polygons, direction):
    """ Iteratively computes a path around hard edges in a region.

    :param temp_path: List of points forming the current path.
    :param end_point: Tuple (x, y) representing the ending point of the path.
    :param closest_intersected_region_edge: The closest intersected hard edge to the start point.
    :param region: The Polygon object representing the region.
    :param i: The current polygon index
    :param polygons: list of sub polygons
    :param direction: String indicating the direction to avoid edges ('left' or 'right').
    :return: List of points forming the updated path.
    """
    #print(f"Direction region: {direction}")
    prev_intersected_edges = [closest_intersected_region_edge]
    prev_intersected_edge = closest_intersected_region_edge
    counter = 0

    while True:
        new_start_point = temp_path[-1]
        #print(f"New start point: {new_start_point}")

        intersected_edges = compute_hard_region_edge_intersections(new_start_point, end_point, region)
        #print(f"New intersected edges: {intersected_edges}")

        filtered_edges = filter_intersected_region_edges(new_start_point, end_point, intersected_edges, region)
        #print(f"New filtered edges: {filtered_edges}")

        # A clear path
        if len(filtered_edges) == 0:
            #print("Clear path")
            return temp_path, True

        #print(f"previous intersected: {prev_intersected_edges}")

        closest_edge = find_closest_intersected_region_edge(new_start_point, filtered_edges, prev_intersected_edge, polygons, prev_intersected_edges)
        #print(f"Closest edge1: {closest_edge}")

        if not closest_edge:
            print("No clear path, all intersections tried")
            return [], False

        if closest_edge in prev_intersected_edges:
            #print(f"prev intersected edges: {prev_intersected_edges}")
            #print(f"Edge already tried: {closest_edge}")

            left_adjacent, right_adjacent = find_adjacent_edges(region, closest_edge)
            #print(f"left adjacent, right adjacent: {left_adjacent}, {right_adjacent}")

            if direction == 'left':
                closest_edge = left_adjacent
            else:
                closest_edge = right_adjacent

        #print(f"closest edge: {closest_edge}")
        prev_intersected_edge = closest_edge

        # Keeping track of already checked edges
        prev_intersected_edges.append(closest_edge)

        if direction == 'left':
            is_shorter, shorter_points = is_shorter_region_path(closest_edge[1], temp_path, region)
            if is_shorter:
                temp_path = shorter_points + [closest_edge[1]]
                #print(f"Shorter path left: {temp_path}")
            else:
                temp_path.append(closest_edge[1])

        else:
            is_shorter, shorter_points = is_shorter_region_path(closest_edge[0], temp_path, region)
            if is_shorter:
                temp_path = shorter_points + [closest_edge[0]]
                #print(f"Shorter path right: {temp_path}")
            else:
                temp_path.append(closest_edge[0])

        # Should never be reached
        if counter > 1000:
            print(f"REGION: Max iterations reached, no clear path found going {direction}")
            return [], False
        counter += 1


def compute_hard_region_edge_intersections(start_point, end_point, region, epsilon=1e-9):
    """ Finds all hard edges in the given region that intersect with a line segment or contain the start/end point.

    :param start_point: Tuple (x, y) representing the start of the line segment.
    :param end_point: Tuple (x, y) representing the end of the line segment.
    :param region: A Polygon object representing the region to check.
    :param epsilon: Float tolerance for floating-point comparisons.
    :return: List of intersected hard edges as tuples ((x1, y1), (x2, y2)).
    """
    intersected_hard_edges = []
    #print(f"start point: {start_point}")
    #print(f"end point: {end_point}")

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
                start_on_edge, end_on_edge = is_start_end_on_edge(start_point, end_point, v1, v2)

                if start_on_edge or end_on_edge:
                    intersected_hard_edges.append((v1, v2))

    return intersected_hard_edges


def filter_intersected_region_edges(start_point, end_point, intersected_edges, region, epsilon=1e-9):
    """
    Filters intersected region edges based on whether the line segment between start_point and end_point
    is inside or outside the region, or if start_point and end_point lie on the same or adjacent edges.

    Cases handled:
    1. If there is only 1 intersected edge, check if the line enters the region. If it does, filter out the edge.
    2. If there are 2 to 4 intersected edges, handle cases where vertices or edges are shared:
       - Check if the line enters the region and filter edges accordingly.

    :param start_point: Tuple (x, y) representing the start of the line segment.
    :param end_point: Tuple (x, y) representing the end of the line segment.
    :param intersected_edges: List of intersected edges as tuples ((x1, y1), (x2, y2)).
    :param region: A Polygon object representing the region.
    :param epsilon: Float tolerance for floating-point comparisons.
    :return: Filtered list of intersected edges. Returns an empty list if:
             1. The line is inside the region.
             2. Conditions for shared vertices and edges are met.
    """
    # Case 1: Single intersected edge
    if len(intersected_edges) == 1:
        edge = intersected_edges[0]

        # Check if start_point or end_point lies on the edge
        start_on_edge = is_point_on_edges(start_point, [edge], epsilon)
        end_on_edge = is_point_on_edges(end_point, [edge], epsilon)

        if start_on_edge or end_on_edge:
            # Use the winding algorithm to determine if the line is entering the region
            if is_line_entering_region(start_point, end_point, region):
                return []  # The edge is not crossed, filter it out
        return intersected_edges  # Edge is crossed, keep it

    # Case 2: Special case for 2 to 4 intersected edges
    if 2 <= len(intersected_edges) <= 4:
        # Identify edges where start_point and end_point lie
        edges_with_start = [edge for edge in intersected_edges if is_point_on_edges(start_point, [edge], epsilon)]
        edges_with_end = [edge for edge in intersected_edges if is_point_on_edges(end_point, [edge], epsilon)]

        # Check if there is any edge that does not involve start_point or end_point
        for edge in intersected_edges:
            if edge not in edges_with_start and edge not in edges_with_end:
                # This edge does not share a vertex with either point; do not filter
                return intersected_edges

        # Check if the line enters the region
        if is_line_entering_region(start_point, end_point, region):
            return []  # The edges are not crossed, filter them out
        return intersected_edges  # The edges are crossed, keep them

    # Default: Return intersected edges if no condition is met
    return intersected_edges


def find_closest_intersected_region_edge(start_point, intersected_edges, previous_edge, regions, ignored_edges=None, epsilon=1e-9):
    """
    Finds the closest intersected edge to the start point, prioritizing edges from polygons containing the previous edge.
    If previous_edge is None, prioritization is skipped.

    :param start_point: Tuple (x, y) representing the start point.
    :param intersected_edges: List of edges as tuples ((x1, y1), (x2, y2)).
    :param previous_edge: Tuple ((x1, y1), (x2, y2)) representing the previous edge, or None.
    :param regions: List of polygons, where each polygon has an attribute `edges` (list of edge tuples).
    :param ignored_edges: List of edges to ignore during the search (list of tuples ((x1, y1), (x2, y2))).
    :param epsilon: Float tolerance for floating-point comparisons.
    :return: Tuple representing the closest intersected edge.
    """
    closest_priority_edge = None
    closest_non_priority_edge = None
    min_priority_distance = float('inf')
    min_non_priority_distance = float('inf')

    # Ensure ignored_edges is a valid list
    if ignored_edges is None:
        ignored_edges = []

    # Initialize priority_edges as empty
    priority_edges = set()

    # If previous_edge is not None, find polygons containing it
    if previous_edge:
        polygons_with_prev_edge = [
            region for region in regions
            if previous_edge in [(edge.v_from, edge.v_to) for edge in region.edges]
        ]
        # Extract edges from these polygons
        priority_edges = set(
            (edge.v_from, edge.v_to) for region in polygons_with_prev_edge for edge in region.edges
        )

    for edge in intersected_edges:
        # Skip ignored edges
        if edge in ignored_edges:
            continue

        v1, v2 = np.array(edge[0]), np.array(edge[1])
        start = np.array(start_point)

        # Compute the closest distance from the point to the edge
        edge_vector = v2 - v1
        point_vector = start - v1
        edge_length = np.linalg.norm(edge_vector)

        if edge_length > epsilon:
            # Projection of the point onto the edge
            proj_length = np.dot(point_vector, edge_vector) / edge_length
            if 0 <= proj_length <= edge_length:
                # The projection falls within the edge segment
                projection = v1 + proj_length * edge_vector / edge_length
                distance_to_edge = np.linalg.norm(start - projection)
            else:
                # The projection falls outside the edge segment; use distance to the closest vertex
                distance_to_edge = min(np.linalg.norm(start - v1), np.linalg.norm(start - v2))
        else:
            # Handle degenerate edges (points)
            distance_to_edge = np.linalg.norm(start - v1)

        # Check if the edge is in the priority edges
        if edge in priority_edges:
            if distance_to_edge < min_priority_distance:
                min_priority_distance = distance_to_edge
                closest_priority_edge = edge
        else:
            if distance_to_edge < min_non_priority_distance:
                min_non_priority_distance = distance_to_edge
                closest_non_priority_edge = edge

    # Prioritize edges from polygons containing the previous edge
    return closest_priority_edge if closest_priority_edge is not None else closest_non_priority_edge


def is_line_entering_region(start_point, end_point, region):
    """ Determines if the line segment between start_point and end_point is inside the region using the winding number algorithm.

    :param start_point: Tuple (x, y) representing the start of the line segment.
    :param end_point: Tuple (x, y) representing the end of the line segment.
    :param region: A Polygon object representing the region.
    :return: True if the line segment is inside the region, False otherwise.
    """
    # Check the midpoint of the line segment
    midpoint = ((start_point[0] + end_point[0]) / 2, (start_point[1] + end_point[1]) / 2)

    return calculate_winding_number(midpoint, region.vertices)


def is_shorter_region_path(start_point, path_points, region, epsilon=1e-9):
    """
    Determines if a shorter path exists from the start_point to any of the points in the path_points list,
    excluding intersections with hard edges after filtering. If a shorter path is found, returns the list
    of points forming the shorter path.

    :param start_point: Tuple representing the starting point of the line segment (x, y).
    :param path_points: List of tuples representing the path points [(x1, y1), (x2, y2), ...].
    :param region: A Polygon object representing the region to check.
    :param epsilon: Float tolerance for precision issues.
    :return: Tuple (bool, list). The boolean indicates if a shorter path was found, and the list contains
             the points up to the shortest valid path point.
    """
    # Step 1: Validate and extract hard edges
    hard_edges = [
        ((edge.v_from.x, edge.v_from.y), (edge.v_to.x, edge.v_to.y))
        for edge in region.edges
        if edge.is_hard_edge
    ]

    # Step 2: Iterate through path points to find the earliest free path
    for i, end_point in enumerate(path_points):
        intersected_edges = []

        # Step 3: Check intersections with hard edges
        for edge_tuple in hard_edges:
            if line_intersection(start_point, end_point, edge_tuple[0], edge_tuple[1], epsilon):
                intersected_edges.append(edge_tuple)

        # Step 4: Filter intersected edges
        filtered_edges = filter_intersected_region_edges(start_point, end_point, intersected_edges, region, epsilon)

        # Step 5: Determine if this segment is valid
        if not filtered_edges:
            # Free path found, return the points forming the shorter path
            return True, path_points[:i + 1]  # Include the current point

    # No free path found
    return False, []


def find_adjacent_edges(region, hard_edge):
    """
    Finds the two edges connected to the left and right vertices of a given hard edge.

    :param region: A Polygon object representing the region.
    :param hard_edge: A tuple ((x1, y1), (x2, y2)) representing the hard edge.
    :return: A tuple (left_adjacent, right_adjacent), where:
             - left_adjacent is the edge connected to the start (left) vertex of the hard edge.
             - right_adjacent is the edge connected to the end (right) vertex of the hard edge.
    """
    left_vertex = hard_edge[0]
    right_vertex = hard_edge[1]

    left_adjacent = None
    right_adjacent = None

    for edge in region.edges:
        edge_tuple = ((edge.v_from.x, edge.v_from.y), (edge.v_to.x, edge.v_to.y))

        # Check for the edge connected to the left vertex
        if edge_tuple[1] == left_vertex and edge_tuple != hard_edge:
            left_adjacent = edge_tuple

        # Check for the edge connected to the right vertex
        if edge_tuple[0] == right_vertex and edge_tuple != hard_edge:
            right_adjacent = edge_tuple

        # Break early if both adjacent edges are found
        if left_adjacent and right_adjacent:
            break

    return left_adjacent, right_adjacent
