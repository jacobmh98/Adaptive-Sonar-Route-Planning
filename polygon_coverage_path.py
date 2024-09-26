import numpy as np
import matplotlib.pyplot as plt
import math
from Polygon import Polygon, Vertex
from matplotlib.patches import Patch

def get_b_mate(p, b):
    """ Find b's neighbour b_mate (counterclockwise neighbour)
    :param b: int, index of vertex b
    :param p: Polygon
    :return neighbour: int, index of b's neighbour vertex b_mate
    """
    n = len(p.vertices)

    if b == (n - 1):
        neighbour = 0
    else:
        neighbour = b + 1

    return neighbour

def get_previous_vertex(p, b):
    """ Find b's previous neighbour
    :param p: Polygon
    :param b: int, index of vertex b
    :return neighbour: int, index of b's previous neighbour
    """
    n = len(p.vertices)

    if b == 0:
        neighbour = n - 1
    else:
        neighbour = b - 1

    return neighbour

def create_vector(v1, v2):
    """ Computing a vector from vertex v1 to vertex v2
    :param v1: NumPy array, start point
    :param v2: NumPy array, end point
    :return vector:
    """
    return v2 - v1

def compute_offset_vector(v1,v2, sweep_direction, d):
    """ Compute a vector parallel to the vector from v1 to v2, but offset perpendicularly
    in the direction specified by sweep_direction.

    :param v1: NumPy array, the start point of the original vector
    :param v2: NumPy array, the end point of the original vector
    :param sweep_direction: int, the direction to offset the vector (+1 or -1)
    :param d: float, the distance by which the vector should be offset
    :return new_v1: NumPy array, the new starting point of the offset vector
    :return new_v2: NumPy array, the new ending point of the offset vector
    """
    # Find the perpendicular direction to the vector from v1 to v2
    vector = create_vector(v1,v2)
    perp_vector = np.array([-vector[1], vector[0]])

    # Normalize the perpendicular vector
    perp_vector_normalized = perp_vector / np.linalg.norm(perp_vector)

    # Offset the original vector by d in the direction of the perpendicular vector
    offset = perp_vector_normalized * d * sweep_direction

    # New starting and ending points for the offset vector
    new_v1 = v1 + offset
    new_v2 = v2 + offset

    return new_v1, new_v2

def compute_sweep_direction(v1, v2, a):
    """
    :param v1: NumPy array, the start point of the vector (b)
    :param v2: NumPy array, the end point of the vector (b_mate)
    :param a: NumPy array with 2D point coordinates
    :return: int, -1 or +1 to indicate sweep direction from original vector
    """
    # Calculate the original vector from b to b_mate
    vector_v1_v2 = create_vector(v1, v2)

    # Calculate the vector from b to a
    vector_v1_a = create_vector(v1, a)

    # Find the perpendicular direction from v1-v2 towards a
    perp_vector = np.array([-vector_v1_v2[1], vector_v1_v2[0]])

    # Normalize the perpendicular vector and calculate direction towards a
    perp_vector_normalized = perp_vector / np.linalg.norm(perp_vector)
    sweep_direction = np.sign(np.dot(perp_vector_normalized, vector_v1_a))  # -1 or +1 depending on which side a is

    return sweep_direction

def extend_vector_to_boundary(poly, v1, v2):
    """ Extend the vector from v1 to v2 to intersect with the boundary box

    :param poly: Polygon
    :param v1: NumPy array, the start point of the original vector
    :param v2: NumPy array, the end point of the original vector
    :return extended_v1: NumPy array, the new start point of the extended vector
    :return extended_v2: NumPy array, the new end point of the extended vector
    """
    # Extract the boundary box values
    min_x, max_x, min_y, max_y = poly.boundary

    # Calculate the direction vector from v1 to v2
    direction = create_vector(v1, v2)

    # Calculate the scaling factors needed to reach the boundary box
    if direction[0] != 0:  # Check for non-vertical lines
        t_min_x = (min_x - v1[0]) / direction[0]
        t_max_x = (max_x - v1[0]) / direction[0]
    else:
        # Vertical line case
        t_min_x = -np.inf  # No need to stretch horizontally
        t_max_x = np.inf

    if direction[1] != 0:  # Check for non-horizontal lines
        t_min_y = (min_y - v1[1]) / direction[1]
        t_max_y = (max_y - v1[1]) / direction[1]
    else:
        # Horizontal line case
        t_min_y = -np.inf  # No need to stretch vertically
        t_max_y = np.inf

    # Choose the correct scaling factor (the smallest positive value to reach the boundary)
    t_min = max(min(t_min_x, t_max_x), min(t_min_y, t_max_y))
    t_max = min(max(t_min_x, t_max_x), max(t_min_y, t_max_y))

    # If t_min or t_max are infinite, it means there is no intersection
    if np.isinf(t_min):
        extended_v1 = v1  # No extension needed
    else:
        extended_v1 = v1 + direction * t_min

    if np.isinf(t_max):
        extended_v2 = v2  # No extension needed
    else:
        extended_v2 = v1 + direction * t_max

    return extended_v1, extended_v2

def distance_between_points(v1, v2):
    """ Calculate the Euclidean distance between two 2D points.

    :param v1: NumPy array (2D point)
    :param v2: NumPy array (2D point)
    :return distance: float, the distance between the two points.
    """
    return math.sqrt((v2[0] - v1[0]) ** 2 + (v2[1] - v1[1]) ** 2)

def check_and_connect(path, p1, p2, b):
    """ Adds points to path in correct order

    :param path: List of points, containing the current path
    :param p1: Point one
    :param p2: Point two
    :param b: Point b, only used incase of empty path
    :return path: List of points with p1 and p2 added in correct order
    """
    n = len(path)-1

    if n < 0:
        # Start path close to start point b
        if distance_between_points(b, p1) <= distance_between_points(b, p2):
            path.append(p1)
            path.append(p2)
        else:
            path.append(p2)
            path.append(p1)
    else:
        # Continue path at next closest point
        if distance_between_points(path[n], p1) <= distance_between_points(path[n], p2):
            path.append(p1)
            path.append(p2)
        else:
            path.append(p2)
            path.append(p1)

    return path

# Algorithm 1
def get_path(poly, fp, dx, ps, pe, b_index, b_mate_index, a_index):
    """ GetPath algorithm from page 5 in Coverage Path Planning for 2D Convex Regions

    :param poly: Polygon P, using Polygon class
    :param fp: bool
    :param dx: Path width (0.1 = 100 meters)
    :param ps: Path starting point (Not a polygon vertex)
    :param pe: Path ending point (Not a polygon vertex)
    :param b_index: Starting vertex index
    :param b_mate_index: b's counterclockwise neighbour index
    :param a_index: b's diametral antipodal point index
    :return path: A path which fully covers P
    """
    # Getting the three vertices as points from the polygon
    b = poly.vertices[b_index].v
    b_mate = poly.vertices[b_mate_index].v
    a = poly.vertices[a_index].v
    #print(f'b = {b}')
    #print(f'b_mate = {b_mate}')
    #print(f'a = {a}')

    # Polygon diameter is distance from b to a
    diameter = distance_between_points(b, a)

    # Finding direction from vector b, b_mate towards a (+1 or -1)
    sweep_direction = compute_sweep_direction(b, b_mate, a)

    # First pass is offset from polygon edge with half path width
    if dx > diameter:
        # In case of path width being too large to fit inside polygon, changed to fit using polygon diameter
        delta_init = diameter / 2
    else:
        delta_init = dx / 2

    # Computing vector b to b_mate
    # L_flight = create_vector(b, b_mate)
    # Offsetting vector b to b_mate with delta_init towards point a
    L_flight = compute_offset_vector(b, b_mate, sweep_direction, delta_init)
    # Extending the offset vector to polygon boundaries to find all intersection points with poly edge (2 points)
    L_flight = extend_vector_to_boundary(poly, L_flight[0], L_flight[1])

    # Initializing the path with the starting point
    if fp:
        path = []
    else:
        path = []
    found_path = True

    # Fail-safe parameters for the while loop
    max_iterations = 10000
    counter = 0

    # Loop until no intersections is found
    while not (Polygon.find_intersections(poly, L_flight[0], L_flight[1]) == []):  # Optimize by only calling function once?
        ip1, ip2 = Polygon.find_intersections(poly, L_flight[0], L_flight[1])

        # Add points to path in correct order
        path = check_and_connect(path, ip1, ip2, b)

        # Plot for checking every iteration of the path
        #plot_path(b, b_mate, a, dx, boundaries, poly, np.array(path))

        # Computing next extended offset vector, offset with full path width dx
        L_flight = compute_offset_vector(L_flight[0], L_flight[1], sweep_direction, dx)
        L_flight = extend_vector_to_boundary(poly, L_flight[0], L_flight[1])

        if counter >= max_iterations:
            print(f"Max iterations of {max_iterations} reached.")
            found_path = False
            break
        counter += 1

        #print(np.array(path))

    #if found_path:
        #print(f"Computed path after {counter} iterations.")
    return np.array(path)

def best_path(polygon, fp, i, j, ps, pe, dx):
    """
    Compute the best back-and-forth path between antipodal points i and j.

    Args:
        polygon: Polygon object representing the region.
        fp: bool
        i: Index of the first antipodal vertex.
        j: Index of the second antipodal vertex.
        ps: Starting point
        pe: Ending point
        dx: Path width

    Returns:
        Tuple (optimal_path, min_cost):
            - optimal_path: The optimal back-and-forth path (computed using get_path).
            - min_cost: The total travel distance for the optimal path.
    """
    # (Triangle edge case) If b_mate and a are the same point, change a to last remaining vertex in the triangle polygon
    if polygon.vertices[i].next.index == j:
        a = polygon.vertices[get_previous_vertex(polygon, i)].index
    else:
        a = j

    # Compute the first back-and-forth path from i to j using get_path
    path1 = get_path(polygon, fp, dx, ps, pe, i, polygon.vertices[i].next.index, a)

    # Calculate the cost for path1
    cost1 = calculate_total_cost(path1, ps, pe)

    #plot_path(polygon.vertices[i].v, polygon.vertices[i].next.v, polygon.vertices[a].v, dx, boundaries, polygon, path1)
    #print(f'Path from {i} to {j} (Cost: {cost1:.2f})')

    # (Triangle edge case) If b_mate and a are the same point, change a to last remaining vertex in the triangle polygon
    if polygon.vertices[j].next.index == i:
        a = polygon.vertices[get_previous_vertex(polygon, j)].index
    else:
        a = i

    # Compute the second back-and-forth path from j to i using get_path
    path2 = get_path(polygon, fp, dx, ps, pe, j, polygon.vertices[j].next.index, a)

    # Calculate the cost for path2
    cost2 = calculate_total_cost(path2, ps, pe)

    #plot_path(polygon.vertices[j].v, polygon.vertices[j].next.v, polygon.vertices[a].v, dx, boundaries, polygon, path2)
    #print(f'Path from {j} to {i} (Cost: {cost2:.2f})')

    # Compare costs and return the path with the lowest total cost
    if cost1 < cost2:
        return path1, cost1
    else:
        return path2, cost2

def calculate_total_cost(path, start_point, end_point):
    """
    Calculate the total cost of a given path, including the travel distance to
    the starting point and from the ending point.

    Args:
        path: The back-and-forth path (array of waypoints or segments).
        start_point: Starting point of the mission (2D tuple or numpy array).
        end_point: Ending point of the mission (2D tuple or numpy array).

    Returns:
        Total travel distance (float).
    """
    # 1. Calculate the distance from the start_point to the first point of the path
    distance_to_start = np.linalg.norm(np.array(start_point) - np.array(path[0]))

    # 2. Calculate the distance for the entire back-and-forth path (sum of segment distances)
    path_distance = 0
    for k in range(1, len(path)):
        path_distance += np.linalg.norm(np.array(path[k]) - np.array(path[k - 1]))

    # 3. Calculate the distance from the last point of the path to the end_point
    distance_to_end = np.linalg.norm(np.array(end_point) - np.array(path[-1]))

    # 4. Total cost is the sum of all distances
    total_cost = distance_to_start + path_distance + distance_to_end
    return total_cost

def plot_path(b, b_mate, a, dx, boundaries, polygon, path, ps=None, pe=None):
    """
    Plot the original vector from b to b_mate, the offset vector, the boundary box, the polygon,
    including the intersection points between the offset vector and the polygon, and the path points.
    Also, plot the coverage area around the path, and the start/end points of the mission.

    :param b: Start point of the original vector
    :param b_mate: End point of the original vector
    :param a: Diametric point of b, the direction towards which the vector should be offset
    :param dx: float, the distance by which the vector should be offset (this defines the width of coverage)
    :param boundaries: array, representing the boundary box [min_x, max_x, min_y, max_y]
    :param polygon: Polygon, the polygon object to be plotted
    :param path: NumPy array, the array of points representing the path [[x1, y1], [x2, y2], ...]
    :param ps: Starting point of the mission (optional)
    :param pe: Ending point of the mission (optional)
    """

    # Plot options
    show_points = True
    show_polygon = True
    show_start_vectors = True
    show_boundary_box = True
    show_path = True
    show_start_end_path_points = True
    show_coverage_area = True
    show_start_end_mission_points = True

    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    # Plot the boundary box
    min_x, max_x, min_y, max_y = boundaries
    if show_boundary_box:
        ax.plot([min_x, max_x, max_x, min_x, min_x], [min_y, min_y, max_y, max_y, min_y], 'k--', label='Boundary Box')

    # Plot the points b, b_mate, and a
    if show_points:
        ax.plot([b[0], b_mate[0], a[0]], [b[1], b_mate[1], a[1]], 'ro', label='Points b, b_mate, a', markersize=10)
        ax.text(b[0], b[1], f'b', fontsize=16, color='darkblue')
        ax.text(b_mate[0], b_mate[1], 'b_mate', fontsize=16, color='darkblue')
        ax.text(a[0], a[1], "a", fontsize=16, color='darkblue')

    # Plot the polygon
    x_coords, y_coords = polygon.get_coords()
    if show_polygon:
        ax.plot(x_coords, y_coords, 'b-', marker='o', label='Polygon')
        ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'b-')

    # Plot the path
    if len(path) > 0:
        path_x, path_y = path[:, 0], path[:, 1]
        if show_path:
            ax.plot(path_x, path_y, 'g-', marker='x', label='Path', linewidth=3)

        # Highlight the start and end points of the path
        if show_start_end_path_points:
            ax.plot(path_x[0], path_y[0], 'go', markersize=10, label='Start Point')  # Start point
            ax.plot(path_x[-1], path_y[-1], 'yo', markersize=10, label='End Point')  # End point

        # Compute and plot coverage area along the path
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]
            # Vector along the path segment
            segment_vector = p2 - p1
            # Normalize the vector to get the perpendicular direction
            perp_vector = np.array([-segment_vector[1], segment_vector[0]])
            if np.linalg.norm(perp_vector) != 0:
                perp_vector = perp_vector / np.linalg.norm(perp_vector) * dx / 2

            # Create four corners of the coverage area for this segment
            corner1 = p1 + perp_vector
            corner2 = p1 - perp_vector
            corner3 = p2 - perp_vector
            corner4 = p2 + perp_vector

            # Plot the coverage area for this segment as a filled polygon
            if show_coverage_area:
                ax.fill([corner1[0], corner2[0], corner3[0], corner4[0]],
                        [corner1[1], corner2[1], corner3[1], corner4[1]],
                        'orange', alpha=0.3, label='_nolegend_')

    else:
        print("Empty path")

    # Plot the mission start and end points
    if ps is not None and show_start_end_mission_points:
        ax.plot(ps[0], ps[1], 'bo', markersize=12, label='Mission Start Point')
        ax.text(ps[0], ps[1], 'Start', fontsize=14, color='blue')

    if pe is not None and show_start_end_mission_points:
        ax.plot(pe[0], pe[1], 'mo', markersize=12, label='Mission End Point')
        ax.text(pe[0], pe[1], 'End', fontsize=14, color='magenta')

    # Used to add coverage in legend as square
    coverage_patch = Patch(color='orange', label='Coverage Area', alpha=0.3)

    # Get handles and labels from the plot, and add the custom coverage patch
    handles, labels = ax.get_legend_handles_labels()
    handles.append(coverage_patch)
    labels.append('Covered Area')

    if show_start_vectors or show_points or show_path or show_polygon or show_start_end_path_points or show_boundary_box or show_coverage_area:
        ax.legend(handles=handles, labels=labels, loc='best')

    plt.grid(True)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()