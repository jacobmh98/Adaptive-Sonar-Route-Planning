import numpy as np
import matplotlib.pyplot as plt
import math
from Polygon import Polygon, Vertex
from matplotlib.patches import Patch
from global_variables import *

def triangle_antipodal_edge_case(poly, mate_index, antipodal_index):
    """
    :param poly: Polygon
    :param mate_index: Index of point's mate in the polygon (b_mate)
    :param antipodal_index: Index of points's antipodal point (a)
    :return: index of a
    """
    # If b_mate index and a_index are the same point, change point a to last remaining vertex in the triangle polygon
    if mate_index == antipodal_index:
        return poly.get_mate(mate_index)
    else:
        return antipodal_index

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

def compute_boundary(poly):
    coords = poly.vertices_matrix()
    boundary = (np.min(coords[0, :]), np.max(coords[0, :]),
                     np.min(coords[1, :]), np.max(coords[1, :]))
    return boundary

def extend_vector_to_boundary(v1, v2, boundary):
    """ Extend the vector from v1 to v2 to intersect with the boundary box

    :param v1: NumPy array, the start point of the original vector
    :param v2: NumPy array, the end point of the original vector
    :param boundary: List of polygon's boundaries
    :return extended_v1: NumPy array, the new start point of the extended vector
    :return extended_v2: NumPy array, the new end point of the extended vector
    """
    # Extract the boundary box values
    min_x, max_x, min_y, max_y = boundary

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
    :param b: Point b, only used in case of empty path
    :return path: List of points with p1 and p2 added in correct order
    """
    n = len(path)-1

    if n < 0:
        # Start path offset from start point b
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

def closest_vertex(poly1, poly2, a_index):
    """
    Find the closest point in poly2 to the vertex 'a' in poly1.

    :param poly1: Polygon object (Polygon 1)
    :param poly2: Polygon object (Polygon 2)
    :param a_index: int, index of the vertex 'a' in poly1
    :return: NumPy array (1D) representing the closest point [x, y] in poly2
    """
    # Get the vertex 'a' from poly1
    vertex_a = poly1.vertices[a_index].v.flatten()  # Convert to 1D array [x, y]

    # Initialize the minimum distance to a large value
    min_distance = float('inf')
    closest_point = None

    # Iterate over each vertex in poly2 and calculate the distance to vertex 'a'
    for v in poly2.vertices:
        vertex_b = v.v.flatten()  # Convert to 1D array [x, y]
        distance = np.linalg.norm(vertex_a - vertex_b)  # Euclidean distance

        # Update the closest point if a smaller distance is found
        if distance < min_distance:
            min_distance = distance
            closest_point = vertex_b

    return closest_point

def get_path(poly, dx, b_index, b_mate_index, a_index, boundary):
    """ # Algorithm 1 - GetPath algorithm from page 5 in Coverage Path Planning for 2D Convex Regions

    :param poly: Polygon P, using Polygon class
    :param dx: Path width (0.1 = 100 meters)
    :param b_index: Starting vertex index
    :param b_mate_index: b's counterclockwise neighbour index
    :param a_index: b's diametral antipodal point index
    :param boundary: List of polygon's boundaries
    :return path: A path which fully covers P
    """
    # Getting the three vertices as points from the polygon
    b = poly.vertices[b_index].v.flatten()
    b_mate = poly.vertices[b_mate_index].v.flatten()
    a = poly.vertices[a_index].v.flatten()

    path = []

    # Finding direction from vector b, b_mate towards a (+1 or -1)
    sweep_direction = compute_sweep_direction(b, b_mate, a)

    # First pass is offset from polygon edge with half path width, unless path width is too large for the polygon
    b_a_dist = distance_between_points(b, a)
    b_mate_a_dist = distance_between_points(b_mate, a)

    # Find polygon smallest diameter
    if b_a_dist < b_mate_a_dist:
        diameter = b_a_dist
    else:
        diameter = b_mate_a_dist

    # If path width is larger than the smallest diameter, then change path to half the diameter size
    if dx > diameter:
        # In case of path width being too large to fit inside polygon
        delta_init = diameter / 2
    else:
        delta_init = dx / 2

    # Computing vector b to b_mate
    # L_flight = create_vector(b, b_mate)
    # Offsetting vector b to b_mate with delta_init towards point a
    L_flight = compute_offset_vector(b, b_mate, sweep_direction, delta_init)
    # Extending the offset vector to polygon boundaries to find all intersection points with poly edge (2 points)
    L_flight = extend_vector_to_boundary(L_flight[0], L_flight[1], boundary)

    # Fail-safe parameters for the while loop
    max_iterations = 10000
    counter = 0

    # Loop until no intersections is found
    # TODO: Change loop such that last turn checks to create a closer path to poly edge
    while not (Polygon.find_intersections(poly, L_flight[0], L_flight[1]) == []):
        ip1, ip2 = Polygon.find_intersections(poly, L_flight[0], L_flight[1])

        # Add points to path in correct order
        path = check_and_connect(path, ip1, ip2, b)

        # Plot for checking every iteration of the path
        #plot_path(poly, b, b_mate, a, dx, np.array(path))

        # Ensure coverage gets as close to polygon edge as needed for full coverage
        """  TODO: Need to create function to check full coverage, else best coverage path wont get chosen as it is longer than path without full coverage
        if (distance_between_points(a, ip1) <= dx) or (distance_between_points(a, ip2) <= dx):
            if go_to_edge:  # Only divide once per poly, to avoid never ending loop
                dx = dx/2
                go_to_edge = False
        """

        # Computing next extended offset vector, offset with full path width dx
        L_flight = compute_offset_vector(L_flight[0], L_flight[1], sweep_direction, dx)
        L_flight = extend_vector_to_boundary(L_flight[0], L_flight[1], boundary)

        # Avoid infinite while loop
        if counter >= max_iterations:
            print(f"Max iterations of {max_iterations} reached.")
            break
        counter += 1

    return np.array(path)

def best_path(polygons, current_polygon_index, path, dx, i, j):
    """ Compute the best path based on both distance and number of turns.

    :param path:
    :param polygons: List of polygons
    :param current_polygon_index:
    :param dx:
    :param i:
    :param j:
    :return:
    """
    poly = polygons[current_polygon_index]
    boundary = compute_boundary(poly)
    n = len(polygons)

    # Setting start point as the current last point in the total path
    if path.shape[0] > 0:  # if ext points included, ext_p_start will be the last (only) point in the path
        path_start = path[path.shape[0]-1]
    else:  # If first polygon and no ext points, then the first point in the first path will act as the start point
        path_start = None

    # TODO: Use distance from b to a metric to avoid computing all 4 paths, might be difficult due to start and end points depends on other polygons
    # From b towards a
    # Clockwise caliper
    cw_b1 = i
    cw_b1_mate = poly.get_mate(cw_b1)
    cw_a1 = triangle_antipodal_edge_case(poly, cw_b1_mate, j)
    cw_path1 = get_path(poly, dx, cw_b1, cw_b1_mate, cw_a1, boundary)

    # Finding the closest vertex in the next polygon from path aim point a
    if current_polygon_index < (n-1):
        cw_p_end1 = closest_vertex(poly, polygons[current_polygon_index+1], cw_a1)
    else:
        cw_p_end1 = poly.vertices[cw_a1].v  # Else last point will be point a

    # Counterclockwise
    ccw_b1 = cw_b1_mate
    ccw_b1_mate = i
    ccw_a1 = cw_a1  # Unchanged
    ccw_path1 = get_path(poly, dx, ccw_b1, ccw_b1_mate, ccw_a1, boundary)
    if current_polygon_index < (n-1):
        ccw_p_end1 = closest_vertex(poly, polygons[current_polygon_index+1], ccw_a1)  # Finding the closest vertex in next polygon from path endpoint a
    else:
        ccw_p_end1 = poly.vertices[ccw_a1].v  # Else last point will be point a
    #plot_paths_comparison(poly, p_start, poly.vertices[cw_b1].v, poly.vertices[cw_b1_mate].v, poly.vertices[cw_a1].v, cw_path1, poly.vertices[ccw_b1].v, poly.vertices[ccw_b1_mate].v, poly.vertices[ccw_a1].v, ccw_path1,dx,(optimal_path1, optimal_dist1, poly.vertices[b1_best].v, poly.vertices[b1_mate_best].v, poly.vertices[a1_best].v),cw_dist1, ccw_dist1)

    # From a towards b
    # Clockwise caliper
    cw_b2 = j
    cw_b2_mate = poly.get_mate(cw_b2)
    cw_a2 = triangle_antipodal_edge_case(poly, cw_b2_mate, i)
    cw_path2 = get_path(poly, dx, cw_b2, cw_b2_mate, cw_a2, boundary)
    if current_polygon_index < (n-1):
        cw_p_end2 = closest_vertex(poly, polygons[current_polygon_index+1], cw_a2)  # Finding the closest vertex in next polygon from path endpoint a
    else:
        cw_p_end2 = poly.vertices[cw_a2].v  # Else last point will be point a

    # Counterclockwise
    ccw_b2 = cw_b2_mate
    ccw_b2_mate = j
    ccw_a2 = cw_a2  # Unchanged
    ccw_path2 = get_path(poly, dx, ccw_b2, ccw_b2_mate, ccw_a2, boundary)
    if current_polygon_index < (n-1):
        ccw_p_end2 = closest_vertex(poly, polygons[current_polygon_index+1], ccw_a2)  # Finding the closest vertex in next polygon from path endpoint a
    else:
        ccw_p_end2 = poly.vertices[cw_a1].v  # Else last point will be point a

    # Calculating path scores
    cw_score1 = calculate_path_score(cw_path1, path_start, cw_p_end1)
    ccw_score1 = calculate_path_score(ccw_path1, path_start, ccw_p_end1)
    cw_score2 = calculate_path_score(cw_path2, path_start, cw_p_end2)
    ccw_score2 = calculate_path_score(ccw_path2, path_start, ccw_p_end2)

    # Select the optimal rotation from the shortest path distance for path1
    if cw_score1 < ccw_score1:
        optimal_path1, optimal_score1 = cw_path1, cw_score1
        b1_best, b1_mate_best, a1_best = cw_b1, cw_b1_mate, cw_a1
    else:
        optimal_path1, optimal_score1 = ccw_path1, ccw_score1
        b1_best, b1_mate_best, a1_best = ccw_b1, ccw_b1_mate, ccw_a1

    # Select the optimal rotation from the shortest path distance for path2
    if cw_score2 < ccw_score2:
        optimal_path2, optimal_score2 = cw_path2, cw_score2
        b2_best, b2_mate_best, a2_best = cw_b2, cw_b2_mate, cw_a2
    else:
        optimal_path2, optimal_score2 = ccw_path2, ccw_score2
        b2_best, b2_mate_best, a2_best = ccw_b2, ccw_b2_mate, ccw_a2

    #plot_paths_comparison(poly, p_start, poly.vertices[cw_b2].v, poly.vertices[cw_b2_mate].v, poly.vertices[cw_a2].v, cw_path2, poly.vertices[ccw_b2].v, poly.vertices[ccw_b2_mate].v, poly.vertices[ccw_a2].v, ccw_path2,dx,(optimal_path2, optimal_dist2, poly.vertices[b2_best].v, poly.vertices[b2_mate_best].v, poly.vertices[a2_best].v),cw_dist2, ccw_dist2)

    # Selecting the optimal path
    if optimal_score1 < optimal_score2:
        optimal_path, optimal_score = optimal_path1, optimal_score1
        b_best, b_mate_best, a_best = b1_best, b1_mate_best, a1_best  # For plot
    else:
        optimal_path, optimal_score = optimal_path2, optimal_score2
        b_best, b_mate_best, a_best = b2_best, b2_mate_best, a2_best  # For plot

    #plot_paths_comparison(poly, p_start, poly.vertices[b1_best].v, poly.vertices[b1_mate_best].v, poly.vertices[a1_best].v, optimal_path1, poly.vertices[b2_best].v, poly.vertices[b2_mate_best].v, poly.vertices[a2_best].v, optimal_path2,dx,(optimal_path, optimal_score, poly.vertices[b_best].v, poly.vertices[b_mate_best].v, poly.vertices[a_best].v),score1, score2)

    return optimal_path, optimal_score

def calculate_path_score(path, path_start, path_end):
    """ Calculate the total cost of a given path, including the travel distance to
    the starting point and from the ending point.

    :param path: A list of points creating the given path
    :param path_start:
    :param path_end:
    :return: Paths score
    """
    if path.size == 0:
        return 0

    if path_start is None:
        distance_to_start = 0
    else:
        distance_to_start = distance_to_start_weight * np.linalg.norm(np.array(path_start) - np.array(path[0]))

    # 2. Calculate the distance for the entire back-and-forth path (sum of segment distances)
    path_distance = 0
    for k in range(1, len(path)):
        path_distance += np.linalg.norm(np.array(path[k]) - np.array(path[k - 1]))
    path_distance *= distance_weight

    # 3. Calculate the distance from the last point of the path to the end_point
    distance_to_end = distance_to_end_weight *  np.linalg.norm(np.array(path_end) - np.array(path[-1]))  # TODO: Need next poly to get vertex closest to optimal end point of current poly

    # 3.1. Account for number of turns in path (points)
    turns = turn_weight * len(path)

    # 4. The final path score heuristic
    score = distance_to_start + path_distance + distance_to_end + turns

    return score

def plot_path(poly, b, b_mate, a, dx, path):
    """
    Plot the original vector from b to b_mate, the offset vector, the boundary box, the polygon,
    including the intersection points between the offset vector and the polygon, and the path points.
    Also, plot the coverage area around the path, and the start/end points of the mission.

    :param poly: Polygon, the polygon object to be plotted
    :param b: Start point of the original vector
    :param b_mate: End point of the original vector
    :param a: Diametric point of b, the direction towards which the vector should be offset
    :param dx: float, the distance by which the vector should be offset (this defines the width of coverage)
    :param path: NumPy array, the array of points representing the path [[x1, y1], [x2, y2], ...]
    """

    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    # Plot the boundary box
    min_x, max_x, min_y, max_y = poly.get_boundary()
    ax.plot([min_x, max_x, max_x, min_x, min_x], [min_y, min_y, max_y, max_y, min_y], 'k--', label='Boundary Box')

    # Plot the points b, b_mate, and a
    ax.plot([b[0], b_mate[0], a[0]], [b[1], b_mate[1], a[1]], 'ro', label='Points b, b_mate, a', markersize=10)
    ax.text(b[0], b[1], f'b', fontsize=16, color='darkblue')
    ax.text(b_mate[0], b_mate[1], 'b_mate', fontsize=16, color='darkblue')
    ax.text(a[0], a[1], "a", fontsize=16, color='darkblue')

    # Plot the polygon
    x_coords, y_coords = poly.get_coords()
    ax.plot(x_coords, y_coords, 'b-', marker='o', label='Polygon')
    ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'b-')

    # Plot the path
    if len(path) > 0:
        path_x, path_y = path[:, 0], path[:, 1]
        ax.plot(path_x, path_y, 'g-', marker='x', label='Path', linewidth=3)

        # Highlight the start and end points of the path
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
            ax.fill([corner1[0], corner2[0], corner3[0], corner4[0]],
                    [corner1[1], corner2[1], corner3[1], corner4[1]],
                    'orange', alpha=0.3, label='_nolegend_')

    else:
        print("Empty path")

    # Used to add coverage in legend as square
    coverage_patch = Patch(color='orange', label='Coverage Area', alpha=0.3)

    # Get handles and labels from the plot, and add the custom coverage patch
    handles, labels = ax.get_legend_handles_labels()
    handles.append(coverage_patch)
    labels.append('Covered Area')

    ax.legend(handles=handles, labels=labels, loc='best')

    plt.grid(True)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()


def plot_paths_comparison(poly, p_start, b1, b_mate1, a1, path1, b2, b_mate2, a2, path2, dx, best_path_output, score1,
                          score2):
    """
    Plot the two back-and-forth paths (path1 and path2) in separate subplots for comparison,
    and show the best path in a third subplot.

    :param poly: Polygon, the polygon object to be plotted
    :param b1, b2: Start points of the original vectors for path1 and path2
    :param b_mate1, b_mate2: End points of the original vectors for path1 and path2
    :param a1, a2: Diametric points for path1 and path2
    :param path1, path2: NumPy arrays, the arrays of points representing the paths
    :param dx: float, the distance by which the vectors should be offset (defining coverage width)
    :param best_path_output: Tuple containing the best path and its distance
    :param distance1, distance2: The distances for path1 and path2
    """

    # Unpack the best path information from the output of best_path function
    optimal_path, optimal_distance, b_best, b_mate_best, a_best = best_path_output

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(12, 4))
    fig.suptitle('Comparison of Path 1, Path 2, and Best Path', fontsize=16)

    # Plot Path 1
    plot_single_path(ax1, poly, b1, b_mate1, a1, dx, path1, title=f"Distance = {score1:.2f}")

    # Plot Path 2
    plot_single_path(ax2, poly, b2, b_mate2, a2, dx, path2, title=f"Distance = {score2:.2f}")

    # Plot the Best Path in the third subplot
    plot_single_path(ax3, poly, b_best, b_mate_best, a_best, dx, optimal_path, title=f"Best Path")

    plt.tight_layout()
    plt.show()


def plot_single_path(ax, poly, b, b_mate, a, dx, path, title):
    """
    Helper function to plot a single path in a given axis, with the distance displayed as text.
    """
    ax.set_aspect('equal')
    ax.set_title(title)

    # Plot the boundary box
    min_x, max_x, min_y, max_y = poly.get_boundary()
    ax.plot([min_x, max_x, max_x, min_x, min_x], [min_y, min_y, max_y, max_y, min_y], 'k--', label='Boundary Box')

    # Plot the points b, b_mate, and a
    ax.plot([b[0], b_mate[0], a[0]], [b[1], b_mate[1], a[1]], 'ro', label='Points b, b_mate, a', markersize=10)
    ax.text(b[0], b[1], f'b', fontsize=12, color='darkblue')
    ax.text(b_mate[0], b_mate[1], 'b_mate', fontsize=12, color='darkblue')
    ax.text(a[0], a[1], "a", fontsize=12, color='darkblue')

    # Plot the polygon
    x_coords, y_coords = poly.get_coords()
    ax.plot(x_coords, y_coords, 'b-', marker='o', label='Polygon')
    ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'b-')

    # Plot the path
    if len(path) > 0:
        path_x, path_y = path[:, 0], path[:, 1]
        ax.plot(path_x, path_y, 'g-', marker='x', label='Path', linewidth=3)

        # Highlight the start and end points of the path
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
            ax.fill([corner1[0], corner2[0], corner3[0], corner4[0]],
                    [corner1[1], corner2[1], corner3[1], corner4[1]],
                    'orange', alpha=0.3, label='_nolegend_')

    else:
        print("Empty path")

    # Add legend for coverage area
    #ax.legend(loc='best')

    ax.grid(True)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')