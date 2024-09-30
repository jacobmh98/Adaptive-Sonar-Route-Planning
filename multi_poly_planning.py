import numpy as np
import polygon_coverage_path
import rotating_calipers_antipodal_pairs
import matplotlib.pyplot as plt



def get_nearest_neighbour_vertex(poly1, poly2):
    nearest_vertex = None
    min_distance = float('inf')

    # Loop over all vertices in poly1 and poly2
    for v1 in poly1.vertices:
        for v2 in poly2.vertices:
            # Calculate the Euclidean distance between the two vertices
            distance = np.linalg.norm(v1.v - v2.v)

            # Check if this is the closest vertex found so far
            if distance < min_distance:
                min_distance = distance
                nearest_vertex = v2  # Nearest vertex in poly2 is a shared vertex with poly1

    return nearest_vertex

def rotating_calipers_path_planner(polygons, current_polygon_index, dx, d_pq, p_start, p_end):
    """ Algorithm 2: Rotating Calipers Path Planner.
    Computes the optimal back-and-forth path that covers a convex polygon efficiently by testing all antipodal pairs.

    :param polygons: List Polygon
    :param current_polygon_index: index of current polygon
    :param dx: float, the path width
    :param d_pq: List of tuples representing antipodal pairs (b, a).
    :param p_start: Start point of path
    :param p_end: End point of path
    :return optimal_path: The best back-and-forth path (computed using best_path).
    """
    # Initialize variables to store the best path and the minimal cost
    min_cost = float('inf')
    optimal_path = None

    # Iterate over all antipodal pairs (b, a)
    for (i, j) in d_pq:
        # Compute the best path for the current antipodal pair
        current_path, current_cost = polygon_coverage_path.best_path(polygons, current_polygon_index, dx, i, j, p_start, p_end)

        # Update the optimal path if the current path has a lower cost
        if current_cost < min_cost:
            min_cost = current_cost
            optimal_path = current_path

    return optimal_path

def multi_path_planning(polygons, dx, include_external_start_end, ext_p_start, ext_p_end):
    """
    :param polygons: List of Polygons
    :param dx: float, path width
    :param include_external_start_end: Bool, indicate if external start and end point included
    :param ext_p_start: External start point (None if include_external_start_end is False)
    :param ext_p_end: External end point (None if include_external_start_end is False)
    :return:
    """
    # Creating the np array to store the total path
    total_path = np.empty((0,2))

    if include_external_start_end:
        # Appending external start point to path
        total_path = np.append(total_path, [ext_p_start], axis=0)

    for i, current_poly in enumerate(polygons):
        b_index = 0  # Always start at first vertex in the current polygon

        # Computing current polygons antipodal points
        antipodal_vertices = rotating_calipers_antipodal_pairs.compute_antipodal_pairs(current_poly)
        # Removes neighbour pairs and double pairs, i.e. for [0,1] and [1,0] only 1 of them is necessary as both [1,0] and [0,1] is checked in best path algorithm
        filtered_antipodal_vertices = rotating_calipers_antipodal_pairs.filter_and_remove_redundant_pairs(current_poly, antipodal_vertices)
        # Computing the diametric antipodal pairs (Minimizing number of paths computed, as diametric pairs produce the shortest paths)
        diametric_antipodal_pairs = rotating_calipers_antipodal_pairs.filter_diametric_antipodal_pairs(current_poly, filtered_antipodal_vertices)
        # Getting index of point a (the diametric antipodal point of b)
        diametric_antipode_index = rotating_calipers_antipodal_pairs.get_diametric_antipodal_point_index(diametric_antipodal_pairs, b_index)

        # Setting start point as the current last point in the total path
        if total_path.shape[0] > 0:  # if ext points included, ext_p_start will be the last (only) point in the path
            new_p_start = total_path[total_path.shape[0]-1]
        else:  # If first polygon and no ext points, then the first point in each path will be start point
            new_p_start = None

        # Setting end point to be the closest point in the next poly
        if i < len(polygons) - 1:
            new_p_end = get_nearest_neighbour_vertex(current_poly, polygons[i+1]).v  # TODO: Might not always create the optimal path
        else: # On the last polygon in the list
            if include_external_start_end:
                new_p_end = ext_p_end
            else:
                # Last point in the current path will be the total path endpoint
                new_p_end = current_poly.vertices[diametric_antipode_index].v

        shortest_path = rotating_calipers_path_planner(polygons, i, dx, diametric_antipodal_pairs, new_p_start, new_p_end)

        total_path = np.vstack([total_path, shortest_path])

    # Appending the last point into path
    if include_external_start_end:
        total_path = np.append(total_path, [ext_p_end], axis=0)

    return total_path


def multi_poly_plot(polygon, dx, include_external_start_end, ps, pe, path):
    """
    Plot multiple polygons, the path between the polygons, and the start/end points of the mission.

    :param include_external_start_end: bool
    :param path: NumPy array, array of points representing the path [[x1, y1], [x2, y2], ...]
    :param polygons: List of Polygon objects to be plotted
    :param ps: Starting point of the mission
    :param pe: Ending point of the mission
    :param dx: float, the distance by which the vector should be offset (this defines the width of coverage)
    """
    coverage = False
    fig, ax = plt.subplots(1, 1)
    color = "k"
    # Plot the polygons
    x_coords, y_coords = polygon.get_coords()

    ax.plot(x_coords, y_coords, f'{color}-', marker='o')
    ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], f'{color}-')
    ax.set_aspect('equal')


    """
    for i, poly in enumerate(polygons):
        x_coords, y_coords = poly.get_coords()
        ax.plot(x_coords, y_coords, 'k-', marker='o',markersize=3, label='Polygon')
        ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'k-')

        # Find the center of the polygon to place the label
        centroid_x = np.mean(x_coords)
        centroid_y = np.mean(y_coords)

        # Label the polygon with its number
        #ax.text(centroid_x, centroid_y, f'{i}', fontsize=26, color='black')
    """
    # Plot the path
    if len(path) > 0:
        path_x, path_y = path[:, 0], path[:, 1]
        ax.plot(path_x, path_y, 'r-', label='Path', linewidth=1)

        # Highlight the start and end points of the path
        ax.plot(path_x[0], path_y[0], 'go', markersize=8, label='Start Point')  # Start point
        ax.plot(path_x[-1], path_y[-1], 'yo', markersize=8, label='End Point')  # End point

        # Compute and plot coverage area along the path
        if coverage:
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

    # Plot the mission start and end points
    if include_external_start_end:
        ax.plot(ps[0], ps[1], 'bo', markersize=12, label='Mission Start Point')
        ax.text(ps[0], ps[1], 'Start', fontsize=14, color='blue')

        ax.plot(pe[0], pe[1], 'mo', markersize=12, label='Mission End Point')
        ax.text(pe[0], pe[1], 'End', fontsize=14, color='magenta')

    plt.grid(True)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()

