import numpy as np
import polygon_coverage_path
import Rotating_Calipers_antipodal_pairs
import matplotlib.pyplot as plt


def rotating_calipers_path_planner(p, d_pq, ps, pe, pw, bounds):
    """ Algorithm 2: Rotating Calipers Path Planner.
    Computes the optimal back-and-forth path that covers a convex polygon efficiently by testing all antipodal pairs.

    :param p: Polygon
    :param d_pq: List of tuples representing antipodal pairs (i, j).
    :param ps: Starting point
    :param pe: Ending point
    :param pw: Path width
    :param bounds: Boundaries of the polygon.
    :return optimal_path: The best back-and-forth path (computed using best_path).
    """

    # Initialize variables to store the best path and the minimal cost
    min_cost = float('inf')
    optimal_path = None
    new_best_pair = ()

    # Iterate over all antipodal pairs (i, j)
    for (i, j) in d_pq:
        #print(f'i,j: {i},{j}')
        # Compute the best path for the current antipodal pair
        current_path, current_cost = polygon_coverage_path.best_path(p, i, j, ps, pe, pw, bounds)

        # Update the optimal path if the current path has a lower cost
        if current_cost < min_cost:
            min_cost = current_cost
            optimal_path = current_path
            new_best_pair = (i,j)

    return optimal_path

def multi_path_planning(polygons, ps, pe, dx, boundaries):

    total_path = np.empty((0,2))


    for i, current_poly in enumerate(polygons):
        b_index = 0
        b_mate_index = polygon_coverage_path.get_b_mate(polygons[0], b_index)  # Automatically computes counterclockwise neighbour vertex to b
        # Computing polygons antipodal points
        antipodal_vertices = Rotating_Calipers_antipodal_pairs.compute_antipodal_pairs(current_poly)
        # Removes neighbour pairs and double pairs, i.e. for [0,1] and [1,0] only 1 of them is necessary
        filtered_antipodal_vertices = Rotating_Calipers_antipodal_pairs.filter_and_remove_redundant_pairs(current_poly,
                                                                                                          antipodal_vertices)
        # Computing the diametric antipodal pairs. Optimizing number of paths computed
        diametric_antipodal_pairs = Rotating_Calipers_antipodal_pairs.filter_diametric_antipodal_pairs(current_poly,
                                                                                                       filtered_antipodal_vertices)
        # Getting index of a, the diametric antipodal point of b
        diametric_antipode_index = Rotating_Calipers_antipodal_pairs.get_diametric_antipodal_point_index(
            diametric_antipodal_pairs, b_index)

        # (Triangle edge case) If b_mate and a are the same point, change a to last remaining vertex in the triangle polygon
        if np.allclose(b_mate_index, diametric_antipode_index):
            diametric_antipode_index = current_poly.vertices[polygon_coverage_path.get_previous_vertex(current_poly, b_index)].v

        if i < len(polygons)-1: # End point is set to first point in next poly
            shortest_path = rotating_calipers_path_planner(current_poly, diametric_antipodal_pairs, ps, polygons[i+1].vertices[0].v, dx, boundaries[i])
        else:  # If last poly then end is set to the point end point from main
            shortest_path = rotating_calipers_path_planner(current_poly, diametric_antipodal_pairs, ps, pe, dx, boundaries[i])

        total_path = np.vstack([total_path, shortest_path])

    return total_path


# def plot_path(b, b_mate, a, dx, boundaries, polygon, path, ps=None, pe=None):
def multi_poly_plot(path, polygons, ps, pe, dx):
    """
    Plot multiple polygons, the path between the polygons, and the start/end points of the mission.

    :param path: NumPy array, array of points representing the path [[x1, y1], [x2, y2], ...]
    :param polygons: List of Polygon objects to be plotted
    :param ps: Starting point of the mission
    :param pe: Ending point of the mission
    :param dx: float, the distance by which the vector should be offset (this defines the width of coverage)
    """

    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    # Plot the polygons
    for poly in polygons:
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

    # Plot the mission start and end points
    ax.plot(ps[0], ps[1], 'bo', markersize=12, label='Mission Start Point')
    ax.text(ps[0], ps[1], 'Start', fontsize=14, color='blue')

    ax.plot(pe[0], pe[1], 'mo', markersize=12, label='Mission End Point')
    ax.text(pe[0], pe[1], 'End', fontsize=14, color='magenta')

    plt.grid(True)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()
