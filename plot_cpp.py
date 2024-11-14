import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from Polygon import Polygon
from shapely.geometry import Polygon as ShapelyPolygon

# Pop plot out of IDE
matplotlib.use('TkAgg')


def plot_antipodal_points(polygon, antipodal_vertices):
    """ Plot the polygon and highlight the antipodal points.

    :param polygon: Polygon object with vertices.
    :param antipodal_vertices: List of tuples representing antipodal vertex pairs.
    """
    # Get the x and y coordinates of the vertices
    x_coords, y_coords = polygon.get_coords()

    # Plot the polygon (ensure the polygon closes by connecting last and first point)
    plt.plot(x_coords + [x_coords[0]], y_coords + [y_coords[0]], 'k-', marker='o')

    # Plot vertex indices for reference
    for v in polygon.vertices:
        plt.text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='blue')

    # Plot the antipodal pairs
    for (i, j) in antipodal_vertices:
        xk = [polygon.vertices[i].x, polygon.vertices[j].x]
        yk = [polygon.vertices[i].y, polygon.vertices[j].y]
        plt.plot(xk, yk, linestyle='--', marker='o', color=[0.7, 0.7, 0.7])

    # Display the plot
    #plt.grid()
    plt.show()

def plot_simple_poly_path(polygon, path):
    # Create a figure and axis
    fig, ax = plt.subplots(1, 1)
    ax.set_aspect('equal')  # Set axis to equal scaling

    x_coords, y_coords = polygon.get_coords()
    ax.plot(x_coords, y_coords, 'k-', marker='o', markersize=3, label='Polygon')
    ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'k-')

    # Label each vertex with its index
    for idx, (x, y) in enumerate(zip(x_coords, y_coords)):
        ax.text(x, y, f'{idx}', fontsize=8, color='green', ha='right', va='bottom')


    # Plot the path
    if len(path) > 0:
        path_x, path_y = path[:, 0], path[:, 1]
        ax.plot(path_x, path_y, 'g-', label='Path', linewidth=2)

        # Highlight the start and end points of the path
        ax.plot(path_x[0], path_y[0], 'go', markersize=8, label='Start Point')  # Start point
        ax.plot(path_x[-1], path_y[-1], 'ro', markersize=8, label='End Point')  # End point

    plt.xlabel('X')
    plt.ylabel('Y')

    # Show the plot in a separate window
    plt.show()


def plot_multi_polys_path(polygon, current_path_width, polygons, path, show_coverage = False):
    """
    Plot multiple polygons, the path between the polygons, and the start/end points of the mission.
    Highlight hard edges specified for each polygon, and label each vertex with its index.

    :param polygon: Polygon of the region
    :param current_path_width: Width of the path
    :param polygons: List of the sub-polygons
    :param path: NumPy array, array of points representing the path [[x1, y1], [x2, y2], ...]
    """
    plot_sub_polygons = True

    hard_edges = []
    for poly in polygons:
        sub_list = []
        for vertex in poly.vertices:
            if vertex.edge_from_v_is_hard:
                sub_list.append(vertex.index)
        hard_edges.append(sub_list)

    # Create a figure and axis
    fig, ax = plt.subplots(1, 1)
    color = "k"

    # Plot the polygon
    if not plot_sub_polygons:
        x_coords, y_coords = polygon.get_coords()
        ax.plot(x_coords, y_coords, f'{color}-', marker='o')
        ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], f'{color}-')

    if plot_sub_polygons:
        for i, poly in enumerate(polygons):
            x_coords, y_coords = poly.get_coords()
            ax.plot(x_coords, y_coords, 'k-', marker='o', markersize=3, label='Polygon')
            ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'k-')

            # Label each vertex with its index
            for idx, (x, y) in enumerate(zip(x_coords, y_coords)):
                ax.text(x, y, f'{idx}', fontsize=8, color='green', ha='right', va='bottom')

            # Plot hard edges
            if i < len(hard_edges):
                for vertex_index in hard_edges[i]:
                    start_idx = vertex_index
                    end_idx = (vertex_index + 1) % len(x_coords)  # Next vertex, looping back if it's the last vertex
                    ax.plot([x_coords[start_idx], x_coords[end_idx]],
                            [y_coords[start_idx], y_coords[end_idx]],
                            'r-', linewidth=3, label='_nolegend_')

            # Find the center of the polygon to place the label
            centroid_x = np.mean(x_coords)
            centroid_y = np.mean(y_coords)

            # Label the polygon with its number (the order number)
            ax.text(centroid_x, centroid_y, f'{i}', fontsize=10, color='blue')

    # Plot the path
    if len(path) > 0:
        path_x, path_y = path[:, 0], path[:, 1]
        ax.plot(path_x, path_y, 'g-', label='Path', linewidth=1)

        # Highlight the start and end points of the path
        ax.plot(path_x[0], path_y[0], 'go', markersize=8, label='Start Point')  # Start point
        ax.plot(path_x[-1], path_y[-1], 'ro', markersize=8, label='End Point')  # End point

        # Compute and plot coverage area along the path
        if show_coverage:
            for i in range(len(path) - 1):
                p1 = path[i]
                p2 = path[i + 1]
                # Vector along the path segment
                segment_vector = p2 - p1
                # Normalize the vector to get the perpendicular direction
                perp_vector = np.array([-segment_vector[1], segment_vector[0]])
                if np.linalg.norm(perp_vector) != 0:
                    perp_vector = perp_vector / np.linalg.norm(perp_vector) * current_path_width / 2

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

    #plt.grid(True)
    #plt.xlabel('X')
    #plt.ylabel('Y')

    # Show the plot in a separate window
    ax.set_aspect('equal')
    plt.show()

    return fig


def plot_vectors_simple(poly, b, b_mate, a, v_initial, v_extended, v_extended2, boundary, show_legend=True):
    """
    Plot the polygon, points b, b_mate, a, and multiple vectors (L_flight, L_flight_ext, and L_flight_2 reversed) for a convex polygon with an optional legend.
    Additionally, draw intersection points where vectors intersect polygon edges.
    """
    fig, ax = plt.subplots()  # Create the figure and axis
    ax.set_aspect('equal')
    ax.set_title("Polygon and Vectors with Intersection Points")

    # Draw the grid first, so it appears behind everything else
    #ax.grid(True)

    # Plot the boundary box (zorder=1 to draw below the vectors and points)
    min_x, max_x, min_y, max_y = boundary
    ax.plot([min_x, max_x, max_x, min_x, min_x], [min_y, min_y, max_y, max_y, min_y], 'k--', label='Boundary Box', zorder=1)

    # Plot the points b, b_mate, and a (zorder=3 to draw above the polygon)
    ax.plot([b[0], b_mate[0], a[0]], [b[1], b_mate[1], a[1]], 'ro', markersize=10, label='Points (b, b_mate, a)', zorder=3)
    ax.text(b[0], b[1], 'b', fontsize=12, color='darkblue', zorder=4)
    ax.text(b_mate[0], b_mate[1], 'b_mate', fontsize=12, color='darkblue', zorder=4)
    ax.text(a[0], a[1], 'a', fontsize=12, color='darkblue', zorder=4)

    # Plot the convex polygon (zorder=2 to draw below the vectors)
    x_coords, y_coords = poly.get_coords()
    ax.plot(x_coords, y_coords, 'k-', marker='o', label='Polygon', zorder=2)
    ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'k-', zorder=2)

    # Adjust arrow length by subtracting the arrow head size from the total length
    def adjust_arrow(start, end, head_length=0.1):
        vector = np.array(end) - np.array(start)
        vector_length = np.linalg.norm(vector)
        if vector_length > head_length:
            # Shorten the arrow by the head length
            adjusted_end = start + vector * ((vector_length - head_length) / vector_length)
        else:
            adjusted_end = end  # If the vector is very short, don't adjust
        return adjusted_end

    # Plot the vector from b to b_mate as an arrow (adjusted)
    adjusted_b_mate = adjust_arrow(b, b_mate)
    ax.arrow(b[0], b[1], adjusted_b_mate[0] - b[0], adjusted_b_mate[1] - b[1],
             head_width=0.05, head_length=0.1, fc='green', ec='green', linewidth=2, label='Vector (b to b_mate)', zorder=4)

    # Plot the extended vector (L_flight_ext) as another arrow (adjusted)
    ext_start, ext_end = v_extended
    adjusted_ext_end = adjust_arrow(ext_start, ext_end)
    ax.arrow(ext_start[0], ext_start[1], adjusted_ext_end[0] - ext_start[0], adjusted_ext_end[1] - ext_start[1],
             head_width=0.05, head_length=0.1, fc='blue', ec='blue', linewidth=2, label='First Extended Offset Vector', zorder=4)

    # Reverse the start and end of the additional vector v_extended2
    vector_2_start, vector_2_end = v_extended2

    # Plot the reversed additional vector (L_flight_2) as another arrow (adjusted)
    adjusted_vector_2_end = adjust_arrow(vector_2_start, vector_2_end)
    ax.arrow(vector_2_start[0], vector_2_start[1],
             adjusted_vector_2_end[0] - vector_2_start[0], adjusted_vector_2_end[1] - vector_2_start[1],
             head_width=0.05, head_length=0.1, fc='purple', ec='purple', linewidth=2, label='Second Extended Offset Vector', zorder=4)

    # Find intersection points for the first and second vectors
    i1_1, i1_2 = Polygon.find_intersections(poly, v_extended)  # First vector intersections
    i2_1, i2_2 = Polygon.find_intersections(poly, v_extended2)    # Second vector intersections

    # Plot intersection points for the first vector
    ax.plot([i1_1[0], i1_2[0]], [i1_1[1], i1_2[1]], 'rx', markersize=8, zorder=5)
    # Plot intersection points for the second vector
    ax.plot([i2_1[0], i2_2[0]], [i2_1[1], i2_2[1]], 'rx', markersize=8, label='Intersections', zorder=5)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    # Show the legend if the flag is True
    if show_legend:
        ax.legend(loc='best')

    plt.show()  # Display the plot


def plot_path(poly, b, b_mate, a, path):
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


def plot_paths_comparison(poly, b1, b_mate1, a1, path1, b2, b_mate2, a2, path2, best_path_output, score1,
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
    min_x, max_x, min_y, max_y = compute_boundary(poly)
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


def plot_coverage(polygon, path_points, covered_area, wasted_area, overlap_area):
    # Create the plot
    fig, ax = plt.subplots()

    # Convert your Polygon class to a Shapely Polygon
    poly_coords = [(v.x, v.y) for v in polygon.vertices]
    poly_shape = ShapelyPolygon(poly_coords)

    # Plot the original polygon
    x_poly, y_poly = poly_shape.exterior.xy
    ax.plot(x_poly, y_poly, 'k-', label='Polygon Boundary')

    # Plot the path as a line
    path_x, path_y = zip(*path_points)
    ax.plot(path_x, path_y, 'b-', linewidth=2, label='Path')

    # Plot the covered area inside the polygon
    if not covered_area.is_empty:
        if covered_area.geom_type == 'Polygon':
            x, y = covered_area.exterior.xy
            ax.fill(x, y, color='green', alpha=0.5, label='Covered Area')
        elif covered_area.geom_type == 'MultiPolygon':
            for sub_polygon in covered_area.geoms:
                x, y = sub_polygon.exterior.xy
                ax.fill(x, y, color='green', alpha=0.5)

    # Plot the wasted area outside the polygon, with label only once
    if not wasted_area.is_empty:
        labeled = False
        if wasted_area.geom_type == 'Polygon':
            x, y = wasted_area.exterior.xy
            ax.fill(x, y, color='red', alpha=0.5, label='Wasted Area')
        elif wasted_area.geom_type == 'MultiPolygon':
            for i, sub_polygon in enumerate(wasted_area.geoms):
                x, y = sub_polygon.exterior.xy
                ax.fill(x, y, color='red', alpha=0.5, label='Wasted Area' if not labeled else None)
                labeled = True

    # Plot the overlap area, with label only once
    if not overlap_area.is_empty:
        labeled = False
        if overlap_area.geom_type == 'Polygon':
            x, y = overlap_area.exterior.xy
            ax.fill(x, y, color='orange', alpha=0.5, label='Overlap Area')
        elif overlap_area.geom_type == 'MultiPolygon':
            for i, sub_polygon in enumerate(overlap_area.geoms):
                x, y = sub_polygon.exterior.xy
                ax.fill(x, y, color='orange', alpha=0.5, label='Overlap Area' if not labeled else None)
                labeled = True

    # Ensure proper aspect ratio and add legend
    ax.set_aspect('equal', 'box')
    ax.legend()
    plt.show()