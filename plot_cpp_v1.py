import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.patches import Patch
from Polygon import Polygon
from shapely.geometry import Polygon as ShapelyPolygon, MultiPolygon

# Pop plot out of IDE
#matplotlib.use('TkAgg')

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
    #plt.show()

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


def plot_multi_polys_path(current_path_width, polygons, path, obstacles=None, show_coverage=False, transit_flags=None, hide_plot_legend=False, hide_sub_polygon_indices=False):
    """
    Plot multiple polygons, the path between the polygons, and the start/end points of the mission.
    Highlight hard edges specified for each polygon, and label each vertex with its index.
    Obstacles are plotted with red edges. Transit edges are highlighted differently.
    Sub-polygon indices are shown at their centroids.
    """
    labels_used = {"Path line": False, "Transit Line": False, "Start point": False, "End point": False, "Hard Edge": False}
    hide_sub_polygon_indices = True
    # Create a figure and axis
    fig, ax = plt.subplots(1, 1)

    # Plot sub-polygons and display their indices at centroids
    for i, poly in enumerate(polygons):
        x_coords, y_coords = poly.get_coords()

        # Ensure the polygon is closed by repeating the first vertex at the end
        if len(x_coords) > 0 and len(y_coords) > 0:
            x_coords.append(x_coords[0])
            y_coords.append(y_coords[0])

        # Plot the polygon edges, highlighting hard edges in red
        for e in poly.edges:
            #if e.is_hard_edge:
            #    ax.plot([e.v_from.x, e.v_to.x], [e.v_from.y, e.v_to.y], 'r-')  # Hard edge in red
            #else:
            ax.plot([e.v_from.x, e.v_to.x], [e.v_from.y, e.v_to.y], 'k-')  # Normal edge in black

        # Calculate and plot the centroid
        if not hide_sub_polygon_indices:
            if len(x_coords) > 1:
                centroid_x = np.mean(x_coords[:-1])  # Ignore duplicate last point for centroid
                centroid_y = np.mean(y_coords[:-1])
                ax.text(centroid_x, centroid_y, str(i), fontsize=10, color='blue', ha='center', va='center')

    # Plot obstacles with hard edges
    if obstacles:
        for o in obstacles:
            for e in o.edges:
                if e.is_hard_edge:
                    hard_edge_label = "Hard Edge" if not labels_used["Hard Edge"] else None
                    ax.plot([e.v_from.x, e.v_to.x], [e.v_from.y, e.v_to.y], 'r-', label=hard_edge_label)
                    labels_used["Hard Edge"] = True
                else:
                    ax.plot([e.v_from.x, e.v_to.x], [e.v_from.y, e.v_to.y], 'k-')

    # Plot the path
    if len(path) > 0:
        path_x, path_y = path[:, 0], path[:, 1]

        for i in range(len(path) - 1):
            if transit_flags and transit_flags[i] == "transit" and transit_flags[i + 1] == "transit":
                label = 'Transit Line' if not labels_used["Transit Line"] else None
                ax.plot([path_x[i], path_x[i + 1]], [path_y[i], path_y[i + 1]], 'y--', linewidth=1.5,
                        path_effects=[pe.Stroke(linewidth=3, foreground='black'), pe.Normal()], label=label)
                labels_used["Transit Line"] = True
            else:
                label = 'Path line' if not labels_used["Path line"] else None
                ax.plot([path_x[i], path_x[i + 1]], [path_y[i], path_y[i + 1]], 'g-', linewidth=1, label=label)
                labels_used["Path line"] = True

        # Highlight start and end points of the path
        start_label = 'Start point' if not labels_used["Start point"] else None
        end_label = 'End point' if not labels_used["End point"] else None
        ax.plot(path_x[0], path_y[0], 'bo', markersize=8, label=start_label)
        ax.plot(path_x[-1], path_y[-1], 'ro', markersize=8, label=end_label)
        labels_used["Start point"] = True
        labels_used["End point"] = True

        # Plot coverage area along the path if enabled
        if show_coverage:
            for i in range(len(path) - 1):
                p1 = path[i]
                p2 = path[i + 1]
                segment_vector = p2 - p1
                perp_vector = np.array([-segment_vector[1], segment_vector[0]])
                if np.linalg.norm(perp_vector) != 0:
                    perp_vector = perp_vector / np.linalg.norm(perp_vector) * current_path_width / 2

                # Define the four corners of the coverage area for this segment
                corner1 = p1 + perp_vector
                corner2 = p1 - perp_vector
                corner3 = p2 - perp_vector
                corner4 = p2 + perp_vector

                # Plot the coverage area for this segment
                ax.fill([corner1[0], corner2[0], corner3[0], corner4[0]],
                        [corner1[1], corner2[1], corner3[1], corner4[1]],
                        'orange', alpha=0.3)

    else:
        print("Empty path")

    # Add legend
    if not hide_plot_legend:
        ax.legend()
    ax.set_aspect('equal')
    plt.show()
    return fig


def plot_coverage_areas(polygons, coverage_area, overlap_buffered_lines, outlier_buffered_lines, path=None, transit_flags=None, hide_plot_legend=False, hide_sub_polygon_indices=False):
    """
    Plot polygons with coverage areas, overlap areas, and outlier areas using buffered lines.
    Highlight hard edges in polygons and obstacles.

    :param polygons: List of sub-polygons
    :param coverage_area: Shapely polygon representing the covered area
    :param overlap_buffered_lines: Buffered lines for overlap visualization
    :param outlier_buffered_lines: Buffered lines for outlier visualization
    :param path: Optional NumPy array of the path [[x1, y1], [x2, y2], ...]
    :param transit_flags: Optional list of flags indicating transit segments
    :param hide_plot_legend: Boolean to hide the plot legend
    :param hide_sub_polygon_indices: Boolean to hide sub-polygon indices
    """
    fig, ax = plt.subplots(1, 1)
    marker_size = 3  # Marker size for vertices

    # Plot the coverage area (below edges)
    if not coverage_area.is_empty:
        plot_polygon_with_holes(ax, coverage_area, '#4CAF50')  # Green for covered area

    # Plot the overlap area using buffered lines
    if overlap_buffered_lines:
        plot_overlap_areas(ax, overlap_buffered_lines, color='orange', alpha=0.6)  # Orange for overlap area

    # Plot the outlier area using buffered lines
    if outlier_buffered_lines:
        plot_overlap_areas(ax, outlier_buffered_lines, color='red', alpha=0.5)  # Red for outlying area

    # Plot polygon edges (highest priority)
    for i, poly in enumerate(polygons):
        # Plot edges, highlighting hard edges in red
        for e in poly.edges:
            if e.is_hard_edge:
                ax.plot([e.v_from.x, e.v_to.x], [e.v_from.y, e.v_to.y], 'r-', zorder=3)  # Hard edge in red
            else:
                ax.plot([e.v_from.x, e.v_to.x], [e.v_from.y, e.v_to.y], 'k-', zorder=3)  # Normal edge in black

        # Plot the polygon vertices and label indices at centroids
        x_coords, y_coords = poly.get_coords()
        ax.plot(x_coords, y_coords, 'ko', markersize=marker_size, zorder=3)  # Ensure vertices are visible

        if not hide_sub_polygon_indices:
            centroid_x = np.mean(x_coords)
            centroid_y = np.mean(y_coords)
            ax.text(centroid_x, centroid_y, str(i), fontsize=10, color='blue', ha='center', va='center', zorder=3)

    # Plot the path if provided
    if path is not None:
        path_x, path_y = path[:, 0], path[:, 1]
        for i in range(len(path) - 1):
            if transit_flags and transit_flags[i] == "transit" and transit_flags[i + 1] == "transit":
                ax.plot(
                    [path_x[i], path_x[i + 1]], [path_y[i], path_y[i + 1]],
                    'y--', linewidth=1.5,
                    path_effects=[pe.Stroke(linewidth=3, foreground='black'), pe.Normal()],
                    label="Transit Line" if i == 0 else None, zorder=2
                )
            else:
                ax.plot(
                    [path_x[i], path_x[i + 1]], [path_y[i], path_y[i + 1]],
                    'g-', linewidth=1.5, label="Path Line" if i == 0 else None, zorder=2
                )

        # Mark start and end points
        ax.plot(path_x[0], path_y[0], 'bo', markersize=8, label="Start Point", zorder=3)
        ax.plot(path_x[-1], path_y[-1], 'ro', markersize=8, label="End Point", zorder=3)

    # Add legend for coverage areas
    if not hide_plot_legend:
        legend_patches = [
            Patch(color='#4CAF50', alpha=0.5, label='Covered Area'),
            Patch(color='orange', alpha=0.6, label='Overlapped Area'),
            Patch(color='red', alpha=0.5, label='Outlying Area'),
        ]
        ax.legend(handles=legend_patches, loc="upper right")

    ax.set_aspect('equal')
    #plt.show()
    return fig


def plot_overlap_areas(ax, overlap_buffered_lines, color='orange', alpha=0.6):
    if overlap_buffered_lines:
        buffered_lines = overlap_buffered_lines.geoms if isinstance(overlap_buffered_lines, MultiPolygon) else overlap_buffered_lines
        for line in buffered_lines:
            if line.geom_type == 'Polygon' and not line.is_empty:
                x, y = line.exterior.xy
                ax.fill(x, y, color=color, alpha=alpha)


def plot_polygon_with_holes(ax, polygon, color):
    """
    Helper function to plot a Shapely Polygon with holes.
    Ensures that holes are always white and displayed above other areas.
    """
    if polygon.geom_type == 'Polygon':
        # Plot the main polygon area
        x, y = polygon.exterior.xy
        ax.fill(x, y, color=color, alpha=0.5, edgecolor='black', linewidth=1.5, zorder=1)

        # Plot the holes explicitly as white
        for hole in polygon.interiors:
            hx, hy = hole.xy
            ax.fill(hx, hy, color='white', alpha=1.0, zorder=2)  # Ensure holes are above with zorder
    elif polygon.geom_type == 'MultiPolygon':
        for sub_polygon in polygon.geoms:
            plot_polygon_with_holes(ax, sub_polygon, color)



def plot_polygon(ax, polygon, color, alpha=0.5):
    """
    Helper function to plot a Shapely Polygon or MultiPolygon.
    """
    if polygon.geom_type == 'Polygon':
        x, y = polygon.exterior.xy
        ax.fill(x, y, color=color, alpha=alpha)
    elif polygon.geom_type == 'MultiPolygon':
        for sub_polygon in polygon.geoms:
            plot_polygon(ax, sub_polygon, color, alpha)


def plot_vectors_simple(poly, b, b_mate, a, v_extended, v_extended2, boundary, show_legend=True):
    """
    Plot the polygon, points b, b_mate, a, and multiple vectors (L_flight, L_flight_ext, and L_flight_2 reversed) for a convex polygon with an optional legend.
    Additionally, draw intersection points where vectors intersect polygon edges.
    """
    fig, ax = plt.subplots()  # Create the figure and axis
    ax.set_aspect('equal')
    ax.set_title("Polygon and Vectors with Intersection Points")

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

    #plt.show()  # Display the plot

def plot_path(poly, b, b_mate, a, path):
    """ Plot the original vector from b to b_mate, the offset vector, the boundary box, the polygon,
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
        ax.plot(path_x[0], path_y[0], 'go', markersize=8, label='Start Point')  # Start point
        ax.plot(path_x[-1], path_y[-1], 'yo', markersize=8, label='End Point')  # End point

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

def plot_paths_comparison(poly, b1, b_mate1, a1, path1, b2, b_mate2, a2, path2, best_path_output, score1,score2):
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
    min_x, max_x, min_y, max_y = poly.compute_boundary()
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
        ax.plot(path_x[0], path_y[0], 'go', markersize=8, label='Start Point')  # Start point
        ax.plot(path_x[-1], path_y[-1], 'yo', markersize=8, label='End Point')  # End point

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

    ax.grid(True)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

def plot_lines(v_initial, last_intersection,title):
    # Extract points from the data
    p1, p2 = v_initial[0], v_initial[1]  # Line from v_initial
    q1, q2 = last_intersection  # Line from last_intersection

    # Create a new figure
    plt.figure()

    # Plot the first line
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'r-', label="v_initial Line", linewidth=2)

    # Plot the second line
    plt.plot([q1[0], q2[0]], [q1[1], q2[1]], 'b-', label="last_intersection Line", linewidth=2)

    # Add labels and title
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title(title)
    plt.legend()

    # Set equal scaling for both axes
    plt.gca().set_aspect('equal', adjustable='box')

    # Show grid
    plt.grid(True)

    # Show the plot
    plt.show()

def plot_shapely_polygon_simple(polygon, title="Shapely Polygon"):
    """
    Plot a Shapely Polygon or MultiPolygon, including holes, with specific styling:
    - Coverage region: Blue fill with a black outline.
    - Interior holes: White fill with no outline.

    :param polygon: A Shapely Polygon or MultiPolygon.
    :param title: Title of the plot.
    """
    plt.figure(figsize=(6, 6))
    ax = plt.gca()
    ax.set_title(title)

    # Handle MultiPolygon by iterating through each component
    if isinstance(polygon, MultiPolygon):
        for poly in polygon.geoms:
            # Plot the exterior
            x, y = poly.exterior.xy
            ax.fill(x, y, facecolor='blue', edgecolor='black', linewidth=1.5, alpha=0.5)  # Coverage region

            # Plot the holes (interiors)
            for interior in poly.interiors:
                ix, iy = interior.xy
                ax.fill(ix, iy, facecolor='white', edgecolor='none')  # Interior holes
    elif isinstance(polygon, ShapelyPolygon):  # Handle single Polygon
        if not polygon.is_empty:
            # Plot the exterior
            x, y = polygon.exterior.xy
            ax.fill(x, y, facecolor='blue', edgecolor='black', linewidth=1.5, alpha=0.5)  # Coverage region

            # Plot the holes (interiors)
            for interior in polygon.interiors:
                ix, iy = interior.xy
                ax.fill(ix, iy, facecolor='white', edgecolor='none')  # Interior holes
    else:
        print("The provided geometry is neither a Polygon nor a MultiPolygon.")
        return

    ax.set_aspect('equal')  # Ensure equal scaling
    plt.grid(True)
    plt.show()


def plot_buffered_lines(buffered_lines, polygon=None, obstacles=None):
    """
    Plots the list of overlapping buffered lines along with the main polygon and obstacles if provided.

    :param buffered_lines: List of Shapely Polygons representing overlapping buffered lines.
    :param polygon: (Optional) Shapely Polygon representing the main polygon.
    :param obstacles: (Optional) List of Shapely Polygons representing obstacles.
    """
    fig, ax = plt.subplots(figsize=(10, 8))

    # Plot the main polygon
    if polygon:
        x, y = polygon.exterior.xy
        ax.plot(x, y, label="Main Polygon", linewidth=2, color='blue')

    # Plot the obstacles
    if obstacles:
        for obs in obstacles:
            x, y = obs.exterior.xy
            ax.fill(x, y, label="Obstacle", color='red', alpha=0.5)

    # Plot the overlapping buffered lines in orange
    for i, line in enumerate(buffered_lines):
        if isinstance(line, MultiPolygon):
            for subline in line.geoms:
                x, y = subline.exterior.xy
                ax.fill(x, y, color='orange', alpha=0.6, label=f"Overlap {i}" if i == 0 else None)
        elif isinstance(line, ShapelyPolygon):
            x, y = line.exterior.xy
            ax.fill(x, y, color='orange', alpha=0.6, label=f"Overlap {i}" if i == 0 else None)

    # Add labels and legend
    ax.set_title("Overlapping Buffered Lines")
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    ax.legend()
    ax.set_aspect('equal', adjustable='box')
    plt.show()