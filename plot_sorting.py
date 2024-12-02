import numpy as np
import matplotlib.pyplot as plt

def compute_centroid(polygon):
    x_coords = [v.x for v in polygon.vertices]
    y_coords = [v.y for v in polygon.vertices]
    centroid = (np.mean(x_coords), np.mean(y_coords))
    return centroid


def plot_tsp_centroid(polygons, tsp_route):
    # Plot polygons
    plt.figure(figsize=(8, 8))

    for i, polygon in enumerate(polygons):
        x_coords, y_coords = polygon.get_coords()
        x_coords.append(x_coords[0])  # Close the polygon
        y_coords.append(y_coords[0])
        plt.plot(x_coords, y_coords, 'r-', lw=2, marker='o')

        # Add a label to the centroid of the polygon
        centroid = compute_centroid(polygon)
        plt.text(centroid[0], centroid[1], f'{tsp_route[i]}', fontsize=12, ha='center', color='blue')

    # Plot TSP path
    for i in range(len(tsp_route)):
        start_polygon = polygons[tsp_route[i]]
        end_polygon = polygons[tsp_route[(i + 1) % len(tsp_route)]]

        # Draw line between centroids
        centroid_start = compute_centroid(start_polygon)
        centroid_end = compute_centroid(end_polygon)
        plt.plot([centroid_start[0], centroid_end[0]], [centroid_start[1], centroid_end[1]], 'b--', lw=2)

    plt.title("TSP Optimal Path Visiting Polygons")
    plt.show()


def plot_subpolygons_with_distances(polygons, pairs):
    """
    Visualize all subpolygons with start-end pairs and connect each end point
    to each start point in other subpolygons, displaying the distances.

    Parameters:
    - polygons: List of polygon objects, each with its own vertices.
    - pairs: List of tuples for each subpolygon, where each tuple contains a start and end point.
    """
    fig, ax = plt.subplots(figsize=(10, 10))

    # Plot each sub polygon and its start-end pairs
    for i, (polygon, (start, end)) in enumerate(zip(polygons, pairs)):
        # Plot the polygon outline
        x_coords, y_coords = polygon.get_coords()
        ax.plot(x_coords + [x_coords[0]], y_coords + [y_coords[0]], 'k-', marker='o')

        # Plot start-end pair for this polygon
        ax.plot([start[0], end[0]], [start[1], end[1]], 'r--', marker='o')
        ax.text(start[0], start[1], f'S{i} Start', color='blue', fontsize=10, ha='right')
        ax.text(end[0], end[1], f'S{i} End', color='green', fontsize=10, ha='right')

    # Draw lines between each end point and every other start point in other subpolygons
    num_pairs = len(pairs)
    for i in range(num_pairs):
        end_point = pairs[i][1]  # End of pair i
        for j in range(num_pairs):
            if i != j:
                start_point = pairs[j][0]  # Start of pair j
                distance = calculate_distance(end_point, start_point)

                # Draw the line and annotate with distance
                ax.plot([end_point[0], start_point[0]], [end_point[1], start_point[1]], 'b--', alpha=0.5)
                mid_x = (end_point[0] + start_point[0]) / 2
                mid_y = (end_point[1] + start_point[1]) / 2
                ax.text(mid_x, mid_y, f'{distance:.2f}', color='purple', fontsize=8, ha='center')

    # Set plot title and aspect ratio
    ax.set_title("Sub polygons with Start-End Connections and Distances")
    ax.set_aspect('equal')
    plt.show()

def plot_distance_matrix(matrix, labels=None):
    """
    Visualize the distance matrix as a heatmap with text annotations.

    :param matrix: 2D array-like distance matrix
    :param labels: Optional list of labels for axes
    """
    plt.figure(figsize=(8, 8))
    heatmap = plt.imshow(matrix, cmap='viridis', interpolation='nearest')
    plt.colorbar(label='Distance')

    # Add labels if provided
    if labels:
        plt.xticks(range(len(labels)), labels, rotation=90)
        plt.yticks(range(len(labels)), labels)
    else:
        plt.xticks(range(matrix.shape[0]))
        plt.yticks(range(matrix.shape[1]))

    # Annotate each cell with the distance value
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            distance = matrix[i, j]
            text = "Inf" if np.isinf(distance) else f"{distance:.2f}"
            plt.text(j, i, text, ha='center', va='center', color="white" if distance < np.max(matrix) / 2 else "black")

    plt.title("Distance Matrix Visualization with Values")
    plt.xlabel("Start Point of Pairs")
    plt.ylabel("End Point of Pairs")
    plt.show()

def plot_polygon_with_start_end_pairs(polygon, start_end_pairs):
    """ Plot a single sub polygon with its start-end pairs labeled.

    :param polygon: Polygon object with vertices
    :param start_end_pairs: List of start-end pairs for the polygon
    """
    fig, axs = plt.subplots(1, len(start_end_pairs), figsize=(5 * len(start_end_pairs), 5))
    if len(start_end_pairs) == 1:
        axs = [axs]  # Ensure axs is iterable even if there is only one pair

    # Loop through each start-end pair
    for i, (start, end) in enumerate(start_end_pairs):
        ax = axs[i]
        x_coords, y_coords = polygon.get_coords()
        ax.plot(x_coords + [x_coords[0]], y_coords + [y_coords[0]], 'k-', marker='o')
        ax.plot([start[0], end[0]], [start[1], end[1]], 'r--', marker='o')
        ax.text(start[0], start[1], 'Start', color='blue', fontsize=10, ha='right')
        ax.text(end[0], end[1], 'End', color='green', fontsize=10, ha='right')
        ax.set_title(f'Start-End Pair {i + 1}')
        ax.set_aspect('equal')

    fig.suptitle('Start-End Pairs on Sub polygon')
    plt.show()