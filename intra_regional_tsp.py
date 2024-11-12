import numpy as np
import connecting_path
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import random
import matplotlib.pyplot as plt

def connect_path_for_tsp(start_pair, intersections):
    """
    Create a TSP path starting from a given pair and connecting all intersections in sequence.

    :param start_pair: Tuple, (start, end) pair to initiate the path
    :param intersections: List of intersection points
    :return: Complete path connecting intersections in order
    """
    path = [start_pair[0], start_pair[1]]
    for intersection in intersections[1:]:
        path = connecting_path.add_intersection_points_to_path(path, intersection)
    return path

def get_pairs(intersections, top_pairs=4):
    """
    Generate start-end pairs for a path based on intersections.

    :param intersections: List of intersection points
    :param top_pairs: Integer, the number of top pairs to consider (default is 4)
    :return: List of start-end pairs
    """
    first_pair = (intersections[0][0], intersections[0][1])
    path = connect_path_for_tsp(first_pair, intersections)
    all_pairs = [
        (path[0], path[-1]),
        (path[1], path[-2]),
        (path[-2], path[1]),
        (path[-1], path[0])
    ]
    # Return only the specified top pairs if selective evaluation is enabled
    return all_pairs[:top_pairs]

def calculate_distance(point1, point2):
    """
    Calculate the Euclidean distance between two points.

    :param point1: Coordinates of the first point
    :param point2: Coordinates of the second point
    :return: Float, Euclidean distance between the two points
    """
    return np.linalg.norm(np.array(point1) - np.array(point2))

def build_distance_matrix(sub_polygons):
    """ Build a matrix representing distances between start-end pairs in all sub-polygons.

    :param sub_polygons: List of polygons, each containing start-end pairs
    :return: Distance matrix for all start-end pairs
    """
    num_sub_polygons = len(sub_polygons)
    distance_matrix = np.zeros((num_sub_polygons, num_sub_polygons))
    for i in range(num_sub_polygons):
        for j in range(num_sub_polygons):
            if i != j:
                min_distance = float('inf')
                for start_end_pair_i in sub_polygons[i]:
                    for start_end_pair_j in sub_polygons[j]:
                        distance = calculate_distance(start_end_pair_i[1], start_end_pair_j[0])
                        if distance < min_distance:
                            min_distance = distance
                distance_matrix[i][j] = min_distance
            else:
                distance_matrix[i][j] = 0  # No self-loop cost
    return distance_matrix

def solve_tsp_with_routing(distance_matrix):
    """ Solve the TSP problem on a distance matrix to find the optimal polygon visiting order.

    :param distance_matrix: Matrix of distances between start-end pairs of sub-polygons
    :return: List of indices representing the optimal order of polygons
    """
    num_polygons = len(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(num_polygons, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.GLOBAL_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 20  # Full time limit for best results

    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        index = routing.Start(0)
        optimal_order = []
        while not routing.IsEnd(index):
            optimal_order.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        return optimal_order
    else:
        print("No solution found.")
        return None

def multi_start_tsp(polygons, all_intersections, max_trials=10, selective_pairs=4):
    """ Perform multi-start TSP optimization to minimize path cost, trying multiple random initial orders.

    :param polygons: List of Polygon objects
    :param all_intersections: List of intersections for each polygon
    :param max_trials: Maximum number of TSP trials (default is 10)
    :param selective_pairs: Number of start-end pairs to evaluate per polygon
    :return: Optimally sorted polygons and the minimum path cost
    """
    best_order = None
    best_cost = float('inf')
    trial = 0

    while trial < max_trials:
        trial += 1
        print(f"Trial {trial}...")

        # Randomize initial order by shuffling
        random_order = list(range(len(polygons)))
        random.shuffle(random_order)
        print(f"New order: {random_order}")

        # Get start-end pairs for each polygon with selective evaluation of top pairs
        pairs = [get_pairs(all_intersections[i], top_pairs=selective_pairs) for i in random_order]

        # Build distance matrix for this specific setup
        distance_matrix = build_distance_matrix(pairs)

        # Solve TSP for the current setup
        optimal_order = solve_tsp_with_routing(distance_matrix)

        if optimal_order is not None:
            current_cost = sum(
                distance_matrix[optimal_order[i]][optimal_order[i + 1]]
                for i in range(len(optimal_order) - 1)
            )

            # Update the best solution if the current solution is better
            if current_cost < best_cost:
                best_cost = current_cost
                best_order = [random_order[idx] for idx in optimal_order]
                print(f"New best cost found: {best_cost}")

    sorted_polygons = [polygons[i] for i in best_order] if best_order else polygons
    return sorted_polygons, best_cost

def solve_intra_regional_tsp(polygons, all_intersections, trials=10):
    """
    Solve the TSP for polygons within a region to find the optimal visiting order.

    :param region: Region identifier
    :param polygons: List of Polygon objects
    :param all_intersections: List of intersections for each polygon
    :param trials: Number of multi-start trials for TSP optimization
    :return: Sorted list of polygons in optimal visiting order
    """
    sorted_polygons, best_cost = multi_start_tsp(polygons, all_intersections, max_trials=trials)
    print(f"Optimal path cost: {best_cost}")
    return sorted_polygons


def visualize_subpolygons_with_distances(polygons, pairs):
    """
    Visualize all subpolygons with start-end pairs and connect each end point
    to each start point in other subpolygons, displaying the distances.

    Parameters:
    - polygons: List of polygon objects, each with its own vertices.
    - pairs: List of tuples for each subpolygon, where each tuple contains a start and end point.
    """
    fig, ax = plt.subplots(figsize=(10, 10))

    # Plot each subpolygon and its start-end pairs
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
    ax.set_title("Subpolygons with Start-End Connections and Distances")
    ax.set_aspect('equal')
    plt.show()

def visualize_distance_matrix(matrix, labels=None):
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
    """
    Plot a single subpolygon with its start-end pairs labeled.

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

    fig.suptitle('Start-End Pairs on Subpolygon')
    plt.show()
