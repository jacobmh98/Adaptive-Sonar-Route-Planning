import numpy as np
import random
import cpp_connect_path
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
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
        path = cpp_connect_path.add_intersection_points_to_path(path, intersection)
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

    # Memoization dictionary to store solutions for each unique order
    memoized_solutions = {}

    while trial < max_trials:
        trial += 1
        print(f"Trial {trial}...")

        # Randomize initial order by shuffling
        random_order = list(range(len(polygons)))
        random.shuffle(random_order)
        print(f"New order: {random_order}")

        # Convert random_order to a tuple so it can be used as a key
        random_order_tuple = tuple(random_order)

        # Check if this order has already been computed
        if random_order_tuple in memoized_solutions:
            print("Order already solved. Retrieving stored solution.")
            optimal_order, current_cost = memoized_solutions[random_order_tuple]

        else:
            # Get start-end pairs for each polygon with selective evaluation of top pairs
            pairs = [get_pairs(all_intersections[i], top_pairs=selective_pairs) for i in random_order]

            # Build distance matrix for this specific setup
            distance_matrix = build_distance_matrix(pairs)
            visualize_distance_matrix(distance_matrix)

            # Solve TSP for the current setup
            optimal_order = solve_tsp_with_routing(distance_matrix)

            if optimal_order is not None:
                # Calculate the cost for this order
                current_cost = sum(
                    distance_matrix[optimal_order[i]][optimal_order[i + 1]]
                    for i in range(len(optimal_order) - 1)
                )

                # Store the solution in memoized_solutions
                memoized_solutions[random_order_tuple] = (optimal_order, current_cost)

        # Update the best solution if the current solution is better
        if optimal_order is not None and current_cost < best_cost:
            best_cost = current_cost
            best_order = [random_order[idx] for idx in optimal_order]
            print(f"New best cost found: {best_cost}")

    # Final sorted polygons and intersections based on the best order found
    sorted_polys = [polygons[i] for i in best_order] if best_order else polygons
    sorted_inters = [all_intersections[i] for i in best_order]

    return sorted_polys, sorted_inters, best_cost

def solve_intra_regional_tsp(polygons, all_intersections, trials=10):
    """
    Solve the TSP for polygons within a region to find the optimal visiting order.

    :param region: Region identifier
    :param polygons: List of Polygon objects
    :param all_intersections: List of intersections for each polygon
    :param trials: Number of multi-start trials for TSP optimization
    :return: Sorted list of polygons in optimal visiting order
    """
    sorted_polys, sorted_inters, best_cost = multi_start_tsp(polygons, all_intersections, max_trials=trials)
    print(f"Optimal path cost: {best_cost}")
    return sorted_polys, sorted_inters


def visualize_distance_matrix(distance_matrix, title="Distance Matrix", cmap="viridis"):
    """ Visualize a distance matrix as a heatmap.

    :param distance_matrix: 2D array-like, the distance matrix to visualize
    :param title: String, title of the plot (default is "Distance Matrix")
    :param cmap: String, colormap to use for the heatmap (default is "viridis")
    """
    plt.figure(figsize=(8, 6))
    plt.imshow(distance_matrix, cmap=cmap, interpolation="nearest")
    plt.colorbar(label="Distance")
    plt.title(title)
    plt.xlabel("Destination Polygon Index")
    plt.ylabel("Source Polygon Index")
    plt.xticks(range(len(distance_matrix)), range(len(distance_matrix)))
    plt.yticks(range(len(distance_matrix)), range(len(distance_matrix)))
    plt.grid(False)
    plt.show()

