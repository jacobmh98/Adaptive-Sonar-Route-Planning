import networkx as nx
import connecting_path
import matplotlib.pyplot as plt
import numpy as np


def connect_path_for_tsp(start_pair, intersections):
    path = [start_pair[0], start_pair[1]]

    for intersection in intersections[1:]:
        path = connecting_path.add_intersection_points_to_path(path, intersection)

    return path

def get_start_end_combinations(intersections):
    # Getting first pair used to create the path from the intersections
    first_pair = (intersections[0][0], intersections[0][1])

    # Generating the path
    path = connect_path_for_tsp(first_pair, intersections)

    # Extracting the 4 different start/end combinations from the path
    first_start = path[0]
    first_end = path[-1]
    second_start = path[1]
    second_end = path[-2]
    third_start = path[-2]
    third_end = path[1]
    fourth_start = path[-1]
    fourth_end = path[0]

    # Creating list of tuples of the start and end pairs
    start_end_combinations = [(first_start, first_end), (second_start, second_end),(third_start, third_end), (fourth_start, fourth_end)]

    return start_end_combinations


def calculate_distance(point1, point2):
    """Calculate Euclidean distance between two points."""
    return np.linalg.norm(np.array(point1) - np.array(point2))


def distance_matrix_test(pairs):
    """
    Build a distance matrix where each entry represents the distance
    from the end of one pair to the start of another. Self-connections
    are disallowed by setting them to infinity.

    Parameters:
    - pairs: List of tuples, each containing a start and end point.

    Returns:
    - A 2D distance matrix where each entry (i, j) is the distance from
      the end of pair i to the start of pair j, with self-connections set to infinity.
    """
    num_pairs = len(pairs)
    distance_matrix = np.zeros((num_pairs, num_pairs))

    for i in range(num_pairs):
        end_point_i = pairs[i][1]  # End point of pair i
        for j in range(num_pairs):
            if i == j:
                # Disallow self-connection by setting to infinity
                distance_matrix[i, j] = np.inf
            else:
                start_point_j = pairs[j][0]  # Start point of pair j
                distance_matrix[i, j] = calculate_distance(end_point_i, start_point_j)

    return distance_matrix


def build_graph(pairs, distance_matrix):
    """Create a weighted graph based on pairs and distance matrix."""
    G = nx.DiGraph()  # Directed graph if order matters

    num_nodes = len(pairs)
    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                G.add_edge(i, j, weight=distance_matrix[i, j])

    return G


def solve_tsp_networkx(G, start_node=0):
    """Solve TSP starting from a specified node using an approximate algorithm."""
    path = nx.approximation.traveling_salesman_problem(G, cycle=False, source=start_node)
    return path


def solve_intra_regional_tsp_networkx(region, polygons, all_intersections):
    tsp_sorting_order = []


    # Generate start-end combinations for each polygon
    start_end_combinations = []
    for i,intersections in enumerate(all_intersections):
        start_end_combinations.append(get_start_end_combinations(intersections))
        #plot_polygon_with_start_end_pairs(polygons[i], start_end_combinations[i])

    smaller_test_pairs = []
    for i, start_end_combination in enumerate(start_end_combinations):
        smaller_test_pairs.append(start_end_combination[2])
        #plot_polygon_with_start_end_pairs(polygons[i], [smaller_test[i]])

    distance_matrix = distance_matrix_test(smaller_test_pairs)
    #visualize_distance_matrix(distance_matrix_test_case, labels=[f"Pair {i + 1}" for i in range(len(smaller_test_pairs))])

    G = build_graph(smaller_test_pairs, distance_matrix)
    print("G")

    tsp_order = solve_tsp_networkx(G, start_node=0)
    print("Order")
    sorted_polygons = [polygons[i] for i in tsp_order]
    sorted_start_end_combinations = [smaller_test_pairs[i] for i in tsp_order]

    visualize_tsp_path(sorted_polygons, tsp_order)


    return tsp_sorting_order


def visualize_tsp_path(polygons, tsp_order):
    fig, ax = plt.subplots(figsize=(10, 10))
    for i, idx in enumerate(tsp_order):
        polygon = polygons[idx]
        x_coords, y_coords = polygon.get_coords()
        ax.plot(x_coords + [x_coords[0]], y_coords + [y_coords[0]], 'k-', marker='o')
        ax.text(x_coords[0], y_coords[0], f'P{idx}', fontsize=12, color="blue")

    # Draw lines between ordered polygons
    for i in range(len(tsp_order) - 1):
        start_polygon = polygons[tsp_order[i]]
        end_polygon = polygons[tsp_order[i + 1]]
        x_start, y_start = start_polygon.get_coords()[0], start_polygon.get_coords()[1]
        x_end, y_end = end_polygon.get_coords()[0], end_polygon.get_coords()[1]
        ax.plot([x_start[0], x_end[0]], [y_start[0], y_end[0]], 'b--', alpha=0.6)

    ax.set_title("TSP Path Through Polygons")
    ax.set_aspect('equal')
    plt.show()



def visualize_distance_matrix(matrix, labels=None):
    """Visualize the distance matrix as a heatmap with text annotations."""
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
            if np.isinf(distance):
                text = "Inf"  # For infinity values
            else:
                text = f"{distance:.2f}"  # Format distance to two decimal places
            plt.text(j, i, text, ha='center', va='center', color="white" if distance < np.max(matrix) / 2 else "black")

    plt.title("Distance Matrix Visualization with Values")
    plt.xlabel("Start Point of Pairs")
    plt.ylabel("End Point of Pairs")
    plt.show()