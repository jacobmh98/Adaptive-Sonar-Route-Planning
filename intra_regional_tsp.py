import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import connecting_path
import coverage_plots
from global_variables import *
from scipy.spatial.distance import euclidean
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from scipy.spatial.distance import euclidean

def add_intersection_points_to_path(path, intersection):
    last_point = path[-1]  # The last point in the current path

    # Unpack the intersection
    p1, p2 = intersection

    # Calculate the distance between the last point and each point in the intersection
    dist_to_p1 = np.linalg.norm(last_point - p1)
    dist_to_p2 = np.linalg.norm(last_point - p2)

    # Add the points in the correct order based on their distance to the last point
    if dist_to_p1 <= dist_to_p2:
        path.append(p1)
        path.append(p2)
    else:
        path.append(p2)
        path.append(p1)

    return path

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
    start_end_combinations = [(first_start, first_end), (second_start, second_end), (third_start, third_end),
                              (fourth_start, fourth_end)]

    return start_end_combinations

def euclidean_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

def compute_distance_matrix(start_end_combinations):
    num_supernodes = len(start_end_combinations)
    num_pairs = 4  # Each supernode has 4 start/end pairs
    matrix_size = num_supernodes * num_pairs

    # Initialize the distance matrix
    distance_matrix = np.zeros((matrix_size, matrix_size))

    # Loop through each supernode and its pairs
    for i, current_pairs in enumerate(start_end_combinations):
        for pair_idx, (start_i, end_i) in enumerate(current_pairs):
            end_index = i * num_pairs + pair_idx

            # Compare this end node to all start nodes in other supernodes
            for j, other_pairs in enumerate(start_end_combinations):
                for other_pair_idx, (start_j, _) in enumerate(other_pairs):
                    start_index = j * num_pairs + other_pair_idx

                    # Ensure we calculate distances between different supernodes
                    if i != j:
                        distance = euclidean_distance(end_i, start_j)
                        distance_matrix[end_index, start_index] = distance
                        distance_matrix[start_index, end_index] = distance  # Ensure symmetry

    return distance_matrix

def is_symmetric(matrix, tol=1e-9):
    """Check if the matrix is symmetric within a given tolerance."""
    rows, cols = matrix.shape
    if rows != cols:
        return False  # It must be a square matrix to be symmetric

    # Compare each element with its transpose, allowing for small differences
    for i in range(rows):
        for j in range(cols):
            if abs(matrix[i][j] - matrix[j][i]) > tol:
                print(matrix[i][j])
                print(matrix[j][i])
                return False
    return True

def solve_vrp_with_supernodes(distance_matrix, start_end_combinations):
    num_supernodes = len(start_end_combinations)
    num_pairs = 4  # Each supernode has 4 start/end pairs

    # Create data model for OR-Tools
    data = {
        'distance_matrix': distance_matrix,
        'num_vehicles': 1,  # Single path/vehicle
        'depot': 0  # Starting point, can adjust as needed
    }

    # Create the routing index manager with multiple choices per supernode
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    # Create a callback for distances
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add disjunctions to ensure only one start/end pair is chosen per supernode
    for i in range(num_supernodes):
        indices = [i * num_pairs + j for j in range(num_pairs)]  # All 4 pairs for supernode `i`
        routing.AddDisjunction(indices)

    # Define search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the VRP
    solution = routing.SolveWithParameters(search_parameters)

    # Extract the TSP order from the solution
    tsp_order = []
    chosen_pairs = []
    if solution:
        index = routing.Start(0)
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            tsp_order.append(node)
            supernode_idx = node // num_pairs
            pair_idx = node % num_pairs
            chosen_pairs.append(start_end_combinations[supernode_idx][pair_idx])
            index = solution.Value(routing.NextVar(index))

    return tsp_order, chosen_pairs

def start_tsp(polygons, intersections, region):
    # Step 1: Compute start/end combinations
    start_end_combinations = [get_start_end_combinations(inter) for inter in intersections]

    plot_tsp_graph(start_end_combinations)

    # Step 2: Compute the distance matrix
    distance_matrix = compute_distance_matrix(start_end_combinations)
    pyomo_test(distance_matrix)

    # Check for symmetry
    if is_symmetric(distance_matrix):
        print("The distance matrix is symmetric.")
    else:
        print("The distance matrix is not symmetric.")

    # Step 3: Solve the VRP-based TSP
    tsp_order, chosen_pairs = solve_vrp_with_supernodes(distance_matrix, start_end_combinations)
    print("Chosen Pairs:", chosen_pairs)

    # Step 4: Extract unique supernodes based on the TSP result
    seen_supernodes = set()
    final_tsp_order = []
    final_chosen_pairs = []

    for idx in tsp_order:
        supernode_index = idx // 4  # Determine which supernode this pair belongs to
        if supernode_index not in seen_supernodes:
            seen_supernodes.add(supernode_index)
            final_tsp_order.append(supernode_index)
            final_chosen_pairs.append(chosen_pairs[idx % len(chosen_pairs)])


    print(final_tsp_order)
    # Step 5: Sort polygons and intersections based on the final TSP order
    sorted_polygons = [polygons[i] for i in final_tsp_order]
    sorted_intersections = [intersections[i] for i in final_tsp_order]

    return sorted_polygons, sorted_intersections, final_chosen_pairs


def plot_tsp_graph(start_end_combinations):
    # Create a graph
    G = nx.Graph()

    # Function to arrange sub-nodes in a circular pattern around a given center position
    def circular_layout(center, radius, num_points):
        # Start from -π/4 radians (which is -45 degrees) for a 45-degree clockwise rotation
        angles = np.linspace(-np.pi / 4, 2 * np.pi - np.pi / 4, num_points, endpoint=False)
        return [center + radius * np.array([np.cos(angle), np.sin(angle)]) for angle in angles]

    # Arrange super nodes in a large circular pattern
    def super_node_circular_layout(num_nodes, radius):
        angles = np.linspace(0, 2 * np.pi, num_nodes, endpoint=False)
        return [radius * np.array([np.cos(angle), np.sin(angle)]) for angle in angles]

    # Add super nodes and edges without specifying positions yet
    for idx, current_start_end_combinations in enumerate(start_end_combinations):
        # Create a super node for the polygon
        super_node = f"Polygon_{idx}"
        G.add_node(super_node, color='red')

        # Add 4 sub-nodes: first, second, third, fourth
        sub_node_labels = ["first", "second", "third", "fourth"]
        for pair_idx, label in enumerate(sub_node_labels):
            # Create unique node labels based on polygon and label
            sub_node = f"P{idx}_{label}"

            # Add sub-nodes
            G.add_node(sub_node, color='blue')

            # Connect each sub-node to the super node
            G.add_edge(super_node, sub_node)

    # Connect super nodes sequentially
    for i in range(len(start_end_combinations) - 1):
        G.add_edge(f"Polygon_{i}", f"Polygon_{i + 1}", color='green')

    # Connect all super nodes to each other (complete graph)
    super_node_labels = [f"Polygon_{i}" for i in range(len(start_end_combinations))]
    for i in range(len(super_node_labels)):
        for j in range(i + 1, len(super_node_labels)):
            G.add_edge(super_node_labels[i], super_node_labels[j], color='purple')

    # Arrange super nodes in a large circular layout
    super_node_positions = super_node_circular_layout(len(start_end_combinations), radius=0.4)
    positions = {f"Polygon_{idx}": pos for idx, pos in enumerate(super_node_positions)}

    # Manually arrange sub-nodes in a circular pattern around their respective super nodes
    for idx, current_start_end_combinations in enumerate(start_end_combinations):
        # Position of the super node
        super_node = f"Polygon_{idx}"
        center = positions[super_node]

        # Arrange sub-nodes around the super node in a circular layout
        sub_node_positions = circular_layout(center, radius=0.08, num_points=4)

        # Define positions for the 4 labeled sub-nodes
        for pair_idx, label in enumerate(sub_node_labels):
            sub_node = f"P{idx}_{label}"
            positions[sub_node] = sub_node_positions[pair_idx]

    # Extract colors for plotting
    node_colors = [G.nodes[node].get('color', 'blue') for node in G.nodes]
    edge_colors = ['purple' if G[u][v].get('color') == 'purple' else
                   'green' if G[u][v].get('color') == 'green' else
                   'gray' for u, v in G.edges()]

    # Draw the graph
    plt.figure(figsize=(12, 8))
    nx.draw(G, positions, with_labels=True, node_color=node_colors, node_size=500, font_size=8, font_weight='bold',
            edge_color=edge_colors)
    plt.title("MC-TSP Graph with Circular Super Nodes and Complete Super Node Connections")
    plt.show()



