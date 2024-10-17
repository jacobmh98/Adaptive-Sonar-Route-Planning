import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import matplotlib.pyplot as plt


def get_start_end_combinations(intersections):
    first_intersection = intersections[0]
    last_intersection = intersections[-1]

    # All possible combinations of start and end points
    start_end_combinations = [
        (first_intersection[0], last_intersection[0]),  # First point to first point
        (first_intersection[0], last_intersection[1]),  # First point to second point
        (first_intersection[1], last_intersection[0]),  # Second point to first point
        (first_intersection[1], last_intersection[1])   # Second point to second point
    ]
    return start_end_combinations


def create_expanded_distance_matrix(polygons, total_intersections):
    # Each polygon has 4 potential start/end combinations, so the matrix is 4x larger
    num_combinations = len(polygons) * 4
    distance_matrix = np.zeros((num_combinations, num_combinations))

    # Loop through each pair of polygons and compute the distance between each combination
    for i, poly1 in enumerate(polygons):
        combinations1 = get_start_end_combinations(total_intersections[i])

        for j, poly2 in enumerate(polygons):
            if i != j:  # Skip self-connections
                combinations2 = get_start_end_combinations(total_intersections[j])

                # Calculate distances for each combination pair
                for k, (start1, end1) in enumerate(combinations1):
                    for l, (start2, end2) in enumerate(combinations2):
                        # Compute the distance from the end of polygon i's combination to the start of polygon j's combination
                        distance_matrix[i*4 + k][j*4 + l] = np.linalg.norm(end1 - start2)

    return distance_matrix


def tsp_solver_with_combinations(distance_matrix, num_polygons):
    # Create the TSP manager for the expanded matrix (4x nodes)
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, 0)  # 1 vehicle, start at index 0

    routing = pywrapcp.RoutingModel(manager)

    # Define cost of each arc using the expanded distance matrix
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add constraint to ensure only one node per polygon is selected
    for i in range(num_polygons):
        routing.AddDisjunction([manager.NodeToIndex(i * 4 + k) for k in range(4)], 0)  # Each polygon has 4 nodes

    # Define search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the TSP
    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        return get_solution(manager, routing, solution, num_polygons)
    else:
        return None


def get_solution(manager, routing, solution, num_polygons):
    """Extract the TSP solution in terms of the best start/end combinations."""
    index = routing.Start(0)
    tsp_route = []

    while not routing.IsEnd(index):
        node = manager.IndexToNode(index)
        polygon_index = node // 4  # Each polygon has 4 nodes, so divide by 4 to get the polygon index
        combination_index = node % 4  # Get which combination was chosen
        tsp_route.append((polygon_index, combination_index))  # Store polygon and combination
        index = solution.Value(routing.NextVar(index))

    return tsp_route

def sort_polygons_by_tsp(optimal_route, polygon_intersections):
    reordered_polygons = []
    reordered_intersections = []
    used_indices = set()

    for i in range(0, len(optimal_route), 2):  # Steps of 2 to get both start and end points
        polygon_index = optimal_route[i] // 2

        if polygon_index not in used_indices:
            # Retrieve both polygon and intersections from the dictionary
            data = polygon_intersections[polygon_index]
            reordered_polygons.append(data["polygon"])
            reordered_intersections.append(data["intersections"])
            used_indices.add(polygon_index)

    return reordered_polygons, reordered_intersections


# Function to compute the centroid of a polygon
def compute_centroid(polygon):
    x_coords, y_coords = polygon.get_coords()
    centroid_x = sum(x_coords) / len(x_coords)
    centroid_y = sum(y_coords) / len(y_coords)
    return (centroid_x, centroid_y)

def visualize_tsp_solution(polygons, tsp_route, total_intersections):
    plt.figure(figsize=(8, 8))

    # Plot each polygon with its intersections
    for i, polygon in enumerate(polygons):
        x_coords, y_coords = polygon.get_coords()
        x_coords.append(x_coords[0])  # Close the polygon loop
        y_coords.append(y_coords[0])
        plt.plot(x_coords, y_coords, 'r-', lw=2, marker='o')  # Plot the polygon edges


        # Add label to the centroid of the polygon
        centroid = compute_centroid(polygon)
        plt.text(centroid[0], centroid[1], f'{i}', fontsize=12, ha='center', color='blue')

    # Plot the TSP path between centroids of polygons
    for i in range(len(tsp_route) - 1):
        start_polygon = polygons[tsp_route[i]]
        end_polygon = polygons[tsp_route[i + 1]]

        # Draw line between centroids
        centroid_start = compute_centroid(start_polygon)
        centroid_end = compute_centroid(end_polygon)
        plt.plot([centroid_start[0], centroid_end[0]], [centroid_start[1], centroid_end[1]], 'b--', lw=2)

    # Mark start and end of the path
    start_polygon = polygons[tsp_route[0]]
    end_polygon = polygons[tsp_route[-1]]
    centroid_start = compute_centroid(start_polygon)
    centroid_end = compute_centroid(end_polygon)

    plt.plot(centroid_start[0], centroid_start[1], 'ro', markersize=10, label='Start')  # Red dot for start
    plt.plot(centroid_end[0], centroid_end[1], 'bo', markersize=10, label='End')  # Blue dot for end

    plt.title("TSP Optimal Path Visiting Polygons and Intersections")
    plt.legend()
    plt.show()