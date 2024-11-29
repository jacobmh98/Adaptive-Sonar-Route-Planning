from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import numpy as np

# Function to compute centroid of a polygon
def compute_centroid(polygon):
    x_coords = [v.x for v in polygon.vertices]
    y_coords = [v.y for v in polygon.vertices]
    centroid = (np.mean(x_coords), np.mean(y_coords))
    return centroid


# Function to compute the distance between two polygons (based on centroid)
def compute_distance(polygon1, polygon2):
    centroid1 = compute_centroid(polygon1)
    centroid2 = compute_centroid(polygon2)
    return np.linalg.norm(np.array(centroid1) - np.array(centroid2))


# Function to create the distance matrix for the polygons
def create_distance_matrix(polygons):
    n = len(polygons)
    distance_matrix = np.zeros((n, n))
    for i in range(n):
        for j in range(i + 1, n):
            distance = compute_distance(polygons[i], polygons[j])
            distance_matrix[i][j] = distance
            distance_matrix[j][i] = distance  # For symmetric matrix
    return distance_matrix


# Solve the TSP using Google OR-Tools
def solve_tsp(distance_matrix):
    tsp_size = len(distance_matrix)
    num_routes = 1
    depot = 0  # Starting node

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(tsp_size, num_routes, depot)

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Define cost of each edge (distance matrix)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic (you can try different strategies)
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Extract the optimal route from the solution
    if solution:
        route = []
        index = routing.Start(0)
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        return route
    else:
        return None


def solve_centroid_tsp(polygons, intersections):
    distance_matrix = create_distance_matrix(polygons)
    tsp_route = solve_tsp(distance_matrix)
    sorted_polys = [polygons[i] for i in tsp_route]
    sorted_non_removed_polys = [polygons[i] for i in tsp_route]
    sorted_inters = [intersections[i] for i in tsp_route]

    return sorted_polys, sorted_non_removed_polys, sorted_inters

