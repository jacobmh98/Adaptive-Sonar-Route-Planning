import copy
import json
import math
import multi_poly_planning
import traveling_salesman_variation
from functions import *
import pickle

from global_variables import load_existing_data

if not load_existing_data:
    # Reading the test data
    f = open('test_data/antwerpen_full.json')
    data = json.load(f)
    vertices_data = data['area']['coordinates']

    # Defining the initial polygon and the boúnding box
    vertices = []

    for i in range(len(vertices_data)):
        vertices.append(Vertex(i, vertices_data[i][0], vertices_data[i][1]))

    P = Polygon(vertices)
    
# Use DFS to order the sub-polygons based on adjacency
def sort_sub_polygons_using_dfs(G, polygons, start):
    """
    :param G: nx graph
    :param polygons: List of polygons
    :param start: string, start node
    :return sorted_polys: The sorted list of polygons
    """
    # Perform DFS on the graph starting from the specified start node
    dfs_order = list(nx.dfs_preorder_nodes(G, start))

    # Convert the node labels back to polygon indices
    dfs_order_indices = [int(node[1:]) for node in dfs_order]

    # Get the DFS-based order of polygons
    ordered_polys = dfs_order_indices

    # Reorder the polygons based on the DFS traversal order
    sorted_polys = [polygons[i] for i in ordered_polys]

    return sorted_polys

def create_adjacency_matrix(polygons):
    # Creating adjacent matrix for the sub-polygons to tell which sub-polygons are connected
    m = len(polygons)
    A = np.zeros((m, m))
    G = nx.Graph()

    # Go through each edge in p_i and compare with each edge in p_j
    for i, p_i in  enumerate(polygons):
        for j, p_j in enumerate(polygons):
            # Ignore if the two polygons are equal
            if i == j:
                A[i, j] = 0
                continue

            # Test if the two polygons p_i and p_j are adjacent (either complete or partial)
            if polygons_are_adjacent(p_i, p_j, i, j):
                # Update the adjacent matrix
                A[i, j] = 1
                A[j, i] = 1
                G.add_edge(f'P{i}', f'P{j}')
            else:
                A[i, j] = np.inf
                #print(f'{i} and {j} are adjacent')
    return A,G

# Reading the test data
f = open('test_data/complex_polygon.json')
f1 = open('test_data/simple_triangle.json')
f2 = open('test_data/simple_skewed_rectangle.json')
f3 = open('test_data/simple_pentagon.json')
data = json.load(f)
vertices_data = data['area']['coordinates']
P = create_polygon(vertices_data)

    # Compute the split that gives the sub-polygons
    print('running')
    sub_polygons = split_polygon(P)

    # Save the sub polygon objects
    with open('C:/Users/jacob/Documents/GitHub/Adaptive-Sonar-Route-Planning/test_data/antwerpen.pkl', 'wb') as file:
        pickle.dump(sub_polygons, file)
# Start parameters
dx = 0.3 # Path width (Must be >0)
extern_start_end = False
if extern_start_end:
    p_start = [0.0, 0.0]
    p_end = [7, 6]
else:
    with open('./test_data/antwerpen.pkl', 'rb') as file:
        sub_polygons = pickle.load(file)

# Optimizing the sub-polygons
optimize_sub_polygons = optimize_polygons(copy.deepcopy(sub_polygons))

# Plotting the sub-polygons
plot_results3(optimize_sub_polygons)


# TODO har ikke testet om den stadig laver den korrekte adjacent matrix. Men tror stadig det virker
# Creating adjacent matrix for the sub-polygons to tell which sub-polygons are connected
"""m = len(sub_polygons)
A = np.zeros((m, m))
G = nx.Graph()

# Go through each edge in p_i and compare with each edge in p_j
for i, p_i in  enumerate(sub_polygons):
    for j, p_j in enumerate(sub_polygons):
        # Ignore if the two polygons are equal
        if i == j:
            A[i, j] = 0
            continue

        # Test if the two polygons p_i and p_j are adjacent (either complete or partial)
        if polygons_are_adjacent(p_i, p_j, i, j):
            # Update the adjacent matrix
            A[i, j] = 1
            A[j, i] = 1
            G.add_edge(f'P{i}', f'P{j}')
        else:
            A[i, j] = np.inf

plot_graph(G)"""
# Order the list of sub polygons
start_node = 'P0'  # TODO: Use top of adjacency graph
adjacency_matrix, adjacency_graph = create_adjacency_matrix(sub_polygons)
#functions.plot_polygons(P, sub_polygons, adjacency_graph)
sorted_polygons = sort_sub_polygons_using_dfs(adjacency_graph, sub_polygons, start_node)

#sorted_polygons = sub_polygons

total_path = multi_poly_planning.multi_path_planning(sorted_polygons, dx, extern_start_end, p_start, p_end)
print(f'Path distance = {path_distance(total_path)}')
multi_poly_planning.multi_poly_plot(sorted_polygons, dx, extern_start_end, p_start, p_end, np.array(total_path))


quit()
# For tsp
distance_matrix = traveling_salesman_variation.create_distance_matrix(sub_polygons)
tsp_route = traveling_salesman_variation.solve_tsp(distance_matrix)
#traveling_salesman_variation.visualize_tsp_solution(sub_polygons, tsp_route)

# Sort the polygons according to the TSP route
sorted_polygons = [sub_polygons[i] for i in tsp_route]
#sorted_polygons = [sub_polygons[5]]
total_path = multi_poly_planning.multi_path_planning(sorted_polygons, dx, extern_start_end, p_start, p_end)
print(path_distance(total_path))
multi_poly_planning.multi_poly_plot(sorted_polygons, dx, extern_start_end, p_start, p_end, np.array(total_path))
