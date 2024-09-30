import copy
import json
import multi_poly_planning
import traveling_salesman_variation
from functions import *
import pickle
import traceback
from global_variables import load_existing_data

if not load_existing_data:
    # Reading the test data
    f = open('test_data/antwerpen_full.json')
    data = json.load(f)
    vertices_data = data['area']['coordinates']

    # Defining the initial polygon and the boúnding box
    vertices = []

    # Compute the split that gives the sub-polygons
    print('running')
    sub_polygons = split_polygon(P)

    # Save the sub polygon objects
    with open('C:/Users/jacob/Documents/GitHub/Adaptive-Sonar-Route-Planning/test_data/antwerpen.pkl', 'wb') as file:
        pickle.dump(sub_polygons, file)
else:
    print('load')
    with open('./test_data/antwerpen.pkl', 'rb') as file:
        sub_polygons = pickle.load(file)

    f = open('test_data/antwerpen_full.json')
    data = json.load(f)
    vertices_data = data['area']['coordinates']

    # Defining the initial polygon and the boúnding box
    vertices = []

    for i in range(len(vertices_data)):
        vertices.append(Vertex(i, vertices_data[i][0], vertices_data[i][1]))

    antwerp_poly = Polygon(vertices)


# Optimizing the sub-polygons
optimize_sub_polygons = optimize_polygons(copy.deepcopy(sub_polygons))

# Plotting the sub-polygons
#plot_results3(optimize_sub_polygons)

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




# Start parameters
dx = 22 # Path width (Must be >0) (1 is 1 meter)
tsp_sort = False
dfs_sort = True
extern_start_end = False
if extern_start_end:
    p_start = [0.0, 0.0]
    p_end = [7, 6]
else:
    p_start = None
    p_end = None

# Choosing sorting method for the order of sub polygons
if tsp_sort:
    distance_matrix = traveling_salesman_variation.create_distance_matrix(optimize_sub_polygons)
    tsp_route = traveling_salesman_variation.solve_tsp(distance_matrix)
    #traveling_salesman_variation.visualize_tsp_solution(sub_polygons, tsp_route)
    sorted_polygons = [sub_polygons[i] for i in tsp_route]

elif dfs_sort:
    # Order the list of sub polygons
    start_node = 'P0'  # TODO: Use top of adjacency graph
    adjacency_matrix, adjacency_graph = create_adjacency_matrix(optimize_sub_polygons)
    #functions.plot_graph(adjacency_graph)
    sorted_polygons = sort_sub_polygons_using_dfs(adjacency_graph, optimize_sub_polygons, start_node)

else:  # Unsorted polygons
    sorted_polygons = optimize_sub_polygons

print(f'num_polygons = {len(sorted_polygons)}')


def remove_unnecessary_vertices(polygon, epsilon=1e-9):
    """Remove vertices that form a straight line with their neighbors.

    :param polygon: Polygon object
    :param epsilon: Tolerance for floating point comparisons
    :return: A new polygon with unnecessary vertices removed
    """
    new_vertices = []
    for i in range(len(polygon.vertices)):
        v_prev = polygon.vertices[i - 1]
        v_curr = polygon.vertices[i]
        v_next = polygon.vertices[(i + 1) % len(polygon.vertices)]

        # Vector from previous vertex to current vertex
        vector1 = np.array([v_curr.x - v_prev.x, v_curr.y - v_prev.y, 0])  # Add 3rd dimension as 0
        # Vector from current vertex to next vertex
        vector2 = np.array([v_next.x - v_curr.x, v_next.y - v_curr.y, 0])  # Add 3rd dimension as 0

        # Compute the cross product and only take the z-component
        cross_product = np.cross(vector1, vector2)[-1]

        # If cross product is not zero (within epsilon), the vertex is necessary
        if abs(cross_product) > epsilon:
            new_vertices.append(v_curr)

    # Return a new polygon with filtered vertices
    return Polygon(new_vertices)

complete_path = np.empty((0,2))

for i, poly in enumerate(sorted_polygons):
    new_polygon = [sorted_polygons[i]]

    try:
        new_polygon = [remove_unnecessary_vertices(new_polygon[0])]  # Removing parallel vertices, minimizing number of paths generated
        total_path = multi_poly_planning.multi_path_planning(new_polygon, dx, extern_start_end, p_start, p_end)
        complete_path = np.vstack([complete_path, total_path])
        #multi_poly_planning.multi_poly_plot(new_polygon, dx, extern_start_end, p_start, p_end, np.array(total_path))
        print(f'{i} is working')

    except Exception as e:
        print(f'not working on {i}')
        traceback.print_exc()
        new_polygon[0].plot()
        break

multi_poly_planning.multi_poly_plot(antwerp_poly, dx, extern_start_end, p_start, p_end, np.array(complete_path))

