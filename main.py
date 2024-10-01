import copy
import json
import multi_poly_planning
import traveling_salesman_variation
import pickle
import traceback
from global_variables import *
from functions import *

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

    # Defining the initial polygon
    vertices = []

    for i in range(len(vertices_data)):
        vertices.append(Vertex(i, vertices_data[i][0], vertices_data[i][1]))

    antwerp_poly = Polygon(vertices)

# Optimizing the sub-polygons (removing edges)
optimize_sub_polygons = optimize_polygons(copy.deepcopy(sub_polygons))

# Plotting the sub-polygons
#plot_results3(optimize_sub_polygons)


# Choosing sorting method for the order of sub polygons
if tsp_sort:
    distance_matrix = traveling_salesman_variation.create_distance_matrix(optimize_sub_polygons)
    tsp_route = traveling_salesman_variation.solve_tsp(distance_matrix)
    #traveling_salesman_variation.visualize_tsp_solution(sub_polygons, tsp_route)
    polygons = [sub_polygons[i] for i in tsp_route]

elif dfs_sort:
    # Order the list of sub polygons
    adjacency_matrix, adjacency_graph = multi_poly_planning.create_adjacency(optimize_sub_polygons)
    start_node = next(iter(adjacency_graph.nodes()))
    #functions.plot_graph(adjacency_graph)
    polygons = multi_poly_planning.sort_sub_polygons_using_dfs(adjacency_graph, optimize_sub_polygons, start_node)

else:  # Unsorted polygons
    polygons = optimize_sub_polygons

print(f'num_polygons = {len(polygons)}')

complete_path = np.empty((0,2))

# If chosen, parallel vertices are removed (Can cause edge case errors if they are not removed)
if remove_parallel_vertices:
    for i,poly in enumerate(polygons):
        polygons[i] = multi_poly_planning.remove_unnecessary_vertices(poly)


total_path = multi_poly_planning.multi_path_planning(polygons, dx, extern_start_end)
multi_poly_planning.multi_poly_plot(antwerp_poly, polygons, dx, extern_start_end, ext_p_start, ext_p_end, total_path)


"""
for i, poly in enumerate(sorted_polygons):
    new_polygon = [sorted_polygons[i]]

    try:
        new_polygon = [remove_unnecessary_vertices(new_polygon[0])]  # Removing parallel vertices, minimizing number of paths generated
        total_path = multi_poly_planning.multi_path_planning(new_polygon, dx, extern_start_end)
        complete_path = np.vstack([complete_path, total_path])
        #multi_poly_planning.multi_poly_plot(new_polygon, dx, extern_start_end, p_start, p_end, np.array(total_path))
        print(f'{i} is working')

    except Exception as e:
        print(f'not working on {i}')
        traceback.print_exc()
        new_polygon[0].plot()
        break
"""



