import copy
import json
import multi_poly_planning
import traveling_salesman_variation
import pickle
import traceback
from global_variables import *
from functions import *
import pickle
from global_variables import load_existing_data

if not load_existing_data:
    # Reading the test data
    f = open('test_data/antwerpen_full.json')

    data = json.load(f)
    vertices_data = data['area']['coordinates']

    # Defining the initial polygon
    vertices = []

    for i in range(len(vertices_data)):
        vertices.append(Vertex(i, vertices_data[i][0], vertices_data[i][1]))

    antwerp_poly = Polygon(vertices)

    # Compute the split that gives the sub-polygons
    print('running')
    sub_polygons = split_polygon(antwerp_poly)

    # Save the sub polygon objects
    with open(f'C:/Users/jacob/Documents/GitHub/Adaptive-Sonar-Route-Planning/test_data/{name_decomposition}.pkl', 'wb') as file:
        pickle.dump(sub_polygons, file)
else:
    f = open('test_data/antwerpen_full.json')

    print('loading')
    with open(f'./test_data/{name_decomposition}.pkl', 'rb') as file:
        sub_polygons = pickle.load(file)

    data = json.load(f)
    vertices_data = data['area']['coordinates']

    # Defining the initial polygon
    vertices = []

    for i in range(len(vertices_data)):
        vertices.append(Vertex(i, vertices_data[i][0], vertices_data[i][1]))

    antwerp_poly = Polygon(vertices)

if not load_existing_optimized_polygons:
    """# Removing collinear vertices from the sub-polygons
    for i, p in enumerate(sub_polygons):
        sub_polygons[i] = remove_collinear_vertices(p)"""

    # Removing illegal sub-polygons from the unoptimized Antwerpen
    i = 0
    while i < len(sub_polygons):
        p = sub_polygons[i]

        if not is_well_formed(p)[0]:
            sub_polygons.pop(i)
            i -= 1
        i += 1

    # Optimizing the sub-polygons (removing edges)
    optimized_sub_polygons = optimize_polygons(copy.deepcopy(sub_polygons))

    # Save the optimized sub polygon objects
    with open(f'C:/Users/jacob/Documents/GitHub/Adaptive-Sonar-Route-Planning/test_data/{name_optimized_decomposition}.pkl', 'wb') as file:
        pickle.dump(optimized_sub_polygons, file)
else:
    """# Removing collinear vertices from the sub-polygons
    for i, p in enumerate(sub_polygons):
        sub_polygons[i] = remove_collinear_vertices(p)"""

    # Removing illegal sub-polygons from the unoptimized Antwerpen
    i = 0
    while i < len(sub_polygons):
        p = sub_polygons[i]

        if not is_well_formed(p)[0]:
            sub_polygons.pop(i)
            i -= 1
        i += 1

    with open(f'./test_data/{name_optimized_decomposition}.pkl', 'rb') as file:
        optimized_sub_polygons = pickle.load(file)

#plot_results3(sub_polygons)
#plot_results3(optimized_sub_polygons)

"""collection = []
for i, p in enumerate(sub_polygons):
    if i == 179 or i == 180 or i == 196 or i == 197 or i == 198 or i == 199 or i == 200:
        collection.append(p)

optimized_collection = optimize_polygons(copy.deepcopy(collection))
plot_results3(collection)
plot_results3(optimized_collection)"""


# Choosing sorting method for the order of sub polygons
if tsp_sort:  # Not working correctly
    distance_matrix = traveling_salesman_variation.create_distance_matrix(optimized_sub_polygons)
    tsp_route = traveling_salesman_variation.solve_tsp(distance_matrix)
    traveling_salesman_variation.visualize_tsp_solution(optimized_sub_polygons, tsp_route)
    polygons = [sub_polygons[i] for i in tsp_route]

elif dfs_sort:
    # Order the list of sub polygons
    adjacency_matrix, adjacency_graph = multi_poly_planning.create_adjacency(optimized_sub_polygons)
    start_node = next(iter(adjacency_graph.nodes()))
    polygons = multi_poly_planning.sort_sub_polygons_using_dfs(adjacency_graph, optimized_sub_polygons, start_node)

else:  # Unsorted polygons
    polygons = optimized_sub_polygons

# If chosen, parallel vertices are removed (Can cause edge case errors if they are not removed)
if remove_parallel_vertices:
    for i,poly in enumerate(polygons):
        polygons[i] = multi_poly_planning.remove_unnecessary_vertices(poly)

print(f'Number of polygons = {len(polygons)}')

# Computing the total path
total_path = multi_poly_planning.multi_path_planning(polygons, dx, extern_start_end)
multi_poly_planning.multi_poly_plot(antwerp_poly, polygons, dx, extern_start_end, ext_p_start, ext_p_end, total_path)


quit()
# Below is for testing path planning for each polygon seperatly
complete_path = np.empty((0,2))
for i, poly in enumerate(polygons):
    new_polygon = [polygons[i]]

    try:
        total_path = multi_poly_planning.multi_path_planning(polygons, dx, extern_start_end)
        complete_path = np.vstack([complete_path, total_path])
        #multi_poly_planning.multi_poly_plot(new_polygon, polygons, dx, extern_start_end, ext_p_start, ext_p_end, np.array(total_path))
        print(f'{i} is working')

    except Exception as e:
        print(f'not working on {i}')
        traceback.print_exc()
        #new_polygon[0].plot()

multi_poly_planning.multi_poly_plot(antwerp_poly, polygons, dx, extern_start_end, ext_p_start, ext_p_end, complete_path)
