import copy
import json

import connecting_path
import coverage_plots
import multi_poly_planning
import optimal_path
import path_comparison_functions
import traveling_salesman_variation
import pickle
import traceback
from global_variables import *
from functions import *
import pickle
from global_variables import load_existing_data

if not load_existing_data:
    # Reading the test data
    f = open(f'test_data/{data_path}.json')

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
    with open(f'C:/Users/jacob/Documents/GitHub/Adaptive-Sonar-Route-Planning/test_data/{name_decomposition}.pkl',
              'wb') as file:
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
#antwerp_poly.plot()
#plot_results3(sub_polygons)

if not load_existing_optimized_sub_polygons:
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
    with open(
            f'C:/Users/jacob/Documents/GitHub/Adaptive-Sonar-Route-Planning/test_data/{name_optimized_decomposition}.pkl',
            'wb') as file:
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

# Remove collinear vertices in each sub-polygon
for i, p in enumerate(optimized_sub_polygons):
    p = remove_collinear_vertices(p)
    optimized_sub_polygons[i] = p

plot_results3(optimized_sub_polygons)
plot_graph(optimized_sub_polygons)

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
    #plot_graph(adjacency_graph)
else:  # Unsorted polygons
    polygons = optimized_sub_polygons

print(f'Number of polygons = {len(polygons)}')

# Computing optimal path width given start path width and a tolerance (+- to path width)
if find_optimal_path_width:
    print(f'Optimal path:')
    path, distance, path_width = optimal_path.compute_optimal_path_width(optimized_sub_polygons, path_width, tolerance, iterations)
    print(f'Found optimal path width: {path_width}')

    if check_reverse:
        path, distance = optimal_path.compute_reverse(optimized_sub_polygons, path_width)

# Check if computing path using reversed sorted polygon list provides a better result
elif check_reverse:
    path, distance = optimal_path.compute_reverse(optimized_sub_polygons, path_width)

# Baseline path
else:
    print('Baseline path:')
    intersections = multi_poly_planning.multi_intersection_planning(optimized_sub_polygons, path_width)
    path = connecting_path.connect_path(optimized_sub_polygons, intersections)
    distance = path_comparison_functions.compute_total_distance(path)

# Computing turns in the path
total_turns, hard_turns, medium_turns, soft_turns = path_comparison_functions.calculate_turns_and_classify(path)

print(f'Distance: {distance}')
print(f'Total turns: {total_turns}')
print(f'Hard turns (<45): {hard_turns}')
print(f'Medium turns (45-90): {medium_turns}')
print(f'Soft turns (>90): {soft_turns}')

coverage_plots.multi_poly_plot(None, path_width, optimized_sub_polygons, path)