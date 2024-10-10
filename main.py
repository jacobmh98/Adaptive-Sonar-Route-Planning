import copy
import json
import os
import pickle
import time
import traceback
import multi_poly_planning
import traveling_salesman_variation
import connecting_path
import coverage_plots
import path_comparison_functions
from global_variables import *
from functions import *

antwerp = True

if antwerp:
    with open('test_data/antwerp_comparison.pkl', 'rb') as file:
        optimized_polygons = pickle.load(file)

else:
    with open('test_data/simple_complex_comparison.pkl', 'rb') as file:
        optimized_polygons = pickle.load(file)

# Computing the total path
start_time = time.time()
total_intersections = multi_poly_planning.multi_intersection_planning(optimized_polygons, extern_start_end)

#multi_poly_planning.multi_poly_plot(poly, polygons, dx, extern_start_end, ext_p_start, ext_p_end, total_path)
total_path = connecting_path.connect_path(optimized_polygons, total_intersections)

end_time = time.time()
elapsed_time = end_time - start_time

# For comparisons
distance = path_comparison_functions.compute_total_distance(total_path)
total_turns, hard_turns, medium_turns, soft_turns = path_comparison_functions.calculate_turns_and_classify(total_path)

print('Newest version:')
print(f'Distance: {distance}')
print(f'Total turns: {total_turns}')
print(f'Hard turns: {hard_turns}')
print(f'Medium turns: {medium_turns}')
print(f'Soft turns: {soft_turns}')
print(f'Elapsed time: {elapsed_time}')

coverage_plots.multi_poly_plot(None, optimized_polygons, total_path)



quit()
"""f = open('test_data/antwerpen_full.json')

    with open('test_data/antwerpen_epsilon_e6_optimized_e2.pkl', 'rb') as file:
        sub_polygons = pickle.load(file)

    data = json.load(f)
    vertices_data = data['area']['coordinates']

    # Defining the initial polygon
    vertices = []

    for i in range(len(vertices_data)):
        vertices.append(Vertex(i, vertices_data[i][0], vertices_data[i][1]))
    poly = Polygon(vertices)
    optimized_polygons = sub_polygons

    # Remove collinear vertices in each sub-polygon
    for i, p in enumerate(optimized_polygons):
        p = remove_collinear_vertices(p)
        optimized_polygons[i] = p

    adjacency_matrix, adjacency_graph = multi_poly_planning.create_adjacency(optimized_polygons)
    start_node = next(iter(adjacency_graph.nodes()))
    optimized_polygons = multi_poly_planning.sort_sub_polygons_using_dfs(adjacency_graph, optimized_polygons, start_node)
"""