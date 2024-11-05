import copy
import json
import multi_poly_planning
import path_comparison_functions
import pickle
import traceback
from global_variables import *
import numpy as np
import time

antwerp = False
if antwerp:
    with open('test_data/antwerp_comparison.pkl', 'rb') as file:
        optimized_polygons = pickle.load(file)

else:
    with open('test_data/simple_complex_comparison.pkl', 'rb') as file:
        optimized_polygons = pickle.load(file)

# Plotting the sub-polygons

if dfs_sort:
    # Order the list of sub polygons
    adjacency_matrix, adjacency_graph = multi_poly_planning.create_adjacency(optimized_polygons)
    start_node = next(iter(adjacency_graph.nodes()))
    #functions.plot_graph(adjacency_graph)
    polygons = multi_poly_planning.sort_sub_polygons_using_dfs(adjacency_graph, optimized_polygons, start_node)

else:  # Unsorted polygons
    polygons = optimized_polygons

start_time = time.time()
total_path = multi_poly_planning.multi_path_planning(polygons, dx, extern_start_end)
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
print()

multi_poly_planning.multi_poly_plot(optimized_polygons, polygons, dx, extern_start_end, ext_p_start, ext_p_end, total_path)
