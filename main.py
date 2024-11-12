import copy
import time
import json

from shapely.creation import polygons

import connecting_path
import coverage_plots
import decomposition
import intra_regional_tsp
import multi_poly_planning
import optimal_path
import path_comparison_functions
#import intra_regional_tsp
import intra_regional_tsp_networkx
import random
from collections import Counter
import traveling_salesman_variation
import traceback
from global_variables import *
from decomposition import *
from plot_functions import *
import pickle
from global_variables import load_existing_data
from obstacles import *
from load_data import *


# Loading the region and obstacles
data_path = 'C:/Users/jacob/Documents/GitHub/Adaptive-Sonar-Route-Planning/test_data/complex_polygon.json'

region, obstacles = get_region(data_path)
plot_obstacles([region], obstacles, False)

# Decompose the region using the sweep line algorithm
sub_polygons_sweep_line = decompose_sweep_line(copy.deepcopy(region), copy.deepcopy(obstacles))

#sub_polygons_sweep_line = decompose_sweep_line(copy.deepcopy(region), copy.deepcopy(obstacles))
#plot_obstacles(sub_polygons_sweep_line, obstacles, False)

# Decompose the region without considering obstacles
sub_polygons = generate_new_data(copy.deepcopy(region))

#%% Plot the decomposed sub_polygons without considered obstacles
#plot_obstacles(sub_polygons, obstacles, False)

# Divide the sub-polygons into clusters that are affected by the obstacles
sub_polygons_filtered_masks = []
sub_polygons_filtered = []
obstacles_affected = []

for o in obstacles:
    filtered_mask, filtered = find_bounding_polygons(sub_polygons, o)
    common_found = False

    #plot_obstacles(filtered, obstacles, False)

    for index, mask in enumerate(sub_polygons_filtered_masks):
        for i in mask:
            for j in filtered_mask:
                if i == j:
                    common_found = True
                    break

            if common_found:
                break

        if common_found:
            for i, p in enumerate(filtered_mask):
                if p not in mask:
                    mask.append(p)
                    sub_polygons_filtered[index].append(filtered[i])

                    if o not in obstacles_affected[index]:
                        obstacles_affected[index].append(o)

    if not common_found:
        sub_polygons_filtered_masks.append(filtered_mask)
        sub_polygons_filtered.append(filtered)
        obstacles_affected.append([o])

# Merge each cluster into a single polygon and decompose it using sweep line algorithm
decomposed_polygons = []
extracted_sub_polygons = []
extracted_sub_polygons_mask = []
dont_include_mask = []

for list in sub_polygons_filtered_masks:
    for p in list:
        dont_include_mask.append(p)

for i, filtered in enumerate(sub_polygons_filtered):
    sub_polygons_extract, merged_sub_polygon = merge_filtered_sub_polygons(copy.deepcopy(filtered), copy.deepcopy(sub_polygons), sub_polygons_filtered_masks[i])

    merged_sub_polygon_decomposed = decompose_sweep_line(merged_sub_polygon, obstacles_affected[i])
    decomposed_polygons += merged_sub_polygon_decomposed

    #plot_obstacles(merged_sub_polygon_decomposed, obstacles, False)

    for p in sub_polygons_extract:
        if p not in extracted_sub_polygons_mask and p not in dont_include_mask:
            extracted_sub_polygons_mask.append(p)
            extracted_sub_polygons.append(sub_polygons[p])

    plot_obstacles(extracted_sub_polygons + [merged_sub_polygon], obstacles, False)

# Combining all the decomposed sub-polygons with obstacles
combined_polygons = sub_polygons#extracted_sub_polygons + decomposed_polygons

# Optimize the sub-polygons by merging when possible
#optimized_sub_polygons = compute_optimized_data(combined_polygons)
plot_obstacles(combined_polygons, obstacles, False)


#%% Plot the sub_polygons computed only with the sweep line algorithm
#plot_obstacles(sub_polygons_sweep_line, obstacles, False)
#%% Plot the region and obstacles
#plot_obstacles([region], obstacles, False)

#%% Plot the optimized sub_polygons without considered obstacles
#plot_obstacles(optimized_sub_polygons, obstacles)

#%% Plot the sub_polygons while considered obstacles
#plot_obstacles(sub_polygons_extract + merged_sub_polygon_decomposed, obstacles, False)
#plot_obstacles(combined_polygons, obstacles, False)

"""sum_combined = 0
for i, p in enumerate(combined_polygons):
    sum_combined += min_polygon_width(p.vertices_matrix())

print(f'{sum_combined=}')

sum_sweep_line = 0
for i, p in enumerate(sub_polygons_sweep_line):
    sum_sweep_line += min_polygon_width(p.vertices_matrix())

print(f'{sum_sweep_line=}')"""

#adjacency_matrix, adjacency_graph = multi_poly_planning.create_adjacency(optimized_sub_polygons)
#start_node = next(iter(adjacency_graph.nodes()))
#optimized_sub_polygons = multi_poly_planning.sort_sub_polygons_using_dfs(adjacency_graph, optimized_sub_polygons, start_node)

distance_matrix = traveling_salesman_variation.create_distance_matrix(combined_polygons)
tsp_route = traveling_salesman_variation.solve_tsp(distance_matrix)
traveling_salesman_variation.visualize_tsp_solution(combined_polygons, tsp_route)
optimized_sub_polygons = [combined_polygons[i] for i in tsp_route]


#plot_obstacles(sub_polygons, obstacles)
#asd(sub_polygons[0], obstacles[0])
# TODO obstruction starts here
#plot_results4(optimized_sub_polygons, obstructions)
#plot_results3(optimized_sub_polygons)
#plot_graph(optimized_sub_polygons)

# Starting timer for all cpp functions
total_start_time = time.time()
intersections = multi_poly_planning.multi_intersection_planning(combined_polygons, path_width)

# Choosing sorting method
if dfs_sorting:
    print("DFS Sorting")
    combined_polygons = multi_poly_planning.sort_sub_polygons_using_dfs(combined_polygons)

elif tsp_centroid_sorting:
    print("TSP Centroid Sorting")
    combined_polygons = traveling_salesman_variation.solve_centroid_tsp(combined_polygons)

elif tsp_intra_regional_sorting:
    print("TSP Intra Regional Sorting")
    combined_polygons = intra_regional_tsp.solve_intra_regional_tsp(combined_polygons, intersections)


print(f'Number of polygons = {len(combined_polygons)}')

# Computing optimal path width given start path width and a tolerance (+- to path width)
#if find_optimal_path_width:
#    path, distance, path_width = optimal_path.compute_optimal_path_width(combined_polygons, path_width, tolerance, iterations)

# Check if computing path using reversed sorted polygon list provides a better result
#elif check_reverse:
#    path, distance = optimal_path.compute_reverse(combined_polygons, path_width)

# Baseline path
#else:
print('Baseline path:')
intersections = multi_poly_planning.multi_intersection_planning(combined_polygons, path_width)
path = connecting_path.connect_path(combined_polygons, intersections, region)
distance = path_comparison_functions.compute_total_distance(path)

# Ending timer and computing total execution time
total_end_time = time.time()
total_execution_time = total_end_time - total_start_time

if get_path_data:
    path_comparison_functions.compute_path_data(region, path, total_execution_time)

coverage_plots.multi_poly_plot(region, path_width, combined_polygons, path)
