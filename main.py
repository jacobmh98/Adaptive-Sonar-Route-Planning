import copy
import time
import json
import connecting_path
import coverage_plots
import intra_regional_tsp
import multi_poly_planning
import optimal_path
import path_comparison_functions
from intra_regional_tsp import *

import traveling_salesman_variation
import traceback
from global_variables import *
from decomposition import *
from plot_functions import *
import pickle
from global_variables import load_existing_data
from obstacles import *
from load_data import *

data_path = 'complex_polygon'
region, obstacles = get_region(data_path)
sub_polygons = generate_new_data(region)
optimized_sub_polygons = compute_optimized_data(sub_polygons)
plot_results3(optimized_sub_polygons)

intersections = multi_poly_planning.multi_intersection_planning(optimized_sub_polygons, path_width)
sorted_polygons, sorted_intersections, chosen_pairs = intra_regional_tsp.start_tsp(optimized_sub_polygons, intersections, region)

quit()

#coverage_plots.multi_poly_plot(region, path_width, optimized_sub_polygons, tsp_route)

#plot_obstacles(sub_polygons, obstacles)
#asd(sub_polygons[0], obstacles[0])
# TODO obstruction starts here
#plot_results4(optimized_sub_polygons, obstructions)
plot_results3(optimized_sub_polygons)
#plot_graph(optimized_sub_polygons)

# Starting timer for all cpp functions
total_start_time = time.time()

# Compute intersections
#intersections = multi_poly_planning.multi_intersection_planning(optimized_sub_polygons, path_width)

# Choosing sorting method
if tsp_sorting:
    #tsp_route = intra_regional_tsp.start_tsp(optimized_sub_polygons, intersections)

    # TODO: Old tsp, use some for new
    """distance_matrix = traveling_salesman_variation.create_distance_matrix(optimized_sub_polygons)
    tsp_route = traveling_salesman_variation.solve_tsp(distance_matrix)
    traveling_salesman_variation.visualize_tsp_solution(optimized_sub_polygons, tsp_route)
    optimized_sub_polygons = [sub_polygons[i] for i in tsp_route]
    """
elif dfs_sorting:  # TODO: Clean further
    # Order the list of sub polygons
    adjacency_matrix, adjacency_graph = multi_poly_planning.create_adjacency(optimized_sub_polygons)
    start_node = next(iter(adjacency_graph.nodes()))
    optimized_sub_polygons = multi_poly_planning.sort_sub_polygons_using_dfs(adjacency_graph, optimized_sub_polygons, start_node)
    #plot_graph(adjacency_graph)

print(f'Number of polygons = {len(optimized_sub_polygons)}')


# Computing optimal path width given start path width and a tolerance (+- to path width)
if find_optimal_path_width:
    path, distance, path_width = optimal_path.compute_optimal_path_width(optimized_sub_polygons, path_width, tolerance, iterations)

# Check if computing path using reversed sorted polygon list provides a better result
elif check_reverse:
    path, distance = optimal_path.compute_reverse(optimized_sub_polygons, path_width)

# Baseline path
else:
    print('Baseline path:')
    intersections = multi_poly_planning.multi_intersection_planning(optimized_sub_polygons, path_width)
    path = connecting_path.connect_path(optimized_sub_polygons, intersections)
    distance = path_comparison_functions.compute_total_distance(path)

coverage_plots.multi_poly_plot(region, path_width, optimized_sub_polygons, path)


# Ending timer and computing total execution time
total_end_time = time.time()
total_execution_time = total_end_time - total_start_time

if get_path_data:
    path_comparison_functions.compute_path_data(region, path, total_execution_time)




