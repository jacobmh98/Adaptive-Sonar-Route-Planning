import copy
import time
import json
import connecting_path
import coverage_plots
import intra_regional_tsp
import multi_poly_planning
#import multi_poly_planning
import optimal_path
import path_comparison_functions
from intra_regional_tsp import *
import random

import traveling_salesman_variation
import traceback
from global_variables import *
from decomposition import *
from plot_functions import *
import pickle
from global_variables import load_existing_data
from obstacles import *
from load_data import *

# TODO: Move or change when finalizing how hard edges are made
def extract_hard_edges(polygons, hard_edges):
    all_hard_edges = []
    for poly_index, poly in enumerate(polygons):
        if poly_index < len(hard_edges):
            edges = hard_edges[poly_index]
            x_coords, y_coords = poly.get_coords()
            for edge_index in edges:
                start_idx = edge_index
                end_idx = (edge_index + 1) % len(x_coords)

                hard_edge_start = np.array([x_coords[start_idx], y_coords[start_idx]])
                hard_edge_end = np.array([x_coords[end_idx], y_coords[end_idx]])

                all_hard_edges.append((hard_edge_start, hard_edge_end))

    return all_hard_edges

data_path = 'complex_polygon'
region, obstacles = get_region(data_path)
sub_polygons = generate_new_data(region)
#optimized_sub_polygons = compute_optimized_data(sub_polygons)
sub_polygons_filtered_mask, sub_polygons_filtered = find_bounding_polygons(sub_polygons, obstacles[0])
sub_polygons_extract, merged_sub_polygon = merge_filtered_sub_polygons(copy.deepcopy(sub_polygons_filtered), copy.deepcopy(sub_polygons), sub_polygons_filtered_mask)
merged_sub_polygon_decomposed = decompose_sweep_line(merged_sub_polygon, obstacles[0])
#combined_polygons = sub_polygons_extract + merged_sub_polygon_decomposed

"""hard_edges = []
for i, poly in enumerate([region]):
    hard_edges_poly = []
    for j, edge in enumerate(poly.edges):
        if edge.is_hard_edge:
            hard_edges_poly.append(edge)
    hard_edges.append(hard_edges_poly)

print(hard_edges)"""
#hard_edges_list = extract_hard_edges(combined_polygons, hard_edges_manuel)

plot_obstacles([region], obstacles, True)
plot_obstacles(sub_polygons, obstacles, False)
plot_obstacles([merged_sub_polygon], obstacles, True)
plot_obstacles(merged_sub_polygon_decomposed, obstacles, False)

print(obstacles[0].edges)
print(obstacles[0].vertices)

#for p in sub_polygons:
#    plot_obstacles([p], [], True)

#plot_obstacles(combined_polygons, obstacles, False)

quit()
intersections = multi_poly_planning.multi_intersection_planning(sub_polygons, path_width)
path = connecting_path.connect_path(sub_polygons, intersections, [])
coverage_plots.multi_poly_plot(region, path_width, sub_polygons, path,[])

quit()
#intersections = multi_poly_planning.multi_intersection_planning(sub_polygons_obstacles, path_width)
#path = connecting_path.connect_path(sub_polygons_obstacles, intersections)
#coverage_plots.multi_poly_plot(region, path_width, sub_polygons_obstacles, path)


quit()
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




