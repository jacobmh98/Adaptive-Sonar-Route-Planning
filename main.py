import coverage_plots
import multi_poly_planning
import path_comparison_functions
import pickle
from Polygon import Polygon
from global_variables import *
import time
import functions

from collections import defaultdict

antwerp = False
if antwerp:
    with open('test_data/antwerp_comparison.pkl', 'rb') as file:
        optimized_polygons = pickle.load(file)

else:
    with open('test_data/simple_complex_comparison.pkl', 'rb') as file:
        optimized_polygons = pickle.load(file)

    with open("test_data/region_complex.pkl", "rb") as file:  # Open file in binary read mode
        region = pickle.load(file)

if dfs_sort:
    # Order the list of sub polygons
    adjacency_matrix, adjacency_graph = multi_poly_planning.create_adjacency(optimized_polygons)
    start_node = next(iter(adjacency_graph.nodes()))
    functions.plot_graph(adjacency_graph)
    polygons = multi_poly_planning.sort_sub_polygons_using_dfs(adjacency_graph, optimized_polygons, start_node)

else:  # Unsorted polygons
    polygons = optimized_polygons

start_time = time.time()
total_path = multi_poly_planning.multi_path_planning(polygons, path_width, extern_start_end)
end_time = time.time()
elapsed_time = end_time - start_time

path_comparison_functions.compute_path_data(region, total_path, elapsed_time)
coverage_plots.multi_poly_plot(region, path_width, polygons, total_path)
