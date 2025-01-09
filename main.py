import coverage_plots
import multi_poly_planning
import path_comparison_functions
import pickle
from global_variables import *
import time
import functions
import plot_cpp_v1
import path_data_v1

#antwerp = False
#if antwerp:
#    with open('test_data/antwerp_comparison.pkl', 'rb') as file:
#        optimized_polygons = pickle.load(file)

#else:
#   with open('test_data/simple_complex_comparison.pkl', 'rb') as file:
#        optimized_polygons = pickle.load(file)

#    with open("test_data/region_complex.pkl", "rb") as file:  # Open file in binary read mode
#        region = pickle.load(file)


with open("test_data/complex_polygon.pkl", "rb") as file:
    region = pickle.load(file)

print(region)

if dfs_sort:
    # Order the list of sub polygons
    adjacency_matrix, adjacency_graph = multi_poly_planning.create_adjacency(region)
    start_node = next(iter(adjacency_graph.nodes()))
    functions.plot_graph(adjacency_graph)
    polygons = multi_poly_planning.sort_sub_polygons_using_dfs(adjacency_graph, region, start_node)

else:  # Unsorted polygons
    polygons = region

start_time = time.time()
total_path, transit_flags = multi_poly_planning.multi_path_planning(polygons, path_width, extern_start_end)
end_time = time.time()
elapsed_time = end_time - start_time

data = path_data_v1.compute_path_data(region, total_path, transit_flags, path_width, None, elapsed_time)
print(data)
plot_cpp_v1.plot_multi_polys_path(path_width, region, total_path, None, False, transit_flags)
