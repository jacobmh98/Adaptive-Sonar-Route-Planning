import copy
import time
import json
import connecting_path
import coverage_plots
import intra_regional_tsp
import multi_poly_planning
import optimal_path
import path_comparison_functions
import traveling_salesman_variation
import traceback
from global_variables import *
from functions import *
import pickle
from intra_regional_tsp import *



# Starting timer for all cpp functions
total_start_time = time.time()

# Compute intersections
intersections = multi_poly_planning.multi_intersection_planning(optimized_sub_polygons, path_width)

# Choosing sorting method
if tsp_sorting:
    tsp_route = intra_regional_tsp.start_tsp(optimized_sub_polygons, intersections)

# Use the optimal path points to construct the final path using the original `connect_path` function
path = connecting_path.connect_path(optimized_sub_polygons, intersections)

# Ending timer and computing total execution time
total_end_time = time.time()
total_execution_time = total_end_time - total_start_time

if get_path_data:
    path_comparison_functions.compute_path_data(polygon_test, path, total_execution_time)


#coverage_plots.multi_poly_plot(None, path_width, sorted_polygons, path)

quit()




# Computing optimal path width given start path width and a tolerance (+- to path width)
if find_optimal_path_width:
    print(f'Optimal path:')
    path, distance, path_width = optimal_path.compute_optimal_path_width(optimized_polygons, path_width, tolerance, iterations)
    print(f'Found optimal path width: {path_width}')

    if check_reverse:
        path, distance = optimal_path.compute_reverse(optimized_polygons, path_width)


# Check if computing path using reversed sorted polygon list provides a better result
elif check_reverse:
    path, distance = optimal_path.compute_reverse(optimized_polygons, path_width)

# Baseline path
else:
    print('Baseline path:')
    intersections = multi_poly_planning.multi_intersection_planning(optimized_polygons, path_width)
    path = connecting_path.connect_path(optimized_polygons, intersections)
    distance = path_comparison_functions.compute_total_distance(path)



coverage_plots.multi_poly_plot(None, path_width, optimized_polygons, path)

