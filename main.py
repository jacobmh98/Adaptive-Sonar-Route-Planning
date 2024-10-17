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
from intra_regional_tsp import *


antwerp = False
if antwerp:
    with open('test_data/antwerp_comparison.pkl', 'rb') as file:
        optimized_sub_polygons = pickle.load(file)
else:
    with open('test_data/simple_complex_comparison.pkl', 'rb') as file:
        optimized_sub_polygons = pickle.load(file)

# Step 1: Get intersections using multi_poly_planning
intersections = multi_poly_planning.multi_intersection_planning(optimized_sub_polygons, path_width)

# Step 2: Pair each polygon with its corresponding intersections using a dictionary
polygon_intersections = {
    poly.i: {"polygon": poly, "intersections": inters}
    for poly, inters in zip(optimized_sub_polygons, intersections)
}

# Step 3: Sort the polygons and intersections based on TSP route
# Create sorted lists of polygons and intersections
sorted_polygons = [polygon_intersections[poly.i]["polygon"] for poly in optimized_sub_polygons]
sorted_intersections = [polygon_intersections[poly.i]["intersections"] for poly in optimized_sub_polygons]

# Step 4: Create the expanded distance matrix to consider start/end combinations
expanded_distance_matrix = create_expanded_distance_matrix(sorted_polygons, sorted_intersections)

# Step 5: Solve the TSP with start/end combinations
tsp_route_with_combinations = tsp_solver_with_combinations(expanded_distance_matrix, len(sorted_polygons))

# Step 6: Use the chosen combination indices to retrieve the optimal start/end points for each polygon
optimal_path = []
for polygon_index, combination_index in tsp_route_with_combinations:
    polygon = sorted_polygons[polygon_index]
    intersections = sorted_intersections[polygon_index]

    # Get the optimal start and end points for the polygon using the selected combination
    start, end = get_start_end_combinations(intersections)[combination_index]

    optimal_path.append((start, end))

# Step 7: Use the optimal path points to construct the final path using the original `connect_path` function
path = connecting_path.connect_path(sorted_polygons, sorted_intersections)
visualize_tsp_solution(sorted_polygons, [x[0] for x in tsp_route_with_combinations], sorted_intersections)

# Step 8: Visualize the path using the coverage plot
coverage_plots.multi_poly_plot(None, path_width, sorted_polygons, path)

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


# Computing turns in the path
total_turns, hard_turns, medium_turns, soft_turns = path_comparison_functions.calculate_turns_and_classify(path)

print(f'Distance: {distance}')
print(f'Total turns: {total_turns}')
print(f'Hard turns (<45): {hard_turns}')
print(f'Medium turns (45-90): {medium_turns}')
print(f'Soft turns (>90): {soft_turns}')

coverage_plots.multi_poly_plot(None, path_width, optimized_polygons, path)

