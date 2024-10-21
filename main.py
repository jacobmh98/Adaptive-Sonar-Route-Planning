import copy
import time
import json
import connecting_path
import coverage_plots
import multi_poly_planning
import optimal_path
import path_comparison_functions
import traveling_salesman_variation
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
        f = open("test_data/complex_polygon.json", "rb")
        data = json.load(f)
        vert_data = data['area']['coordinates']
        vertices = []
        for i in range(len(vert_data)):
            vertices.append(Vertex(i, vert_data[i][0], vert_data[i][1]))
        polygon_test = Polygon(vertices)

# Starting timer for all cpp functions
total_start_time = time.time()

# Compute intersections
intersections = multi_poly_planning.multi_intersection_planning(optimized_sub_polygons, path_width)

# Pairing each polygon with its corresponding intersections using a dictionary, to keep track of which intersection list is for which sub polygon
polygon_intersections = {
    poly.i: {"polygon": poly, "intersections": inters}
    for poly, inters in zip(optimized_sub_polygons, intersections)
}

# Create sorted lists of polygons and intersections, such that poly[0] and intersections[0] are for the same poly
sorted_polygons = [polygon_intersections[poly.i]["polygon"] for poly in optimized_sub_polygons]
sorted_intersections = [polygon_intersections[poly.i]["intersections"] for poly in optimized_sub_polygons]

# Creating the expanded distance matrix to consider start/end combinations
expanded_distance_matrix = create_expanded_distance_matrix(sorted_polygons, sorted_intersections)

# Solve the TSP with start/end combinations
tsp_route_with_combinations = tsp_solver_with_combinations(expanded_distance_matrix, len(sorted_polygons))

# Use the chosen combination indices to retrieve the optimal start/end points for each polygon
optimal_path = []
for polygon_index, combination_index in tsp_route_with_combinations:
    polygon = sorted_polygons[polygon_index]
    intersections = sorted_intersections[polygon_index]

    # Get the optimal start and end points for the polygon using the selected combination
    start, end = get_start_end_combinations(intersections)[combination_index]

    optimal_path.append((start, end))

tsp_polygon_indices = [polygon_index for polygon_index, _ in tsp_route_with_combinations]
#visualize_tsp_solution(sorted_polygons, tsp_polygon_indices)

# Use the optimal path points to construct the final path using the original `connect_path` function
path = connecting_path.connect_path(sorted_polygons, sorted_intersections)

# Ending timer and computing total execution time
total_end_time = time.time()
total_execution_time = total_end_time - total_start_time

if compute_path_data:
    print(f'Execution time: {total_execution_time}')

    covered_area, coverage_percentage = path_comparison_functions.compute_covered_area(polygon_test, path)
    outlier_area = path_comparison_functions.compute_outlier_area(polygon_test, path)
    overlap_area = path_comparison_functions.compute_overlap_area(polygon_test, path)

    print(f'Coverage percentage: {round(coverage_percentage, 2)}%')
    print(f'Covered area: {covered_area.area}')
    print(f'Outlier area: {outlier_area.area}')
    print(f'Overlap area: {overlap_area.area}')
    #coverage_plots.visualize_coverage_wasted_and_overlap(polygon_test, path, covered_area, outlier_area, overlap_area)
    print()

    # Computing turns in the path
    distance = path_comparison_functions.compute_total_distance(path)
    total_turns, hard_turns, medium_turns, soft_turns = path_comparison_functions.compute_turns(path)

    print(f'Distance: {distance}')
    print(f'Total turns: {total_turns}')
    print(f'Hard turns (<45): {hard_turns}')
    print(f'Medium turns (45-90): {medium_turns}')
    print(f'Soft turns (>90): {soft_turns}')

    if store_data:
        output_file = "coverage_results.txt"

        with open(output_file, 'w') as file:
            file.write(f"Execution time: {total_execution_time}\n")
            file.write(f"Coverage percentage: {round(coverage_percentage, 2)}%\n")
            file.write(f"Covered area: {covered_area.area}\n")
            file.write(f"Wasted area: {outlier_area.area}\n")
            file.write(f"Overlap area: {overlap_area.area}\n\n")
            file.write(f"Distance: {distance}\n")
            file.write(f"Total turns: {total_turns}\n")
            file.write(f"Hard turns (<45): {hard_turns}\n")
            file.write(f"Medium turns (45-90): {medium_turns}\n")
            file.write(f"Soft turns (>90): {soft_turns}\n")


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

