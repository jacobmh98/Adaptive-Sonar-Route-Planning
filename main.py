import pickle
import time
import multi_poly_planning
import connecting_path
import coverage_plots
import path_comparison_functions
from global_variables import *

antwerp = True
if antwerp:
    with open('test_data/antwerp_comparison.pkl', 'rb') as file:
        optimized_polygons = pickle.load(file)

else:
    with open('test_data/simple_complex_comparison.pkl', 'rb') as file:
        optimized_polygons = pickle.load(file)


if use_optimal_dx:  # TODO: Problem is it usually takes largest path width, so need to check more parameters (polygon coverage)
    start_width = dx - tolerance
    end_width = dx + tolerance
    step_size = (end_width - start_width) / iterations
    min_distance = float('inf')
    optimal_path = None
    optimal_path_width = None

    # Baseline path
    base_intersections = multi_poly_planning.multi_intersection_planning(optimized_polygons, dx, extern_start_end)
    base_path = connecting_path.connect_path(optimized_polygons, base_intersections)
    base_distance = path_comparison_functions.compute_total_distance(base_path)

    # Finding optimal
    for i in range(iterations + 1):
        current_width = start_width + i * step_size
        current_intersections = multi_poly_planning.multi_intersection_planning(optimized_polygons, current_width, extern_start_end)
        current_path = connecting_path.connect_path(optimized_polygons, current_intersections)
        distance = path_comparison_functions.compute_total_distance(current_path)

        if distance < min_distance:
            min_distance = distance
            optimal_path = current_path
            optimal_path_width = current_width

    print(f'Optimal path width: {optimal_path_width}')
    print(f'Optimal distance: {min_distance}')
    print(f'Base path width: {dx}')
    print(f'Base distance: {base_distance}')

    total_path = optimal_path

else:

    if check_reverse:
    # Computing the total path
        start_time = time.time()
        total_intersections = multi_poly_planning.multi_intersection_planning(optimized_polygons, dx, extern_start_end)
        total_path = connecting_path.connect_path(optimized_polygons, total_intersections)
        distance = path_comparison_functions.compute_total_distance(total_path)

        # Reverse
        optimized_polygons_r = optimized_polygons[::-1]
        total_intersections = multi_poly_planning.multi_intersection_planning(optimized_polygons_r, dx, extern_start_end)
        total_path_r = connecting_path.connect_path(optimized_polygons_r, total_intersections)
        distance_r = path_comparison_functions.compute_total_distance(total_path_r)

        end_time = time.time()
        elapsed_time = end_time - start_time

        if distance < distance_r:
            print('Standard polygon order')
            total_turns, hard_turns, medium_turns, soft_turns = path_comparison_functions.calculate_turns_and_classify(total_path)
        else:
            print('Reversed polygon order')
            distance = distance_r
            total_turns, hard_turns, medium_turns, soft_turns = path_comparison_functions.calculate_turns_and_classify(total_path_r)
            total_path = total_path_r
            optimized_polygons = optimized_polygons_r
    else:
        start_time = time.time()
        total_intersections = multi_poly_planning.multi_intersection_planning(optimized_polygons, dx, extern_start_end)
        total_path = connecting_path.connect_path(optimized_polygons, total_intersections)
        distance = path_comparison_functions.compute_total_distance(total_path)
        end_time = time.time()
        elapsed_time = end_time - start_time
        total_turns, hard_turns, medium_turns, soft_turns = path_comparison_functions.calculate_turns_and_classify(total_path)

    print(f'Distance: {distance}')
    print(f'Total turns: {total_turns}')
    print(f'Hard turns: {hard_turns}')
    print(f'Medium turns: {medium_turns}')
    print(f'Soft turns: {soft_turns}')
    print(f'Elapsed time: {elapsed_time}')


coverage_plots.multi_poly_plot(None, optimized_polygons, total_path)