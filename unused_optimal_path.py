import cpp_connect_path
import cpp_path_data
import cpp_path_planning
from global_variables import *

def compute_optimal_path_width(polygons, start_path_width, start_tolerance, start_iterations):  # TODO: Problem is it usually takes largest path width, so need to check more parameters (polygon coverage)
    start_width = start_path_width - start_tolerance
    end_width = start_path_width #+ start_tolerance  TODO: Usually always picks largest, so best is to check if a smaller one is better for now
    step_size = (end_width - start_width) / start_iterations
    min_distance = float('inf')
    optimal_path = None
    optimal_path_width = None

    # Finding optimal path width
    for i in range(start_iterations + 1):
        current_path_width = start_width + i * step_size
        current_intersections = cpp_path_planning.multi_intersection_planning(polygons, current_path_width)
        current_path = cpp_connect_path.connect_path(polygons, current_intersections)
        current_distance = cpp_path_data.compute_total_distance(current_path)

        if current_distance < min_distance:
            min_distance = current_distance
            optimal_path = current_path
            optimal_path_width = current_path_width

    if check_reverse:
        optimal_path, min_distance = compute_reverse(polygons, optimal_path_width)

    print(f'Optimal path:')
    print(f'Found optimal path width: {path_width}')
    print()

    return optimal_path, min_distance, optimal_path_width

def compute_reverse(polygons, current_path_width):
        # Computing the total path
        current_intersections = cpp_path_planning.multi_intersection_planning(polygons, current_path_width)
        current_path = cpp_connect_path.connect_path(polygons, current_intersections)
        current_distance = cpp_path_data.compute_total_distance(current_path)

        # Reverse
        polygons_r = polygons[::-1]
        current_intersections_r = cpp_path_planning.multi_intersection_planning(polygons_r, current_path_width)
        current_path_r = cpp_connect_path.connect_path(polygons_r, current_intersections_r)
        current_distance_r = cpp_path_data.compute_total_distance(current_path_r)

        if current_distance < current_distance_r:
            print('Standard Polygon Order')
            return current_path, current_distance
        else:
            print('Reversed Polygon Order')
            return current_path_r, current_distance_r