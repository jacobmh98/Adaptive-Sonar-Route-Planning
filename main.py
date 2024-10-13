import pickle
import time
import json
import multi_poly_planning
import connecting_path
import coverage_plots
import path_comparison_functions
from global_variables import *
from Polygon import Polygon, Vertex

antwerp = True
if antwerp:
    with open('test_data/antwerp_comparison.pkl', 'rb') as file:
        optimized_polygons = pickle.load(file)

else:
    with open('test_data/simple_complex_comparison.pkl', 'rb') as file:
        optimized_polygons = pickle.load(file)


if find_optimal_path_width:  # TODO: Problem is it usually takes largest path width, so need to check more parameters (polygon coverage)
    start_width = path_width - tolerance
    end_width = path_width + tolerance
    step_size = (end_width - start_width) / iterations
    current_path_width = path_width
    min_distance = float('inf')
    optimal_path = None
    optimal_path_width = None

    # Baseline path
    base_intersections = multi_poly_planning.multi_intersection_planning(optimized_polygons, path_width, extern_start_end)
    base_path = connecting_path.connect_path(optimized_polygons, base_intersections)
    base_distance = path_comparison_functions.compute_total_distance(base_path)

    # Finding optimal path width
    for i in range(iterations + 1):
        current_path_width = start_width + i * step_size
        current_intersections = multi_poly_planning.multi_intersection_planning(optimized_polygons, current_path_width, extern_start_end)
        current_path = connecting_path.connect_path(optimized_polygons, current_intersections)
        distance = path_comparison_functions.compute_total_distance(current_path)

        if distance < min_distance:
            min_distance = distance
            optimal_path = current_path
            optimal_path_width = current_path_width

    print(f'Optimal path width: {optimal_path_width}')
    print(f'Optimal distance: {min_distance}')
    print(f'Base path width: {path_width}')
    print(f'Base distance: {base_distance}')

    total_path = optimal_path
    coverage_plots.multi_poly_plot(None, current_path_width, optimized_polygons, total_path)

else:
    if check_reverse:
        # Computing the total path
        start_time = time.time()
        total_intersections = multi_poly_planning.multi_intersection_planning(optimized_polygons, path_width, extern_start_end)
        total_path = connecting_path.connect_path(optimized_polygons, total_intersections)
        distance = path_comparison_functions.compute_total_distance(total_path)

        # Reverse
        optimized_polygons_r = optimized_polygons[::-1]
        total_intersections = multi_poly_planning.multi_intersection_planning(optimized_polygons_r, path_width, extern_start_end)
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
        total_intersections = multi_poly_planning.multi_intersection_planning(optimized_polygons, path_width, extern_start_end)
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

    coverage_plots.multi_poly_plot(None, path_width, optimized_polygons, total_path)



quit()
"""f = open('test_data/antwerpen_full.json')

    with open('test_data/antwerpen_epsilon_e6_optimized_e2.pkl', 'rb') as file:
        sub_polygons = pickle.load(file)

    data = json.load(f)
    vertices_data = data['area']['coordinates']

    # Defining the initial polygon
    vertices = []

    for i in range(len(vertices_data)):
        vertices.append(Vertex(i, vertices_data[i][0], vertices_data[i][1]))
    poly = Polygon(vertices)
    optimized_polygons = sub_polygons

    # Remove collinear vertices in each sub-polygon
    for i, p in enumerate(optimized_polygons):
        p = remove_collinear_vertices(p)
        optimized_polygons[i] = p

    adjacency_matrix, adjacency_graph = multi_poly_planning.create_adjacency(optimized_polygons)
    start_node = next(iter(adjacency_graph.nodes()))
    optimized_polygons = multi_poly_planning.sort_sub_polygons_using_dfs(adjacency_graph, optimized_polygons, start_node)
"""

"""For pretty plots
f = open('test_data/simple_pentagon.json')
data = json.load(f)
vertices_data = data['area']['coordinates']

# Defining the initial polygon
vertices = []

for i in range(len(vertices_data)):
    vertices.append(Vertex(i, vertices_data[i][0], vertices_data[i][1]))
poly = Polygon(vertices)
optimized_polygons = [poly]

total_intersections = multi_poly_planning.multi_intersection_planning(optimized_polygons, dx, extern_start_end)
"""