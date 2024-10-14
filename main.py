import pickle
import time
import json
import multi_poly_planning
import connecting_path
import coverage_plots
import path_comparison_functions
import optimal_path
from functions import plot_results3
from global_variables import *
from Polygon import Polygon, Vertex

antwerp = True
if antwerp:
    with open('test_data/antwerp_comparison.pkl', 'rb') as file:
        optimized_polygons = pickle.load(file)
else:
    with open('test_data/simple_complex_comparison.pkl', 'rb') as file:
        optimized_polygons = pickle.load(file)

plot_results3(optimized_polygons)

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