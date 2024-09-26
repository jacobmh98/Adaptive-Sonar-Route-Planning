import json
import math
import multi_poly_planning
import traveling_salesman_variation
from functions import *

def path_distance(cp):
    """ Calculate the total distance travelled along a path.

    :param cp: NumPy array of 2D points representing the current path
    :return distance: The total Euclidean distance between consecutive points in the path.
    """
    dist = 0.0
    for i in range(1, len(cp)):
        # Calculate the distance between the consecutive points
        dist += math.sqrt((cp[i][0] - cp[i - 1][0]) ** 2 + (cp[i][1] - cp[i - 1][1]) ** 2)

    return dist

def create_polygon(vert_data):
    vertices = []
    for i, v in enumerate(vert_data):
        vertices.append(Vertex(i, v[0], v[1]))

    return Polygon(vertices)

# Reading the test data
f = open('test_data/complex_polygon.json')
data = json.load(f)
vertices_data = data['area']['coordinates']
P = create_polygon(vertices_data)

# Compute the split that gives the sub-polygons
sub_polygons = split_polygon(P)

#for i,poly in enumerate(sub_polygons):
#    poly.plot()

# Start parameters
dx = 0.2 # Path width (Must be >0)
extern_start_end = False
if extern_start_end:
    p_start = [0.0, 0.0]
    p_end = [7, 6]
else:
    p_start = None
    p_end = None

for i,poly in enumerate(sub_polygons):
    if i == 1:
        sub_polygons.remove(poly)

#sub_polygons = [sub_polygons[1]]

# For tsp
distance_matrix = traveling_salesman_variation.create_distance_matrix(sub_polygons)
tsp_route = traveling_salesman_variation.solve_tsp(distance_matrix)
#traveling_salesman_variation.visualize_tsp_solution(sub_polygons, tsp_route)

# Sort the polygons according to the TSP route
sorted_polygons = [sub_polygons[i] for i in tsp_route]


total_path = multi_poly_planning.multi_path_planning(sorted_polygons, dx, extern_start_end, p_start, p_end)
multi_poly_planning.multi_poly_plot(sorted_polygons, dx, extern_start_end, p_start, p_end, np.array(total_path))
