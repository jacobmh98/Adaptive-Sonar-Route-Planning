import numpy as np
import matplotlib.pyplot as plt
import math
import json
import antipodal_points
import polygon_coverage_path
import Rotating_Calipers_antipodal_pairs
import multi_poly_planning
from Polygon import Polygon, Vertex
from json.encoder import INFINITY

def path_distance(cp):
    """ Calculate the total distance travelled along a path.

    :param cp: NumPy array of 2D points representing the current path
    :return distance: The total Euclidean distance between consecutive points in the path.
    """
    distance = 0.0
    for i in range(1, len(cp)):
        # Calculate the distance between the consecutive points
        distance += math.sqrt((cp[i][0] - cp[i - 1][0]) ** 2 + (cp[i][1] - cp[i - 1][1]) ** 2)

    return distance

def create_polygon(vertices_data):
    vertices = []
    min_x = INFINITY
    max_x = -INFINITY
    min_y = INFINITY
    max_y = -INFINITY
    for i, v in enumerate(vertices_data):
        if v[0] < min_x:
            min_x = v[0]
        if v[0] > max_x:
            max_x = v[0]
        if v[1] < min_y:
            min_y= v[1]
        if v[1] > max_y:
            max_y = v[1]
        vertices.append(Vertex(i, v[0], v[1]))
    return Polygon(vertices), np.array([min_x, max_x, min_y, max_y])

def rotating_calipers_path_planner(p, d_pq, ps, pe, pw, bounds):
    """ Algorithm 2: Rotating Calipers Path Planner.
    Computes the optimal back-and-forth path that covers a convex polygon efficiently by testing all antipodal pairs.

    :param p: Polygon
    :param d_pq: List of tuples representing antipodal pairs (i, j).
    :param ps: Starting point
    :param pe: Ending point
    :param pw: Path width
    :param bounds: Boundaries of the polygon.
    :return optimal_path: The best back-and-forth path (computed using best_path).
    """

    # Initialize variables to store the best path and the minimal cost
    min_cost = float('inf')
    optimal_path = None
    new_best_pair = ()

    # Iterate over all antipodal pairs (i, j)
    for (i, j) in d_pq:
        #print(f'i,j: {i},{j}')
        # Compute the best path for the current antipodal pair
        current_path, current_cost = polygon_coverage_path.best_path(p, i, j, ps, pe, pw, bounds)

        # Update the optimal path if the current path has a lower cost
        if current_cost < min_cost:
            min_cost = current_cost
            optimal_path = current_path
            new_best_pair = (i,j)

    return optimal_path

# Manually creating polygons
t1_0 = open('test_data/connected_poly_1_0.json')
t1_1 = open('test_data/connected_poly_1_1.json')
data1_0 = json.load(t1_0)
data1_1 = json.load(t1_1)
coord1_0 = data1_0['area']['coordinates']
coord1_1 = data1_1['area']['coordinates']
poly1_0, boundaries1_0 = create_polygon(coord1_0)
poly1_1, boundaries1_1 = create_polygon(coord1_1)
polygons = []
all_boundaries = []
polygons.append(poly1_0)
polygons.append(poly1_1)
all_boundaries.append(boundaries1_0)
all_boundaries.append(boundaries1_1)


# Start parameters
p_start = [0.5, -0.5]
p_end = [2.75, 1.75]
extern_start_end = False
dx = 0.1 # Path width (Must be >0)
#b_index = 0 # Index of start vertex
#b_mate_index = polygon_coverage_path.get_b_mate(polygons[0], b_index)  # Automatically computes counterclockwise neighbour vertex to b

total_path = multi_poly_planning.multi_path_planning(polygons, extern_start_end, p_start, p_end, dx, all_boundaries)
print(np.array(total_path))
multi_poly_planning.multi_poly_plot(np.array(total_path), polygons, p_start, p_end, dx)

quit()



# Reading the test data
f1 = open('test_data/simple_triangle.json')
f1_1 = open('test_data/simple_triangle_opposite.json')
f2 = open('test_data/simple_rectangle.json')
f3 = open('test_data/simple_skewed_rectangle.json')
f3_1 = open('test_data/simple_trapezoid.json')
f4 = open('test_data/simple_pentagon.json')
f5 = open('test_data/simple_polygon.json')
f6 = open('test_data/complex_convex_polygon.json')
f7 = open('test_data/large_complex_convex_polygon.json')  # Not convex
f8 = open('test_data/complex_polygon.json')
data = json.load(f3_1)
coordinates = data['area']['coordinates']
poly, boundaries = create_polygon(coordinates)

# Start parameters
p_start = [-0.5, -0.5]
p_end = [2.75, 1.75]
dx = 0.1 # Path width (Must be >0)
b_index = 0 # Index of start vertex
b_mate_index = polygon_coverage_path.get_b_mate(poly, b_index)  # Automatically computes counterclockwise neighbour vertex to b

# Computing polygons antipodal points
antipodal_vertices = Rotating_Calipers_antipodal_pairs.compute_antipodal_pairs(poly)
# Removes neighbour pairs and double pairs, i.e. for [0,1] and [1,0] only 1 of them is necessary
filtered_antipodal_vertices = Rotating_Calipers_antipodal_pairs.filter_and_remove_redundant_pairs(poly, antipodal_vertices)
# Computing the diametric antipodal pairs. Optimizing number of paths computed
diametric_antipodal_pairs = Rotating_Calipers_antipodal_pairs.filter_diametric_antipodal_pairs(poly, filtered_antipodal_vertices)
# Getting index of a, the diametric antipodal point of b
diametric_antipode_index = Rotating_Calipers_antipodal_pairs.get_diametric_antipodal_point_index(diametric_antipodal_pairs, b_index)

# (Triangle edge case) If b_mate and a are the same point, change a to last remaining vertex in the triangle polygon
if np.allclose(b_mate_index, diametric_antipode_index):
    diametric_antipode_index = poly.vertices[polygon_coverage_path.get_previous_vertex(poly, b_index)].v

# Computing 1 path for b, b_mate and a
#curr_path = polygon_coverage_path.get_path(poly, dx, b_index, b_mate_index, diametric_antipode_index, boundaries)
# Plotting the computed path
#polygon_coverage_path.plot_path(poly.vertices[b_index].v, poly.vertices[b_mate_index].v, poly.vertices[diametric_antipode_index].v, dx, boundaries, poly, curr_path)

# Using rotating calipers method to find the shortest path in the polygon, using b as starting point
shortest_path = rotating_calipers_path_planner(poly, p_start, p_end, dx, boundaries)
polygon_coverage_path.plot_path(poly.vertices[b_index].v, poly.vertices[b_mate_index].v, poly.vertices[diametric_antipode_index].v, dx, boundaries, poly, shortest_path)

quit()

# Computing path for multiple convex polygons
total_path = []
polygons = []


# Manually creating polygons
t1_0 = open('test_data/connected_poly_1_0.json')
t1_1 = open('test_data/connected_poly_1_1.json')
data1_0 = json.load(t1_0)
data1_1 = json.load(t1_1)
coord1_0 = data['area']['coordinates']
coord1_1 = data['area']['coordinates']
poly1_0, boundaries1_0 = create_polygon(coord1_0)
poly1_1, boundaries1_1 = create_polygon(coord1_1)
polygons.append(poly1_0)
polygons.append(poly1_1)

# Start parameters
p_start = [-0.5, -0.5]
p_end = [2.75, 1.75]
dx = 0.1 # Path width (Must be >0)
b_index = 0 # Index of start vertex
b_mate_index = polygon_coverage_path.get_b_mate(poly1_0, b_index)  # Automatically computes counterclockwise neighbour vertex to b



for polyg in polygons:
    #print("1")
    print(polyg.vertices)
    new_path = rotating_calipers_path_planner(polyg, diametric_antipodal_pairs, p_start, poly1_1.vertices[0].index, dx, boundaries)
    #print(np.array(new_path))

    for p in new_path:
        total_path.append(p)

    # Start next path at last path point

total_path = np.array(total_path)


polygon_coverage_path.plot_path(poly1_0.vertices[b_index].v, poly.vertices[b_mate_index].v, poly.vertices[diametric_antipode_index].v, dx, boundaries, poly1_0, total_path)





quit()
# Below uses old antipodal computations
# Computing antipodal and diametral point for b
antipodal_vertices = antipodal_points.antipodal_vertice_pairs(poly)
antipodal_vertices = antipodal_points.remove_parallel_antipodal_vertices(poly, antipodal_vertices)  # Removing parallel vertices
# TODO: Optimize above functions to find diametral point while computing antipodal pairs
diametral_point_index = antipodal_points.get_diametral_antipodal_point(poly, antipodal_vertices, b_index)  # index

# If b_mate and a (the diametral point) are the same point, return direction from b, b_mate vector to remaining point (triangle edge case)
if np.allclose(b_mate_index, diametral_point_index):
    diametral_point_index = poly.vertices[get_previous_vertex(poly, b_index)].v

#antipodal_points.plot_antipodal_points(poly, antipodal_vertices)

# Computing the coverage path for a convex Polygon
curr_path = polygon_coverage_path.get_path(poly, dx, b_index, b_mate_index, diametral_point_index, boundaries)
#polygon_coverage_path.plot_path(poly.vertices[b].v, poly.vertices[b_mate].v, poly.vertices[diametral_point].v, dx, boundaries, poly, curr_path)

find_shortest_path = True
if find_shortest_path:
    start_vertex_index, shortest_path, shortest_path_distance = get_shortest_dist_start_vertex(poly, dx, antipodal_vertices, boundaries)
    print(f'Start vertex index: {start_vertex_index}')
    print(f'shortest path distance: {shortest_path_distance}')
    #print(shortest_path)
    # Plotting the shortest path
    polygon_coverage_path.plot_path(poly.vertices[start_vertex_index].v,
                                    poly.vertices[get_b_mate(poly, start_vertex_index)].v,
                                    poly.vertices[antipodal_points.get_diametral_antipodal_point(poly, antipodal_vertices, start_vertex_index)].v,
                                    dx, boundaries, poly, shortest_path)



# Old functions
# Update to not use all vertices but instead go through each antipodal pair and compute the path
def get_shortest_dist_start_vertex(p, pw, pq, bounds):
    """ Computing the shortest distance path using each vertex as start vertex

    :param p: Polygon
    :param pw: float, path width
    :param pq: list of antipodal vertices in poly
    :param bounds: list of boundaries in poly
    :return start_vertex_index: index of vertex to start with for shortest path
    :return shortest_path: list containing points in the shortest path
    :return shortest_path_distance: float, the total distance in the shortest path
    """

    n = len(p.vertices)
    p_s = 0
    shortest_path = []
    shortest_path_distance = INFINITY

    for i in range(0, n):
        b_index = i
        b_mate_index = get_b_mate(p, b_index)  # Automatically computes counterclockwise neighbour vertex to b
        diametral_point = antipodal_points.get_diametral_antipodal_point(p, antipodal_vertices, b_index)
        curr_path = polygon_coverage_path.get_path(p, pw, b_index, b_mate_index, diametral_point, boundaries)
        curr_path_dist = path_distance(curr_path)

        if shortest_path_distance > curr_path_dist:
            p_s = i
            shortest_path_distance = curr_path_dist
            shortest_path = curr_path

    return start_vertex_index, shortest_path, shortest_path_distance