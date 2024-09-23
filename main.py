import numpy as np
import matplotlib.pyplot as plt
import math
import json
import antipodal_points
import polygon_coverage_path
import Rotating_Calipers_antipodal_pairs
from Polygon import Polygon, Vertex
from json.encoder import INFINITY

def get_b_mate(p, b):
    """ Find b's neighbour b_mate (counterclockwise neighbour)
    :param b: int, index of vertex b
    :param p: Polygon
    :return neighbour: int, index of b's neighbour vertex b_mate
    """
    n = len(p.vertices)

    if b == (n - 1):
        neighbour = 0
    else:
        neighbour = b + 1

    return neighbour

def get_previous_vertex(p, b):
    """ Find b's previous neighbour
    :param p: Polygon
    :param b: int, index of vertex b
    :return neighbour: int, index of b's previous neighbour
    """
    n = len(p.vertices)

    if b == 0:
        neighbour = n - 1
    else:
        neighbour = b - 1

    return neighbour

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

def get_shortest_dist_start_vertex(p, pw, antipodal_vertices, boundaries):
    """ Computing shortest distance path using each vertex as start vertex

    :param p: Polygon
    :param pw: float, path width
    :param antipodal_vertices: list of antipodal vertices in poly
    :param boundaries: list of boundaries in poly
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

# Reading the test data
f1 = open('test_data/simple_triangle.json')
f1_1 = open('test_data/simple_triangle_opposite.json')
f2 = open('test_data/simple_rectangle.json')
f3 = open('test_data/skewed_simple_rectangle.json')
f4 = open('test_data/simple_pentagon.json')
f5 = open('test_data/simple_polygon.json')
f6 = open('test_data/complex_convex_polygon.json')
f7 = open('test_data/large_complex_convex_polygon.json')  # Not convex
f8 = open('test_data/complex_polygon.json')
data = json.load(f8)
coordinates = data['area']['coordinates']

poly, boundaries = create_polygon(coordinates)
#poly.plot()

# Start parameters
dx = 0.01  # Path width
b_index = 0 # Index of start vertex
b_mate_index = get_b_mate(poly, b_index)  # Automatically computes counterclockwise neighbour vertex to b

antipodal_vertices = Rotating_Calipers_antipodal_pairs.compute_antipodal_pairs(poly)

# Removes neighbour pairs and double pairs, i.e. for [0,1] and [1,0] only 1 of them is necessary
filtered_antipodal_vertices = Rotating_Calipers_antipodal_pairs.filter_and_remove_redundant_pairs(poly, antipodal_vertices)
Rotating_Calipers_antipodal_pairs.plot_antipodal_points(poly, filtered_antipodal_vertices)
diametric_antipode_index = Rotating_Calipers_antipodal_pairs.get_diametric_antipodal_pair_index(poly, filtered_antipodal_vertices, b_index)

# If b_mate and a (the diametric point) are the same point, return direction from b, b_mate vector to remaining point (triangle edge case)
if np.allclose(b_mate_index, diametric_antipode_index):
    diametric_antipode_index = poly.vertices[get_previous_vertex(poly, b_index)].v

curr_path = polygon_coverage_path.get_path(poly, dx, b_index, b_mate_index, diametric_antipode_index, boundaries)
polygon_coverage_path.plot_path(poly.vertices[b_index].v, poly.vertices[b_mate_index].v, poly.vertices[diametric_antipode_index].v, dx, boundaries, poly, curr_path)


quit()
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
    # Plotting shortest path
    polygon_coverage_path.plot_path(poly.vertices[start_vertex_index].v,
                                    poly.vertices[get_b_mate(poly, start_vertex_index)].v,
                                    poly.vertices[antipodal_points.get_diametral_antipodal_point(poly, antipodal_vertices, start_vertex_index)].v,
                                    dx, boundaries, poly, shortest_path)

