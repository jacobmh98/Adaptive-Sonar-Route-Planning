import numpy as np
import json
import antipodal_points
import polygon_coverage_path
import matplotlib.pyplot as plt
from Polygon import Polygon, Vertex
from json.encoder import INFINITY

# Reading the test data
f1 = open('test_data/simple_triangle.json')
f2 = open('test_data/simple_rectangle.json')
f3 = open('test_data/skewed_simple_rectangle.json')
f4 = open('test_data/simple_pentagon.json')
f5 = open('test_data/simple_polygon.json')
f6 = open('test_data/complex_convex_polygon.json')
f7 = open('test_data/complex_polygon.json')
data = json.load(f4)
coordinates = data['area']['coordinates']

def get_b_mate(b, poly):
    """ Find b's neighbour b_mate (counterclockwise neighbour)
    :param b: int, index of vertex b
    :param poly: Polygon
    :return neighbour: int, index of b's neighbour vertex b_mate
    """
    n = len(poly.vertices)

    if b == (n - 1):
        neighbour = 0
    else:
        neighbour = b + 1

    return neighbour

# Using new Polygon class for path finding in convex polygon
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

poly, boundaries = create_polygon(coordinates)
#poly.plot()

# Start parameters
dx = 0.5  # Path width
b = 2
b_mate = get_b_mate(b, poly)

# Computing antipodal and diametral point for b
antipodal_vertices = antipodal_points.antipodal_vertice_pairs(poly)
antipodal_vertices = antipodal_points.remove_parallel_antipodal_vertices(poly, antipodal_vertices)  # Removing parallel vertices
diametral_point_0 = antipodal_points.get_diametral_antipodal_point(poly, antipodal_vertices, b)
#antipodal_points.plot_antipodal_points(poly, antipodal_vertices)

# Computing the coverage path for a convex Polygon
path = polygon_coverage_path.get_path(poly, dx, b, b_mate, diametral_point_0, boundaries)