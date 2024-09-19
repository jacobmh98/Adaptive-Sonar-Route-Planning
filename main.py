import numpy as np
import json
import functions
import antipodal_points
import polygon_coverage_path
import CPP_2D_convex_functions
import matplotlib.pyplot as plt
from Polygon import Polygon, Vertex
from json.encoder import INFINITY

# Reading the test data
f1 = open('test_data/complex_polygon.json')
f2 = open('test_data/simple_rectangle.json')
f3 = open('test_data/simple_polygon1.json')
f4 = open('test_data/simple_pentagon.json')
f5 = open('test_data/skewed_simple_rectangle.json')
f6 = open('test_data/complex_convex_polygon.json')

data = json.load(f4)
P = np.array(data["area"]["coordinates"])

# Find antipodal vertex testing (Not using new Polygon class)
antipodal_vertices = antipodal_points.antipodal_vertice_pairs(P)
antipodal_vertices = antipodal_points.remove_parallel_antipodal_vertices(P, antipodal_vertices)  # Removing parallel vertices
diametral_point_0 = antipodal_points.get_diametral_antipodal_point(P, antipodal_vertices, 0)

# Using new Polygon class for path finding in convex polygon
vertices_data = data['area']['coordinates']
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

boundaries = np.array([min_x, max_x, min_y, max_y])
poly = Polygon(vertices)
#poly.plot()

dx = 1
path = polygon_coverage_path.get_path(poly, dx, 0, 1, diametral_point_0, boundaries)

#path = CPP_2D_convex_functions.get_path(P, 0.20,0, 1, diametral_point_0, boundaries)



quit()
# Defining the vertices in counter-clock wise order
vertices = data['area']['coordinates']
vertices = np.array(vertices).T

# Number of vertices
n = vertices.shape[1]

# Creating a plot
fig = plt.figure()

# Plotting the points
x_coords = vertices[0, :]
y_coords = vertices[1, :]

# Plotting path
#path = np.array(path).T
#path_x = path[0, :]
#path_y = path[1, :]

plt.plot(x_coords, y_coords, 'b-', marker='o')  # Connect back to the first point to close the polygon
plt.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'b-')

#plt.plot(path_x, path_y, 'r-', marker='o')
plt.grid()

for i in range(n):
    x = vertices[0][i]
    y = vertices[1][i]
    plt.text(x, y, f'{i}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex

    #for k in range(antipodal_vertices.shape[0]):
    #    xk = [P[antipodal_vertices[k, 0], 0], P[antipodal_vertices[k, 1], 0]]
    ##    yk = [P[antipodal_vertices[k, 0], 1], P[antipodal_vertices[k, 1], 1]]
      #  plt.plot(xk, yk, linestyle='--', marker='o', color=[0.7, 0.7, 0.7])

plt.show()

quit()