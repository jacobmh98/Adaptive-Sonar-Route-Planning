from json.encoder import INFINITY
from os.path import split

import numpy as np
import json

from Polygon import Vertex, Polygon
from functions import *
from functions import compute_polygon_width

# Reading the test data
f = open('test_data/complex_polygon.json')
#f = open('test_data/intersection_in_vertex.json')
data = json.load(f)
vertices_data = data['area']['coordinates']

# Defining the initial polygon and the boúnding box
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

polygon = Polygon(vertices)


#polygon.plot()

# Compute the concave vertices
polygon.concave_vertices = compute_concave_vertices(polygon)
print(f'concave vertices = {polygon.concave_vertices}')

# Initialize the width sum matrix
ncc = len(polygon.concave_vertices)
n = len(polygon.vertices)
D = np.empty((ncc, n))
D_polygons = []

# Go through each concave vertex
for i, cv in enumerate(polygon.concave_vertices):
   # print(f"checking for {cv.index=} with coord = ({cv.x}, {cv.y})")
    split_polygons = []

    # Check lines which passes the concave vertex i and parallels edge e
    for j, e in enumerate(polygon.edges):
        intersection_points = []
        intersection_edges = []
        intersection_directions = []

       # print(f'\tchecking edge {e}')

        # Define a vector from the vertices in edge e
        vec = e.v_to.get_array() - e.v_from.get_array()
        #vec = -vec

        # Go through each edge in the polygon
        for e2 in polygon.edges:
            if e == e2:
                continue

            # Compute intersection with edge e2 (if any)
            ip, t = compute_intersection(vec, cv, e2)
            if ip is not None:
                #print(f'\t\t{e} intersects {e2} at ({ip[0,0]}, {ip[1,0]})), {t=}')
                intersection_points.append(ip)
                intersection_edges.append(e2)
                intersection_directions.append(t)

        min_index = np.argmin(np.abs(intersection_directions))

        P1, P2 = split_polygon_single(intersection_edges[min_index], intersection_points[min_index], cv)

        # Compute the width sum of P1 and P2
        D[i, j] = compute_polygon_width(P1) + compute_polygon_width(P2)

        split_polygons.append((P1, P2))
        #plot_results2(P1, P2)

    plot_results(split_polygons, cv.index, D[i, :])
    D_polygons.append(split_polygons)

v, (i,j) = find_min_value_matrix(D)


P1, P2 =  D_polygons[i][j]

print(f'{v=}')
print(f'(i, j)=({i}, {j})')

P1.plot('b')
P2.plot('r')
