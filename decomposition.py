
import json
from functions import *
import numpy as np

# Reading the test data
f = open('test_data/complex_polygon.json')
#f = open('test_data/buggy_file3.json')
data = json.load(f)
vertices_data = data['area']['coordinates']

# Defining the initial polygon and the boúnding box
vertices = []
min_x = np.inf
max_x = -np.inf
min_y = np.inf
max_y = -np.inf

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

P = Polygon(vertices)

P.plot('b')
# Compute the split that gives the sub-polygons
sub_polygons = split_polygon(P)

# Creating adjacent matrix for the sub-polygons to tell which sub-polygons are connected
"""
P = Polygon([Vertex(0, 3, 10), Vertex(1, 3, 6), Vertex(2, 5, 4), Vertex(3, 9, 3),
             Vertex(4, 7, 6), Vertex(5, 9, 8), Vertex(6, 6, 8)])

P1 = Polygon([Vertex(0, 3, 10), Vertex(1, 3, 6), Vertex(2, 6, 8)])
P2 = Polygon([Vertex(0, 3, 6), Vertex(1, 5, 4), Vertex(2, 9, 8), Vertex(3, 6, 8)])
P3 = Polygon([Vertex(0, 7, 6), Vertex(1, 5, 4), Vertex(2, 9, 3)])

sub_polygons = [P1, P2, P3]
"""

# Define the adjacent matrix of the undirected graph
m = len(sub_polygons)
A = np.zeros((m, m))
G = nx.Graph()

# Go through each edge in p_i and compare with each edge in p_j
for i, p_i in  enumerate(sub_polygons):
    for j, p_j in enumerate(sub_polygons):
        # Ignore if the two polygons are equal
        if i == j:
            continue

        # Test if the two polygons p_i and p_j are adjacent (either complete or partial)
        if polygons_are_adjacent(p_i, p_j, i, j):
            # Update the adjacent matrix
            A[i, j] = 1
            A[j, i] = 1
            G.add_edge(f'P{i}', f'P{j}')

            print(f'{i} and {j} are adjacent')

plot_polygons(P, sub_polygons, G)