from json.encoder import INFINITY

import json
from functions import *
import numpy as np

# Reading the test data
f = open('test_data/complex_polygon.json')
#f = open('test_data/buggy_file2.json')
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

# Compute the split that gives the initial sub-polygons
sub_polygons = split_polygon(P)
plot_polygons(sub_polygons)
quit()
# Creating adjacent matrix for the sub-polygons
A = np.empty((len(sub_polygons), len(sub_polygons)))

for i, p_i in  enumerate(sub_polygons):
    for j, p_j in enumerate(sub_polygons):
        # Ignore if p_i equals p_j
        if i == j:
            continue

        # Go through each edge in p_i and compare with each edge in p_j



plot_polygons(sub_polygons)


#print(results)