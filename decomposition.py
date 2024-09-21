from json.encoder import INFINITY
from os.path import split

import numpy as np
import json

from Polygon import Vertex, Polygon
from functions import *
from functions import compute_polygon_width

# Reading the test data
#f = open('test_data/complex_polygon.json')
f = open('test_data/buggy_file.json')
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

P = Polygon(vertices)

results = split_polygon(P)

print(results)
