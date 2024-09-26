import copy
import json
from functions import *
import numpy as np
from python_tsp.exact import solve_tsp_dynamic_programming

# Reading the test data
f = open('test_data/complex_polygon3.json')

data = json.load(f)
vertices_data = data['area']['coordinates']

# Defining the initial polygon and the boúnding box
vertices = []

for i, v in enumerate(vertices_data):
    vertices.append(Vertex(i, v[0], v[1]))

P = Polygon(vertices)

# Compute the split that gives the sub-polygons
sub_polygons = split_polygon(P)
optimized_sub_polygons = optimize_polygons(copy.deepcopy(sub_polygons))
plot_polygons2(P, sub_polygons, optimized_sub_polygons)

"""
# Creating adjacent matrix for the sub-polygons to tell which sub-polygons are connected
m = len(sub_polygons)
A = np.zeros((m, m))
G = nx.Graph()

# Go through each edge in p_i and compare with each edge in p_j
for i, p_i in  enumerate(sub_polygons):
    for j, p_j in enumerate(sub_polygons):
        # Ignore if the two polygons are equal
        if i == j:
            A[i, j] = 0
            continue

        # Test if the two polygons p_i and p_j are adjacent (either complete or partial)
        if polygons_are_adjacent(p_i, p_j, i, j):
            # Update the adjacent matrix
            A[i, j] = 1
            A[j, i] = 1
            G.add_edge(f'P{i}', f'P{j}')
        else:
            A[i, j] = np.inf
            #print(f'{i} and {j} are adjacent')
#print(adj_matrix)
#print(A)
# Replace 0s (disconnected) with large value, except diagonal (self-loops)
#adj_matrix = np.where(adj_matrix == 0, 999999, adj_matrix)

# Solve TSP
#permutation, distance = solve_tsp_dynamic_programming(A)
print(permutation)
plot_polygons(P, sub_polygons, G)
"""