from json.encoder import INFINITY

import numpy as np
import json
import matplotlib.pyplot as plt
from Polygon import Vertex, Polygon
from functions import *

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

# Compute the concave vertices
polygon.concave_vertices = compute_concave_vertices(polygon)
print(f'concave vertices = {polygon.concave_vertices}')

splitting_vectors = []

fig, ax = plt.subplots(3, 5)
plot_r = 0
plot_c = 0
max_c = 5

# Go through each concave vertex
for i, cv in enumerate(polygon.concave_vertices):
    split_polygons = []

    print(f"checking for {cv.index=} with coord = ({cv.x}, {cv.y})")

    # Check lines which passes the concave vertex i and parallels edge e
    for j, e in enumerate(polygon.edges):
        print(f'\tedge from {e.v_from} to {e.v_to}')
        # Define a vector from the vertices in edge e end its negation
        v_dir1 = e.v_to.v - e.v_from.v
        v_dir2 = -v_dir1

        # TODO fix for floats later
        # TODO test the cases below and fix errors if any

        # Extend the vector and its negation to the boundary box of the polygon

        if v_dir1[0] > 0 and v_dir1[1] < 0: # positive x, negative y
            t1_limx = (max_x - cv.x) / v_dir1[0]
            t1_limy = (min_y - cv.y) / v_dir1[1]
            t1 = np.min(np.array([t1_limx, t1_limy]))

            t2_limx = (min_x - cv.x) / v_dir2[0]
            t2_limy = (max_y - cv.y) / v_dir2[1]
            t2 = np.min(np.array([t2_limx, t2_limy]))
            
            v_dir1 = v_dir1 * t1
            v_dir2 = v_dir2 * t2
        elif v_dir1[0] > 0 and v_dir1[1] == 0: # positive x, neutral y
            t1 = (max_x - cv.x) / v_dir1[0]
            t2 = (min_x - cv.x) / v_dir2[0]

            v_dir1 = v_dir1 * t1
            v_dir2 = v_dir2 * t2
        elif v_dir1[0] > 0 and v_dir1[1] > 0: # positive x, positive y
            t1_limx = (max_x - cv.x) / v_dir1[0]
            t1_limy = (max_y - cv.y) / v_dir1[1]
            t1 = np.min(np.array([t1_limx, t1_limy]))

            t2_limx = (min_x - cv.x) / v_dir2[0]
            t2_limy = (min_y - cv.y) / v_dir2[1]
            t2 = np.min(np.array([t2_limx, t2_limy]))

            v_dir1 = v_dir1 * t1
            v_dir2 = v_dir2 * t2
        elif v_dir1[0] == 0 and v_dir1[1] > 0: # neutral x, positive y
            t1 = (max_y - cv.x) / v_dir1[0]
            t2 = (min_y - cv.x) / v_dir2[0]

            v_dir1 = v_dir1 * t1
            v_dir2 = v_dir2 * t2
        elif v_dir1[0] < 0 and v_dir1[1] > 0:  # negative x, positive y
            t1_limx = (min_x - cv.x) / v_dir1[0]
            t1_limy = (max_y - cv.y) / v_dir1[1]
            t1 = np.min(np.array([t1_limx, t1_limy]))

            t2_limx = (max_x - cv.x) / v_dir2[0]
            t2_limy = (min_y - cv.y) / v_dir2[1]
            t2 = np.min(np.array([t2_limx, t2_limy]))

            v_dir1 = v_dir1 * t1
            v_dir2 = v_dir2 * t2
        elif v_dir1[0] < 0 and v_dir1[1] == 0: # negative x, neutral y
            t1 = (min_x - cv.x) / v_dir1[0]
            t2 = (max_x - cv.x) / v_dir2[0]

            v_dir1 = v_dir1 * t1
            v_dir2 = v_dir2 * t2
        elif v_dir1[0] < 0 and v_dir1[0] < 0: # negative x, negative y
            t1_limx = (min_x - cv.x) / v_dir1[0]
            t1_limy = (min_y - cv.y) / v_dir1[1]
            t1 = np.min(np.array([t1_limx, t1_limy]))

            t2_limx = (max_x - cv.x) / v_dir2[0]
            t2_limy = (max_y - cv.y) / v_dir2[1]
            t2 = np.min(np.array([t2_limx, t2_limy]))

            v_dir1 = v_dir1 * t1
            v_dir2 = v_dir2 * t2
        elif v_dir1[0] == 0 and v_dir1[1] < 0: # neutral x, negative y
            t1 = (min_y - cv.x) / v_dir1[0]
            t2 = (max_y - cv.x) / v_dir2[0]

            v_dir1 = v_dir1 * t1
            v_dir2 = v_dir2 * t2

        coords = polygon.vertices_matrix()



       # for v in polygon.vertices:
        #    ax[plot_r, plot_c].text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex

        ax[plot_r, plot_c].plot(coords[0, :], coords[1, :], 'b-', marker='o')
        ax[plot_r, plot_c].plot([coords[0, :][-1], coords[0, :][0]], [coords[1, :][-1], coords[1, :][0]], 'b-')
        ax[plot_r, plot_c].quiver(cv.x, cv.y, v_dir1[0], v_dir1[1], angles='xy', scale_units='xy', scale=1, color='r')
        ax[plot_r, plot_c].quiver(cv.x, cv.y, v_dir2[0], v_dir2[1], angles='xy', scale_units='xy', scale=1, color='g')
        plot_c += 1

        if plot_c != 0 and plot_c % max_c == 0:
            plot_r += 1
            plot_c = 0
    plt.show()
    quit()


