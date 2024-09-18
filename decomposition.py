from json.encoder import INFINITY

import numpy as np
import json

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
#polygon.plot()

# Compute the concave vertices
polygon.concave_vertices = compute_concave_vertices(polygon)
print(f'concave vertices = {polygon.concave_vertices}')

split_polygons = []

# Go through each concave vertex
for i, cv in enumerate(polygon.concave_vertices):
    print(f"checking for {cv.index=} with coord = ({cv.x}, {cv.y})")

    # Check lines which passes the concave vertex i and parallels edge e
    for j, e in enumerate(polygon.edges):
        intersection_points = []
        intersection_edges = []
        intersection_directions = []

        # Define a vector from the vertices in edge e end its negation
        vec = e.v_to.v - e.v_from.v

        # Go through each edge in the polygon
        for e2 in polygon.edges:
            # Don't care about the intersection if the two lines are equal
            if e == e2:
                continue

            # Parameterize the edge and vector
            coefficient = np.concatenate((vec, -(e2.v_to.get_array() - e2.v_from.get_array())), axis=1)
            rhs = e2.v_from.get_array() - cv.get_array()

            try:
                x = np.linalg.solve(coefficient, rhs)
                t = x[0]
                s = x[1]

                # Test if the line e intersects the edge e2
                if 0 <= s <= 1:
                    # Compute the coordinates of the intersection point
                    intersection_P = cv.get_array() + t * vec

                    # Ignore the intersection point that happens in the concave vertex
                    if points_are_equal(intersection_P, cv.get_array()):
                        continue

                    # Ignore the intersection that happens in the adjacent edges
                    if t < 0 and e.v_from == e2.v_to:
                        continue
                    if t > 0 and e.v_to == e2.v_from:
                        continue

                    print(f'\t{e} intersects {e2} at ({intersection_P[0,0]}, {intersection_P[1,0]}) with direction t={t[0]}')
                    intersection_points.append(intersection_P)
                    intersection_edges.append(e2)
                    intersection_directions.append(t)
            except:
                None

        # Split the polygon into sub-polygons at a single intersection point
        if len(intersection_points) == 1:
            e2 = intersection_edges[0]
            v0 = Vertex(0, intersection_points[0][0, 0], intersection_points[0][1, 0])
            P1_vertices = [v0]
            v_next = e2.v_to
            v_index = 1
            while v_next != cv:
                P1_vertices.append(Vertex(v_index, v_next.x, v_next.y))
                v_index += 1
                v_next = v_next.next

            vn = Vertex(v_index, v_next.x, v_next.y)
            P1_vertices.append(vn)

            v0 = Vertex(0, cv.x, cv.y)
            P2_vertices = [v0]
            v_next = cv.next
            v_index = 1

            while v_next != e2.v_from:
                P2_vertices.append(Vertex(v_index, v_next.x, v_next.y))
                v_index += 1
                v_next = v_next.next

            vn = Vertex(v_index, intersection_points[0][0, 0], intersection_points[0][1, 0])
            P2_vertices.append(vn)

            P1 = Polygon(P1_vertices)
            P2 = Polygon(P2_vertices)
            split_polygons.append((P1, P2))

        # Split the polygon into sub-polygons given two intersection points in different directions
        elif len(intersection_points) == 2 and intersection_directions[0] * intersection_directions[1] < 0:
            e2_1 = intersection_edges[0]
            e2_2 = intersection_edges[1]

            v0 = Vertex(0, intersection_points[0][0, 0], intersection_points[0][1, 0])
            P1_vertices = [v0]
            v_next = e2_1.v_to
            v_index = 1
            while v_next != e2_2.v_to:
                P1_vertices.append(Vertex(v_index, v_next.x, v_next.y))
                v_index += 1
                v_next = v_next.next

            vn = Vertex(v_index, intersection_points[1][0, 0], intersection_points[1][1, 0])
            P1_vertices.append(vn)

            v0 = Vertex(0, intersection_points[1][0, 0], intersection_points[1][1, 0])
            P2_vertices = [v0]
            v_next = e2_2.v_to
            v_index = 1

            while v_next != e2_1.v_to:
                P2_vertices.append(Vertex(v_index, v_next.x, v_next.y))
                v_index += 1
                v_next = v_next.next

            vn = Vertex(v_index, intersection_points[0][0, 0], intersection_points[0][1, 0])
            P2_vertices.append(vn)

            P1 = Polygon(P1_vertices)
            P2 = Polygon(P2_vertices)
            split_polygons.append((P1, P2))
    break

print(split_polygons)

# tmp plot
rows = 3
cols = 5
fig, ax = plt.subplots(rows, cols)

r = 0
c = 0
print(len(split_polygons))
for (P1, P2) in split_polygons:
    P1_coords = P1.vertices_matrix()
    P2_coords = P2.vertices_matrix()
    ax[r, c].plot(P1_coords[0, :], P1_coords[1, :], 'b-', marker='o')
    ax[r, c].plot([P1_coords[0, :][-1], P1_coords[0, :][0]], [P1_coords[1, :][-1], P1_coords[1, :][0]], 'b-')
    ax[r, c].plot(P2_coords[0, :], P2_coords[1, :], 'r-', marker='o')
    ax[r, c].plot([P2_coords[0, :][-1], P2_coords[0, :][0]], [P2_coords[1, :][-1], P2_coords[1, :][0]], 'r-')
    ax[r, c].set_title(f"v_")

    #ax[r, c].quiver(cv.x, cv.y, vec[0], vec[1], angles='xy', scale_units='xy', scale=1, color='r', width=0.015)
    #ax[plot_r, c].quiver(cv.x, cv.y, v_dir2[0], v_dir2[1], angles='xy', scale_units='xy', scale=1, color='g')
    c += 1

    if c != 0 and c % cols == 0:
        r += 1
        c = 0
fig.tight_layout()
plt.show()
quit()