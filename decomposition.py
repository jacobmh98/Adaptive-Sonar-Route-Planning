import numpy as np
import json
import matplotlib.pyplot as plt
from Polygon import Vertex, Polygon
from functions import *

# Reading the test data
f1 = open('test_data/complex_polygon.json')
data = json.load(f1)
vertices_data = data['area']['coordinates']

# Defining the initial polygon
vertices = []

for i, v in enumerate(vertices_data):
    vertices.append(Vertex(i, v[0], v[1]))

polygon = Polygon(vertices)

# Compute the concave vertices
polygon.concave_vertices = compute_concave_vertices(polygon)
print(f'concave vertices = {polygon.concave_vertices}')

# Creating the width-sum matrix
ncc = len(polygon.concave_vertices)
n_edges = len(polygon.edges)
D = np.empty((ncc, n_edges))

# Go through each concave vertex
for i, cv in enumerate(polygon.concave_vertices):
    split_polygons = []

    print(f"checking for {cv.index=} with coord = ({cv.x}, {cv.y})")

    # Check lines which passes the concave vertex i and parallels edge e
    for j, e in enumerate(polygon.edges):
        m = e.slope

        # Translate the line to lie on the concave vertex (slope stays the same)
        c = cv.y - m * cv.x

        # Check each edge in the polygon for intersection with line e
        intersection_points = []
        intersection_edges = []

        for e2 in polygon.edges:
            # Don't need to do anything if the line intersects itself
            if e == e2:
                continue

            # TODO handle well-formedness of intersection points

            # Check if the line intersects the polygon
            t = np.round((m * e2.v_from.x + c - e2.v_from.y) / ((e2.v_to.y - e2.v_from.y) - m * (e2.v_to.x - e2.v_from.x)),2)

            if 0 < t < 1:
                # Compute the intersection point between the line and the polygon
                x_intersect = e2.v_from.x + t * (e2.v_to.x - e2.v_from.x)
                y_intersect = e2.v_from.y + t * (e2.v_to.y - e2.v_from.y)

                intersection_points.append((x_intersect, y_intersect))
                intersection_edges.append(e2)

        # Split the polygon into sub-polygons P1 and P2 (case with 2 intersection points)
        if len(intersection_points) == 2:
            # Computing the vertices for P1
            vertices_P1 = [Vertex(0, intersection_points[0][0], intersection_points[0][1])]
            v_next = intersection_edges[0].v_to

            while v_next != intersection_edges[1].v_to:
                vertices_P1.append(Vertex(len(vertices_P1), v_next.x, v_next.y))
                v_next = v_next.next

            vertices_P1.append(Vertex(len(vertices_P1), intersection_points[1][0], intersection_points[1][1]))

            # Computing the vertices for P2
            vertices_P2 = [Vertex(0, intersection_points[1][0], intersection_points[1][1])]
            v_next = intersection_edges[1].v_to

            while v_next != intersection_edges[0].v_to:
                vertices_P2.append(Vertex(len(vertices_P2), v_next.x, v_next.y))
                v_next = v_next.next

            vertices_P2.append(Vertex(len(vertices_P2), intersection_points[0][0], intersection_points[0][1]))

            P1 = Polygon(vertices_P1)
            P2 = Polygon(vertices_P2)

            split_polygons.append((P1, P2))

            D[i, j] = compute_width_sum(P1) + compute_width_sum(P2)
        else:
            # Computing the vertices for P1
            vertices_P1 = [Vertex(0, intersection_points[0][0], intersection_points[0][1])]
            v_next = intersection_edges[0].v_to

            while v_next != cv:
                vertices_P1.append(Vertex(len(vertices_P1), v_next.x, v_next.y))
                v_next = v_next.next

            vertices_P1.append(Vertex(len(vertices_P1), cv.x, cv.y))
            #print(f'\t{vertices_P1=}')

            # Computing the vertices for P2
            vertices_P2 = [Vertex(0, cv.x, cv.y)]
            v_next = cv.next

            while v_next != intersection_edges[0].v_to:
                vertices_P2.append(Vertex(len(vertices_P2), v_next.x, v_next.y))
                v_next = v_next.next

            vertices_P2.append(Vertex(len(vertices_P2), intersection_points[0][0], intersection_points[0][1]))
            #print(f'\t{vertices_P2=}')

            P1 = Polygon(vertices_P1)
            P2 = Polygon(vertices_P2)#

            split_polygons.append((P1, P2))
            # Compute the width sum of P1 and P2
            D[i, j] = compute_width_sum(P1) + compute_width_sum(P2)
        print(f'D{i},{j} = {D[i, j]}')
    plot_results(split_polygons, D[i, :], cv.index)
plt.show()

