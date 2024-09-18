"""import numpy as np
import json
import matplotlib.pyplot as plt
from Polygon import Vertex, Polygon
from functions import *

# Reading the test data
# f1 = open('test_data/complex_polygon.json')
f2 = open('test_data/intersection_in_vertex.json')
data = json.load(f2)
vertices_data = data['area']['coordinates']

# Defining the initial polygon
vertices = []

for i, v in enumerate(vertices_data):
    vertices.append(Vertex(i, v[0], v[1]))

polygon = Polygon(vertices)
polygon.plot()

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

        print(f'\tchecking edge from {e.v_from} to {e.v_to}')

        for e2 in polygon.edges:
            # Don't need to do anything if the line intersects itself
            if e == e2:
                continue

            # Check if the line intersects the polygon
            # if (e2.v_to.y - e2.v_from.y) - m * (e2.v_to.x - e2.v_from.x) == 0:
            #   t = 0
            # else:
            print(f'checking for {e2}')
            t = np.round(
                (m * e2.v_from.x + c - e2.v_from.y) / ((e2.v_to.y - e2.v_from.y) - m * (e2.v_to.x - e2.v_from.x)), 2)

            if 0 <= t <= 1:
                # Compute the intersection point between the line and the polygon
                x_intersect = e2.v_from.x + t * (e2.v_to.x - e2.v_from.x)
                y_intersect = e2.v_from.y + t * (e2.v_to.y - e2.v_from.y)

                print(f'\t\tedge {e} intersects {e2} at {x_intersect}, {y_intersect}')

                intersection_points.append((x_intersect, y_intersect))
                intersection_edges.append(e2)
        """
"""
        # Handling simple case of a single intersection point
        if len(intersection_points) < 2:
            # Computing the vertices for P1
            vertices_P1 = [Vertex(0, intersection_points[0][0], intersection_points[0][1])]
            v_next = intersection_edges[0].v_to

            while v_next != cv:
                vertices_P1.append(Vertex(len(vertices_P1), v_next.x, v_next.y))
                v_next = v_next.next

            vertices_P1.append(Vertex(len(vertices_P1), cv.x, cv.y))

            # Computing the vertices for P2
            vertices_P2 = [Vertex(0, cv.x, cv.y)]
            v_next = cv.next

            while v_next != intersection_edges[0].v_to:
                vertices_P2.append(Vertex(len(vertices_P2), v_next.x, v_next.y))
                v_next = v_next.next

            vertices_P2.append(Vertex(len(vertices_P2), intersection_points[0][0], intersection_points[0][1]))

            P1 = Polygon(vertices_P1)
            P2 = Polygon(vertices_P2)

            split_polygons.append((P1, P2))

            # Compute the width sum of P1 and P2
            D[i, j] = compute_width_sum(P1) + compute_width_sum(P2)
        # Handling case with multiple intersection points
        else:
            print(f'\tedge from {e.v_from.index} to {e.v_to.index}')
            # Draw vectors from the concave vertex to each intersection point
            # Order the vectors into two lists that contains the vectors in opposite directions
            # Compute the distances between each vector and the concave vertex
            intersection_vectors_dir1 = []
            intersection_vectors_dir1_dist = []
            intersection_edges_dir1 = []

            intersection_vectors_dir2 = []
            intersection_vectors_dir2_dist = []
            intersection_edges_dir2 = []

            for k in range(0, len(intersection_points)):
                vk = intersection_points[k] - np.array([cv.x, cv.y])

                if k == 0:
                    intersection_vectors_dir1.append(vk)
                    intersection_vectors_dir1_dist.append(distance(np.array([cv.x, cv.y]), vk))
                    intersection_edges_dir1.append(intersection_edges[k])
                    continue

                v0 = intersection_vectors_dir1[0]

                if np.dot(v0, vk) > 0: # vk has same direction as v0
                    intersection_vectors_dir1.append(vk)
                    intersection_vectors_dir1_dist.append(distance(v0, vk))
                    intersection_edges_dir1.append(intersection_edges[k])
                else: # vk has opposite direction as v0
                    intersection_vectors_dir2.append(vk)
                    intersection_vectors_dir2_dist.append(distance(v0, vk))
                    intersection_edges_dir2.append(intersection_edges[k])

            # Convert lists to numpy array
            intersection_points = np.array(intersection_points)
            intersection_vectors_dir1 = np.array(intersection_vectors_dir1)
            intersection_vectors_dir2 = np.array(intersection_vectors_dir2)

            # Sort each list based on distance to the concave vertex
            mask1 = np.argsort(np.array(intersection_vectors_dir1_dist))
            intersection_vectors_dir1_sorted = intersection_vectors_dir1[mask1]
            #intersection_edges_dir1_sorted = intersection_edges[mask1]

            mask2 = np.argsort(np.array(intersection_vectors_dir2_dist))
            intersection_vectors_dir2_sorted = intersection_vectors_dir2[mask2]
            #intersection_edges_dir2_sorted = intersection_edges[mask2]

            # Follow each vector in dir1 to compute the correct intersection
            for vk in intersection_vectors_dir1_sorted:
                # Check if intersection is in a vertex
                #if
                print(f'\t\tintersection in dir_1 at {np.array([cv.x, cv.y]) + vk}')

            for vk in intersection_vectors_dir2_sorted:
                print(f'\t\tintersection in dir_2 at {np.array([cv.x, cv.y]) + vk}')

            # Follow each vector and check if the intersection it makes is in an edge then stop

        """
        """
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

        """
    # plot_results(split_polygons, D[i, :], cv.index)


