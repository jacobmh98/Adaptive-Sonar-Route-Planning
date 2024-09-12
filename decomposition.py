import numpy as np
import json
import matplotlib.pyplot as plt
from Polygon import Vertex, Polygon

# Reading the test data
f1 = open('test_data/complex_polygon.json')
f2 = open('test_data/simple_rectangle.json')
f3 = open('test_data/single_concave_vertex.json')

data = json.load(f3)
vertices_data = data['area']['coordinates']

# Defining the initial polygon
vertices = []

for i, v in enumerate(vertices_data):
    vertices.append(Vertex(i, v[0], v[1]))

polygon = Polygon(vertices)
split_polygons = []

# Compute the concave vertices
polygon.compute_concave_vertices()
print(f'concave vertices = {polygon.concave_vertices}')

# Go through each concave vertex
for cv in polygon.concave_vertices:
    print(f"checking for {cv.index=} with coord = ({cv.x}, {cv.y})")

    # Check lines which passes vertex cv and parallels edge j
    for j in polygon.edges:
        m = j.slope

        # Translate the line to lie on the concave vertex (slope stays the same)
        c = cv.y - m * cv.x

        # Check each edge in the polygon for intersection with line j
        for j2 in polygon.edges:
            if j == j2:
                continue

            # TODO extend to be able to handle multiple intersection points
            # Computing the intersection point the line has with the polygon
            t = np.round((m * j2.v_from.x + c - j2.v_from.y) / ((j2.v_to.y - j2.v_from.y) - m * (j2.v_to.x - j2.v_from.x)),2)

            if 0 < t < 1:
                x_intersect = j2.v_from.x + t * (j2.v_to.x - j2.v_from.x)
                y_intersect = j2.v_from.y + t * (j2.v_to.y - j2.v_from.y)
                print(f'\t{j} intersects {j2} at ({x_intersect}, {y_intersect})')

                # Splitting the polygon into two sub-polygons P1 and P2 by the intersection point
                vertices_P1 = [Vertex(0, cv.x, cv.y)]
                v = cv.next
                while v.index != j2.v_to.index:
                    vertices_P1.append(Vertex(len(vertices_P1), v.x, v.y))
                    v = v.next
                vertices_P1.append(Vertex(len(vertices_P1), x_intersect, y_intersect))
                print(f'\t\t{vertices_P1=}')

                vertices_P2 = [Vertex(0, x_intersect, y_intersect)]
                v = j2.v_to
                while v != cv.next:
                    vertices_P2.append(Vertex(len(vertices_P2), v.x, v.y))
                    v = v.next
                print(f'\t\t{vertices_P2=}')

                P1 = Polygon(vertices_P1)
                P2 = Polygon(vertices_P2)

                split_polygons.append((P1, P2))

# Create a figure with a grid of subplots (2 rows, 4 columns)
fig, axs = plt.subplots(2, 4, figsize=(12, 8))

# Initial Polygon
x_coords, y_coords = polygon.get_coords()
axs[0, 0].plot(x_coords, y_coords, 'b-', marker='o')
axs[0, 0].plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'b-')
axs[0, 0].set_title('Initial Polygon (1 concave vertex)')

# Hide the remaining three axes in the first row
for i in range(1, 4):
    fig.delaxes(axs[0, i])

for i, (P1, P2) in enumerate(split_polygons):
    P1_xcoords, P1_ycoords = P1.get_coords()
    P2_xcoords, P2_ycoords = P2.get_coords()

    axs[1, i].plot(P1_xcoords, P1_ycoords, 'b-', marker='o')
    axs[1, i].plot([P1_xcoords[-1], P1_xcoords[0]], [P1_ycoords[-1], P1_ycoords[0]], 'b-')
    axs[1, i].plot(P2_xcoords, P2_ycoords, 'r-', marker='o')
    axs[1, i].plot([P2_xcoords[-1], P2_xcoords[0]], [P2_ycoords[-1], P2_ycoords[0]], 'r-')

    axs[1, i].set_title(f'Split Polygons {i + 1}')
plt.show()