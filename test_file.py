import numpy as np
import json
import matplotlib.pyplot as plt

# Reading the test data
f1 = open('test_data/complex_polygon.json')
f2 = open('test_data/simple_rectangle.json')

data = json.load(f1)

# Defining the vertices in counter-clock wise order
vertices = data['area']['coordinates']
vertices = np.array(vertices).T

# Number of vertices
n = vertices.shape[1]

# Number of concave vertices
ncc = 0
concave_vertices = []

# Determine the concave vertices
for i in range(n):
    # Find the adjacent vertices
    v_minus = vertices[:, (i - 1) % n]
    v = vertices[:, i % n]
    v_plus = vertices[:, (i + 1) % n]

    # Computing the concave judgement matrix
    S_vi = np.linalg.det(np.array([[v_minus[0], v_minus[1], 1],
                                   [v[0], v[1], 1],
                                   [v_plus[0], v_plus[1], 1]]))

    # Test if v_i is concave
    if S_vi < 0:
        ncc += 1
        concave_vertices.append(i)

print(f'{concave_vertices=}')

for i in range(ncc):
    concave_v = concave_vertices[i]
    print(f"checking for {concave_v=}")

    # Check lines which passes vertex i and parallels edge j
    for j in range(n):
        # Slope-Intercept form for line from vertex j to vertex j+1
        x1, y1 = vertices[:, j]
        x2, y2 = vertices[:, (j + 1) % n]

        m1 = y2 - y1 / x2 - x1
        c1 = y1 - m1 * x1

        print(f'\tSlope-Intercept form from vertex {j} to {(j + 1) % n} m1 = {np.round(m1, 2)}, c1 = {np.round(c1, 2)}')

        # Check each line in the polygon for intersection
        for j2 in range(n):
            if j == j2:
                continue
            x3, y3 = vertices[:, j2]
            x4, y4 = vertices[:, (j2 + 1) % n]

            t = (m1 * x3 + c1 - y3) / ((y4 - y3) - m1 * (x4 - x3))

            print(f'\t\tmaking line from {j2} to {(j2 + 1) % n}, t={np.round(t, 2)}')

    # TMP break
    break

# Creating a plot
fig = plt.figure()

# Plotting the points
x_coords = vertices[0, :]
y_coords = vertices[1, :]

plt.plot(x_coords, y_coords, 'b-', marker='o')  # Connect back to the first point to close the polygon
plt.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'b-')

for i in range(n):
    x = vertices[0][i]
    y = vertices[1][i]
    plt.text(x, y, f'{i}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex

plt.show()

quit()
ax.plot(x, y, 'b-', linewidth=1)
ax.plot([x[-1], x[0]], [y[-1], y[0]], 'b-', linewidth=1)

ax.set_title('Polygon from points')
ax.set_xlabel('x (km)')
ax.set_ylabel('y (km)')

ax.set_aspect('equal')
plt.show()