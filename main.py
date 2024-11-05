import numpy as np
import json
import functions
import CPP_2D_convex_functions
import matplotlib.pyplot as plt

# Reading the test data
f1 = open('test_data/complex_polygon.json')

data = json.load(f1)

# Find antipodal vertex testing
P = np.array(data["area"]["coordinates"])
antipodal_vertices = CPP_2D_convex_functions.antipodal_vertice_pairs(P)
antipodal_vertices = CPP_2D_convex_functions.remove_parallel_antipodal_vertices(P, antipodal_vertices)  # Removing parallel vertices
diametral_point = CPP_2D_convex_functions.get_diametral_antipodal_point(P, antipodal_vertices, 2)

# Defining the vertices in counter-clock wise order
vertices = data['area']['coordinates']
vertices = np.array(vertices).T

# Number of vertices
n = vertices.shape[1]

# Creating a plot
fig = plt.figure()

# Plotting the points
x_coords = vertices[0, :]
y_coords = vertices[1, :]

# Plotting path
#path = np.array(path).T
#path_x = path[0, :]
#path_y = path[1, :]

plt.plot(x_coords, y_coords, 'b-', marker='o')  # Connect back to the first point to close the polygon
plt.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'b-')

#plt.plot(path_x, path_y, 'r-', marker='o')
plt.grid()

for i in range(n):
    x = vertices[0][i]
    y = vertices[1][i]
    plt.text(x, y, f'{i}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex

    for k in range(antipodal_vertices.shape[0]):
        xk = [P[antipodal_vertices[k, 0], 0], P[antipodal_vertices[k, 1], 0]]
        yk = [P[antipodal_vertices[k, 0], 1], P[antipodal_vertices[k, 1], 1]]
        plt.plot(xk, yk, linestyle='--', marker='o', color=[0.7, 0.7, 0.7])

plt.show()

quit()