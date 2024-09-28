import copy
import json
from functions import *
import pickle

from global_variables import load_existing_data

if not load_existing_data:
    # Reading the test data
    f = open('test_data/antwerpen_full.json')
    data = json.load(f)
    vertices_data = data['area']['coordinates']

    # Defining the initial polygon and the boúnding box
    vertices = []

    for i in range(len(vertices_data)):
        vertices.append(Vertex(i, vertices_data[i][0], vertices_data[i][1]))

    P = Polygon(vertices)

    # Compute the split that gives the sub-polygons
    print('running')
    sub_polygons = split_polygon(P)

    # Save the sub polygon objects
    with open('C:/Users/jacob/Documents/GitHub/Adaptive-Sonar-Route-Planning/test_data/antwerpen.pkl', 'wb') as file:
        pickle.dump(sub_polygons, file)
else:
    with open('./test_data/antwerpen.pkl', 'rb') as file:
        sub_polygons = pickle.load(file)

# Optimizing the sub-polygons
optimize_sub_polygons = optimize_polygons(copy.deepcopy(sub_polygons))

# Plotting the sub-polygons
plot_results3(optimize_sub_polygons)


# TODO har ikke testet om den stadig laver den korrekte adjacent matrix. Men tror stadig det virker
# Creating adjacent matrix for the sub-polygons to tell which sub-polygons are connected
"""m = len(sub_polygons)
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

plot_graph(G)"""