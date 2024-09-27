import copy
import json
from functions import *
import pickle

# Reading the test data
f1 = open('test_data/antwerpen_part1.json')
f2 = open('test_data/antwerpen_part2.json')
data1 = json.load(f1)
data2 = json.load(f2)
vertices_data1 = data1['area']['coordinates']
vertices_data2 = data2['area']['coordinates']

# Defining the initial polygon and the boúnding box
vertices1 = []
vertices2 = []

for i, v in enumerate(vertices_data1):
    vertices1.append(Vertex(i, v[0], v[1]))

for i, v in enumerate(vertices_data2):
    vertices2.append(Vertex(i, v[0], v[1]))

P1 = Polygon(vertices1)
P2 = Polygon(vertices2)

# Compute the split that gives the sub-polygons
run = False
while run:
    try:
        print('running 1')
        sub_polygons1 = split_polygon(P1)
        run = False
    except:
        None

while run:
    try:
        print('running 2')
        sub_polygons2 = split_polygon(P2)
        run = False
    except:
        None

#optimized_sub_polygons = optimize_polygons(copy.deepcopy(sub_polygons1))
#optimized_sub_polygons2 = optimize_polygons(copy.deepcopy(sub_polygons2))

#sub_polygons = optimized_sub_polygons + optimized_sub_polygons2
#with open('C:/Users/jacob/Documents/GitHub/Adaptive-Sonar-Route-Planning/test_data/antwerpen.pkl', 'wb') as file:
#    pickle.dump(sub_polygons, file)

# TODO Det her er sub polygonerne du skal bruge
with open('./test_data/antwerpen.pkl', 'rb') as file:
    sub_polygons = pickle.load(file)
plot_results3(sub_polygons)

# TODO har ikke testet om den stadig laver den korrekte adjacent matrix for adjacent polygoner
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

plot_graph(G)

plt.show()