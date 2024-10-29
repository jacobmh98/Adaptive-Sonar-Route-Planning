import copy
import json
import pickle

from Polygon import *
from decomposition import split_polygon, is_well_formed, optimize_polygons, remove_collinear_vertices, \
    remove_equal_points


def get_region(data_path):
    # Reading the test data
    f = open(f'test_data/{data_path}.json')

    data = json.load(f)
    vertices_data = data['area']['coordinates']
    num_of_obstacles = data['obstacles']['num_of_obstacles']
    #hard_edges = data['area']['hard_edges']
    #print(hard_edges)
    # Defining the initial polygon
    vertices = []

    for i in range(len(vertices_data)):
        vertices.append(Vertex(i, vertices_data[i][0], vertices_data[i][1]))
    region = Polygon(vertices)

    obstacles = []
    for i in range(num_of_obstacles):
        vertices = []
        obs = data['obstacles'][f'obstacle_{i + 1}']

        for j in range(len(obs)):
            vertices.append(Vertex(j, obs[j][0], obs[j][1], True))

        O = Polygon(vertices, True)
        O.compute_bounding_box()
        obstacles.append(O)

    return region, obstacles

def generate_new_data(region):
    # Compute the split that gives the sub-polygons
    print('running')
    return split_polygon(region)

def compute_optimized_data(sub_polygons):
    # Removing illegal sub-polygons from the unoptimized Antwerpen
    i = 0
    while i < len(sub_polygons):
        p = sub_polygons[i]

        if not is_well_formed(p)[0]:
            sub_polygons.pop(i)
            i -= 1
        i += 1

    # Optimizing the sub-polygons (removing edges)
    optimized_sub_polygons = optimize_polygons(copy.deepcopy(sub_polygons))

    # Remove collinear vertices in each sub-polygon
    for i, p in enumerate(optimized_sub_polygons):
        p = remove_equal_points(p)
        p = remove_collinear_vertices(p)
        p.compute_bounding_box()
        optimized_sub_polygons[i] = p

    return optimized_sub_polygons

# TODO not tested if these functions works
def load_existing_data(data_path):
    print('loading')
    with open(f'./test_data/{data_path}.pkl', 'rb') as file:
        sub_polygons = pickle.load(file)
    return sub_polygons

def save_data(data_path, sub_polygons):
    # Save the sub polygon objects
    with open(f'C:/Users/jacob/Documents/GitHub/Adaptive-Sonar-Route-Planning/test_data/{data_path}.pkl',
              'wb') as file:
        pickle.dump(sub_polygons, file)