import copy
import json
import pickle
from Polygon import *
from decomposition import split_polygon, is_well_formed, optimize_polygons, remove_collinear_vertices, \
    remove_equal_points

def get_region(data_path):
    """
    Load the region and obstacles from the JSON file, and extract hard edges.
    """
    # Reading the test data
    with open(data_path, 'r') as f:
        data = json.load(f)

    # Extracting region data
    vertices_data = data['area']['coordinates']
    hard_edges = data['area']['hard_edges']

    # Create region vertices
    vertices = [Vertex(i, coord[0], coord[1]) for i, coord in enumerate(vertices_data)]
    region = Polygon(vertices)

    # Mark hard edges in the region
    for e in hard_edges:
        region.vertices[e].edge_from_v_is_hard = True
        region.edges[e].is_hard_edge = True

    # Extract obstacle data
    num_of_obstacles = data['obstacles']['num_of_obstacles']
    hard_obstacles = data['obstacles']['hard_obstacles']
    obstacles = []

    for i in range(num_of_obstacles):
        obs_data = data['obstacles'][f'obstacle_{i + 1}']
        obs_vertices = [
            Vertex(j, coord[0], coord[1], is_obstacle=True)
            for j, coord in enumerate(obs_data)
        ]

        # Mark edges as hard if specified
        for v in obs_vertices:
            v.edge_from_v_is_hard = hard_obstacles[i] == 1

        # Create an obstacle polygon
        obstacle_polygon = Polygon(obs_vertices, is_obstacle=True)
        obstacle_polygon.is_hard_obstacle = hard_obstacles[i] == 1
        obstacles.append(obstacle_polygon)

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
        #p.compute_bounding_box()
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