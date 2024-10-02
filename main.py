import copy
import json
import multi_poly_planning
import traveling_salesman_variation
import pickle
import traceback
from global_variables import *
from functions import *
import pickle
from global_variables import load_existing_data

if not load_existing_data:
    # Reading the test data
    f = open('test_data/antwerpen_full.json')

    data = json.load(f)
    vertices_data = data['area']['coordinates']

    # Defining the initial polygon
    vertices = []

    for i in range(len(vertices_data)):
        vertices.append(Vertex(i, vertices_data[i][0], vertices_data[i][1]))

    antwerp_poly = Polygon(vertices)

    # Compute the split that gives the sub-polygons
    print('running')
    sub_polygons = split_polygon(antwerp_poly)

    # Save the sub polygon objects
    with open(f'C:/Users/jacob/Documents/GitHub/Adaptive-Sonar-Route-Planning/test_data/{name_decomposition}.pkl', 'wb') as file:
        pickle.dump(sub_polygons, file)
else:
    f = open('test_data/antwerpen_full.json')

    print('loading')
    with open(f'./test_data/{name_decomposition}.pkl', 'rb') as file:
        sub_polygons = pickle.load(file)

    data = json.load(f)
    vertices_data = data['area']['coordinates']

    # Defining the initial polygon
    vertices = []

    for i in range(len(vertices_data)):
        vertices.append(Vertex(i, vertices_data[i][0], vertices_data[i][1]))

    antwerp_poly = Polygon(vertices)

antwerp_poly.plot()

if not load_existing_optimized_polygons:
    """# Removing collinear vertices from the sub-polygons
    for i, p in enumerate(sub_polygons):
        sub_polygons[i] = remove_collinear_vertices(p)"""

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

    # Save the optimized sub polygon objects
    with open(f'C:/Users/jacob/Documents/GitHub/Adaptive-Sonar-Route-Planning/test_data/{name_optimized_decomposition}.pkl', 'wb') as file:
        pickle.dump(optimized_sub_polygons, file)
else:
    """# Removing collinear vertices from the sub-polygons
    for i, p in enumerate(sub_polygons):
        sub_polygons[i] = remove_collinear_vertices(p)"""

    # Removing illegal sub-polygons from the unoptimized Antwerpen
    i = 0
    while i < len(sub_polygons):
        p = sub_polygons[i]

        if not is_well_formed(p)[0]:
            sub_polygons.pop(i)
            i -= 1
        i += 1

    with open(f'./test_data/{name_optimized_decomposition}.pkl', 'rb') as file:
        optimized_sub_polygons = pickle.load(file)

plot_results3(sub_polygons)
plot_results3(optimized_sub_polygons)

"""collection = []
for i, p in enumerate(sub_polygons):
    if i == 179 or i == 180 or i == 196 or i == 197 or i == 198 or i == 199 or i == 200:
        collection.append(p)

optimized_collection = optimize_polygons(copy.deepcopy(collection))
plot_results3(collection)
plot_results3(optimized_collection)"""

# TODO har ikke testet om den stadig laver den korrekte adjacent matrix. Men tror stadig det virker
# Creating adjacent matrix for the sub-polygons to tell which sub-polygons are connected
m = len(optimized_sub_polygons)
A = np.zeros((m, m))
G = nx.Graph()

# Go through each edge in p_i and compare with each edge in p_j
for i, p_i in  enumerate(optimized_sub_polygons):
    for j, p_j in enumerate(optimized_sub_polygons):
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
quit()

#collected_polygons[0].plot()
#collected_polygons[1].plot()
#test = optimize_polygons(collected_polygons)
#print(collected_polygons[0].vertices)
#print(collected_polygons[1].vertices)
# Plotting the sub-polygons
#plot_results3(test)
#plot_results3(test)

# Choosing sorting method for the order of sub polygons
if tsp_sort:
    distance_matrix = traveling_salesman_variation.create_distance_matrix(optimized_sub_polygons)
    tsp_route = traveling_salesman_variation.solve_tsp(distance_matrix)
    traveling_salesman_variation.visualize_tsp_solution(optimized_sub_polygons, tsp_route)
    polygons = [sub_polygons[i] for i in tsp_route]

elif dfs_sort:
    # Order the list of sub polygons
    adjacency_matrix, adjacency_graph = multi_poly_planning.create_adjacency(optimized_sub_polygons)
    start_node = next(iter(adjacency_graph.nodes()))
    #print(start_node)
    plot_graph(adjacency_graph)

    polygons = multi_poly_planning.sort_sub_polygons_using_dfs(adjacency_graph, optimized_sub_polygons, start_node)

else:  # Unsorted polygons
    polygons = optimized_sub_polygons

print(f'num_polygons = {len(polygons)}')

complete_path = np.empty((0,2))

plot_results3(polygons)

# If chosen, parallel vertices are removed (Can cause edge case errors if they are not removed)
if remove_parallel_vertices:
    for i,poly in enumerate(polygons):
        polygons[i] = multi_poly_planning.remove_unnecessary_vertices(poly)

    plot_results3(polygons)
total_path = multi_poly_planning.multi_path_planning(polygons, dx, extern_start_end)
multi_poly_planning.multi_poly_plot(antwerp_poly, polygons, dx, extern_start_end, ext_p_start, ext_p_end, total_path)


"""
for i, poly in enumerate(sorted_polygons):
    new_polygon = [sorted_polygons[i]]

    try:
        new_polygon = [remove_unnecessary_vertices(new_polygon[0])]  # Removing parallel vertices, minimizing number of paths generated
        total_path = multi_poly_planning.multi_path_planning(new_polygon, dx, extern_start_end)
        complete_path = np.vstack([complete_path, total_path])
        #multi_poly_planning.multi_poly_plot(new_polygon, dx, extern_start_end, p_start, p_end, np.array(total_path))
        print(f'{i} is working')

    except Exception as e:
        print(f'not working on {i}')
        traceback.print_exc()
        new_polygon[0].plot()
        break
"""