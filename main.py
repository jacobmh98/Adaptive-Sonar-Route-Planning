import coverage_plots
import multi_poly_planning
import path_comparison_functions
import pickle
from Polygon import Polygon
from global_variables import *
import time

from collections import defaultdict

def combine_sub_polygons(sub_polygons):
    edge_count = defaultdict(int)
    edge_map = {}

    # Step 1: Collect all edges and count occurrences
    for poly in sub_polygons:
        for edge in poly.edges:
            # Represent each edge as a tuple of sorted vertex indices to make it direction-agnostic
            v_from, v_to = edge.v_from.index, edge.v_to.index
            edge_key = tuple(sorted((v_from, v_to)))
            edge_count[edge_key] += 1
            edge_map[edge_key] = edge  # Map to the actual Edge object

    # Step 2: Identify outer edges (those that appear only once)
    outer_edges = [edge_map[edge_key] for edge_key, count in edge_count.items() if count == 1]

    # Step 3: Build a continuous path from outer edges
    outer_vertices = []
    visited_vertices = set()

    # Find a starting point for the boundary
    start_edge = outer_edges[0]
    current_edge = start_edge

    while len(visited_vertices) < len(outer_edges):
        # Add the vertex at the start of the edge if not visited
        if current_edge.v_from not in visited_vertices:
            outer_vertices.append(current_edge.v_from)
            visited_vertices.add(current_edge.v_from)

        # Add the vertex at the end of the edge if not visited
        if current_edge.v_to not in visited_vertices:
            outer_vertices.append(current_edge.v_to)
            visited_vertices.add(current_edge.v_to)

        # Find the next edge in the boundary
        next_edge = None
        for edge in outer_edges:
            if edge not in visited_vertices:
                if edge.v_from == current_edge.v_to:
                    next_edge = edge
                    break
                elif edge.v_to == current_edge.v_to:
                    # Reverse direction if needed to continue the path
                    edge.v_from, edge.v_to = edge.v_to, edge.v_from
                    next_edge = edge
                    break

        if next_edge:
            current_edge = next_edge
        else:
            break  # No more edges to add, boundary is closed

    # Step 4: Create a new Polygon object with the list of outer vertices
    region_polygon = Polygon(outer_vertices)
    return region_polygon


antwerp = False
if antwerp:
    with open('test_data/antwerp_comparison.pkl', 'rb') as file:
        optimized_polygons = pickle.load(file)

else:
    with open('test_data/simple_complex_comparison.pkl', 'rb') as file:
        optimized_polygons = pickle.load(file)

region = combine_sub_polygons(optimized_polygons)

if dfs_sort:
    # Order the list of sub polygons
    adjacency_matrix, adjacency_graph = multi_poly_planning.create_adjacency(optimized_polygons)
    start_node = next(iter(adjacency_graph.nodes()))
    #functions.plot_graph(adjacency_graph)
    polygons = multi_poly_planning.sort_sub_polygons_using_dfs(adjacency_graph, optimized_polygons, start_node)

else:  # Unsorted polygons
    polygons = optimized_polygons

start_time = time.time()
total_path = multi_poly_planning.multi_path_planning(polygons, path_width, extern_start_end)
end_time = time.time()
elapsed_time = end_time - start_time

path_comparison_functions.compute_path_data(region, total_path, elapsed_time)
coverage_plots.multi_poly_plot(region, path_width, polygons, total_path)
