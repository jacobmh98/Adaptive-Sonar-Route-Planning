import time
import cpp_connect_path
import decomposition
import plot_cpp
import sorting_dfs_adjacency_graph
import sorting_tsp_centroid
import sorting_tsp_intra_regional
import cpp_path_planning
import cpp_path_data
from obstacles import *
from load_data import *

# Loading the region and obstacles
data_path = 'test_data/obstacle_multi_easy.json'
region, obstacles = get_region(data_path)

# Decompose the region using the sweep line algorithm
sub_polygons_sweep_line = decompose_sweep_line(copy.deepcopy(region), copy.deepcopy(obstacles))

# Decompose the region without considering obstacles
sub_polygons = generate_new_data(copy.deepcopy(region))

# Divide the sub-polygons into clusters that are affected by the obstacles
sub_polygons_filtered_masks = []
sub_polygons_filtered = []
obstacles_affected = []

for o in obstacles:
    filtered_mask, filtered = find_bounding_polygons(sub_polygons, o)
    common_found = False

    for index, mask in enumerate(sub_polygons_filtered_masks):
        for i in mask:
            for j in filtered_mask:
                if i == j:
                    common_found = True
                    break

            if common_found:
                break

        if common_found:
            for i, p in enumerate(filtered_mask):
                if p not in mask:
                    mask.append(p)
                    sub_polygons_filtered[index].append(filtered[i])

                    if o not in obstacles_affected[index]:
                        obstacles_affected[index].append(o)

    if not common_found:
        sub_polygons_filtered_masks.append(filtered_mask)
        sub_polygons_filtered.append(filtered)
        obstacles_affected.append([o])

# Merge each cluster into a single polygon and decompose it using sweep line algorithm
decomposed_polygons = []
extracted_sub_polygons = []
extracted_sub_polygons_mask = []
dont_include_mask = []

for list in sub_polygons_filtered_masks:
    for p in list:
        dont_include_mask.append(p)

for i, filtered in enumerate(sub_polygons_filtered):
    sub_polygons_extract, merged_sub_polygon = merge_filtered_sub_polygons(copy.deepcopy(filtered), copy.deepcopy(sub_polygons), sub_polygons_filtered_masks[i])

    merged_sub_polygon_decomposed = decompose_sweep_line(merged_sub_polygon, obstacles_affected[i])
    decomposed_polygons += merged_sub_polygon_decomposed

    for p in sub_polygons_extract:
        if p not in extracted_sub_polygons_mask and p not in dont_include_mask:
            extracted_sub_polygons_mask.append(p)
            extracted_sub_polygons.append(sub_polygons[p])

# Combining all the decomposed sub-polygons with obstacles
combined_polygons = extracted_sub_polygons + decomposed_polygons

# Optimize the sub-polygons by merging when possible
#optimized_sub_polygons = compute_optimized_data(combined_polygons)

# Starting timer for all cpp functions
total_start_time = time.time()

removed_col_sub_polygons = []
for poly in combined_polygons:
    removed_col_sub_polygons.append(decomposition.remove_collinear_vertices(poly))
combined_polygons = removed_col_sub_polygons

intersections = cpp_path_planning.multi_intersection_planning(combined_polygons, global_path_width, global_overlap_distance)

# Choosing sorting method
if dfs_sorting:
    print("DFS Sorting")
    sorted_combined_polygons, sorted_sub_poly, sorted_intersections = sorting_dfs_adjacency_graph.solve_dfs(combined_polygons, intersections)

elif tsp_centroid_sorting:
    print("TSP Centroid Sorting")
    sorted_combined_polygons, sorted_sub_poly, sorted_intersections = sorting_tsp_centroid.solve_centroid_tsp(combined_polygons, intersections)

elif tsp_intra_regional_sorting:
    print("TSP Intra Regional Sorting")
    # Using intersections start/end pairs to find optimal sorting order using tsp
    sorted_combined_polygons, sorted_sub_poly, sorted_intersections = sorting_tsp_intra_regional.solve_intra_regional_tsp(combined_polygons, intersections, trials = 2)

else:
    print('Baseline path:')
    sorted_combined_polygons = combined_polygons
    sorted_intersections = intersections

# Computing new intersections (if running intra regional) using the sorted combined polygons
path, transit_flags = cpp_connect_path.connect_path(sorted_combined_polygons, sorted_intersections, region, obstacles)

print(f'Number of polygons (main)= {len(sorted_combined_polygons)}')

# Ending timer and computing total execution time
total_end_time = time.time()
total_execution_time = total_end_time - total_start_time

if get_path_data:
    cpp_path_data.compute_path_data(region, path, transit_flags, global_path_width, obstacles, total_execution_time)

plot_cpp.plot_multi_polys_path(global_path_width, sorted_combined_polygons, path, obstacles)
