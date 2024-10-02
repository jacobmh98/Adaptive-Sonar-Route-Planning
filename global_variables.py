scale = 1
epsilon = 1e-2

name_decomposition = 'antwerpen_epsilon_e6'
name_optimized_decomposition = 'antwerpen_epsilon_e6_optimized_e2'

load_existing_data = True
load_existing_optimized_polygons = True

# Start parameters
dx = 10 # Path width (Must be >0) (1 is 1 meter)
remove_parallel_vertices = True  # Can cause issues in path coverage if not removed
extern_start_end = False
if extern_start_end:
    ext_p_start = [0.0, 0.0]
    ext_p_end = [7, 6]
else:
    ext_p_start = None
    ext_p_end = None

# Choose sub-polygon sorting method
tsp_sort = False
dfs_sort = True

# Coverage path heuristic weights
distance_weight = 1
distance_to_start_weight = 1
distance_to_end_weight = 1
turn_weight = 5