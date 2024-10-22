scale = 1

epsilon = 1e-6
epsilon_medium = 1e-3
epsilon_large = 1e-2
epsilon_xl = 0.2

#small_epsilon = 1e-6

optimize_epsilon = 0.

intersection_epsilon = 1e-9

data_path = 'single_obstacle'
name_decomposition = ''
name_optimized_decomposition = ''

load_existing_data = False
load_existing_optimized_polygons = False
save_data = False

# Start parameters
path_width = 20 # Path width (Must be >0) (1 is 1 meter)
overlap_distance = 0 # Set to 0 for no overlap, should not be >path_width/2

# Path data
get_path_data = True
store_data = True

# Choose sub-polygon sorting method
tsp_sorting = True
dfs_sort = True

# Finding optimal path width, with tolerances
find_optimal_path_width = False
tolerance = 0.5  # +- meters for path width
iterations = 50 # Number of different path widths to check

# Check path in opposite direction, can give better path
check_reverse = False