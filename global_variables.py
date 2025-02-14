from Polygon import VertexType as VT

epsilon = 1e-6
epsilon_xl = 0.2
intersection_epsilon = 1e-9

# Start parameters
global_path_width = 10.0 # Path width (Must be >0) (1 is 1 meter)
global_overlap_distance = 0 # Set to 0 for no overlap, should not be greater than path_width/2

# Path data
get_path_data = False
store_data = False

# Choose sub-polygon sorting method
dfs_sorting = True
tsp_centroid_sorting = True
tsp_intra_regional_sorting = False

# Finding optimal path width, with tolerances
find_optimal_path_width = False
tolerance = 0.5  # +- meters for path width
iterations = 50 # Number of different path widths to check

# Check path in opposite direction, can give better path
check_reverse = False

OPEN = VT.OPEN
CLOSE = VT.CLOSE
SPLIT = VT.SPLIT
MERGE = VT.MERGE
FLOOR_CONVEX = VT.FLOOR_CONVEX
FLOOR_CONCAVE = VT.FLOOR_CONCAVE
CEIL_CONVEX = VT.CEIL_CONVEX
CEIL_CONCAVE = VT.CEIL_CONCAVE
COLLINEAR_CEILING = VT.COLLINEAR_CEILING
COLLINEAR_FLOOR = VT.COLLINEAR_FLOOR