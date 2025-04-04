import reroute_hard_obstacles
import reroute_hard_region_edges

def hard_edges_rerouting(start_point, end_point, region, obstacles, i, polygons):
    """ Handles rerouting around hard edges, considering both region and obstacle intersections.

    :param start_point: Tuple (x, y) representing the starting point of the path.
    :param end_point: Tuple (x, y) representing the ending point of the path.
    :param region: The Polygon object representing the current region.
    :param obstacles: List of obstacles containing edges to avoid.
    :param i: Index of the current polygon
    :param polygons: The list of sub polygons
    :return: List of intermediate points for the rerouted path, or an empty list if the path is clear.
    """

    ignore_hard_edges = False

    if ignore_hard_edges:
        return []

    intersected_obstacle_edges = (reroute_hard_obstacles.
                                  compute_obstacle_intersections(start_point, end_point, obstacles))

    filtered_obstacle_edges = (reroute_hard_obstacles.
                               filter_initial_obstacle_intersections(start_point, end_point, intersected_obstacle_edges, obstacles))

    intersected_region_edges = (reroute_hard_region_edges.
                                compute_hard_region_edge_intersections(start_point, end_point, region))

    filtered_region_edges = (reroute_hard_region_edges.
                             filter_intersected_region_edges(start_point, end_point, intersected_region_edges, region))

    # Obstacle vertex intersected, but does not enter the obstacle, so we count it as a clear path
    if len(filtered_obstacle_edges) < 2 and len(filtered_region_edges) == 0: # No region edge can be intersected for a clear path
        return []

    # Region edge intersected, but clear from obstacles
    elif len(filtered_obstacle_edges) < 2 and len(filtered_region_edges) > 0:
        return reroute_hard_region_edges.reroute_path_region(start_point, end_point, intersected_region_edges, region, i, polygons)

    # Obstacle edges intersected
    elif len(filtered_obstacle_edges) > 0: # and len(filtered_region_edges) == 0:
        return reroute_hard_obstacles.reroute_path_obstacles(start_point, end_point, obstacles, polygons[i-1], filtered_obstacle_edges)

    # Clear path
    else:
        return []