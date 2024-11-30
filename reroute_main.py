import reroute_hard_obstacles
import reroute_hard_region_edges

def hard_edges_rerouting(start_point, end_point, region, obstacles, prev_poly):

    print(f"Start point: {start_point}")
    print(f"End point: {end_point}")

    intersected_obstacle_edges = (reroute_hard_obstacles.
                                  find_obstacle_intersections(start_point, end_point, obstacles))
    print(f"initial obstacle intersections: {intersected_obstacle_edges}")

    filtered_obstacle_edges = (reroute_hard_obstacles.
                               filter_initial_intersections(intersected_obstacle_edges, obstacles, start_point, end_point))
    print(f"Initial filtered obstacle intersections: {filtered_obstacle_edges}")

    intersected_region_edges = (reroute_hard_region_edges.
                                find_region_hard_edge_intersections(start_point, end_point, region))
    print(f"Initial region intersections: {intersected_region_edges}")

    filtered_region_edges = (reroute_hard_region_edges.
                             filter_intersected_region_edges(intersected_region_edges, start_point, end_point, region))
    print(f"Initial filtered region intersections: {filtered_region_edges}")

    # Obstacle vertex intersected, but does not enter the obstacle, so we count it as a clear path
    if len(filtered_obstacle_edges) < 2 and len(filtered_region_edges) == 0: # No region edge can be intersected for a clear path
        print("No intersections, Clear path")
        return []

    # Region edge intersected, but clear from obstacles
    elif len(filtered_obstacle_edges) < 2 and len(filtered_region_edges) > 0:
        print("Region edges intersected")
        return reroute_hard_region_edges.avoid_region_edges(start_point, end_point, intersected_region_edges, region)

    # Obstacle edges intersected
    elif len(filtered_obstacle_edges) > 0: # and len(filtered_region_edges) == 0:
        print("Obstacle edges intersected")
        return reroute_hard_obstacles.avoid_obstacles(start_point, end_point, obstacles, prev_poly, filtered_obstacle_edges)

    else:
        print("No cases hit, clear path")
        return []