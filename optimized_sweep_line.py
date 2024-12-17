import numpy as np

from Polygon import Vertex, Polygon
from decomposition import sum_of_widths, is_well_formed
from obstacles import decompose_sweep_line, plot_obstacles

def copy_region(P):
    vertices = []

    for i, v in enumerate(P.vertices):
        new_v = Vertex(v.index, v.x, v.y)
        vertices.append(new_v)

    new_P = Polygon(vertices)

    for i, e in enumerate(P.edges):
        new_P.edges[i].is_hard_edge = e.is_hard_edge
        new_P.vertices[i].edge_from_v_is_hard = P.vertices[i].edge_from_v_is_hard
    return new_P

def copy_obstacles(obstacles):
    new_obstacles = []

    for o in obstacles:
        vertices = []

        for v in o.vertices:
            new_v = Vertex(v.index, v.x, v.y, True)
            new_v.edge_from_v_is_hard = v.edge_from_v_is_hard
            vertices.append(new_v)

        new_o = Polygon(vertices, is_obstacle=True)
        new_o.is_hard_obstacle = o.is_hard_obstacle
        new_obstacles.append(new_o)

    return new_obstacles

def rotate_system(region, obstacles):
    """ Rotate the coordinate system until no vertices share an x-coordinate """
    angle_deg = 0

    while share_x_coordinate(region, obstacles):
        angle_deg += 1

        region.rotate(1)
        #print('ROTATING')
        #print(f'{share_x_coordinate(region, obstacles)}')

        for o in obstacles:
            #print("NOT HERE")
            o.rotate(1)
        #plot_obstacles([region], obstacles, True, 'ROTATING')
    #return region, obstacles, angle_deg
    return angle_deg

def share_x_coordinate(region, obstacles):
    """ Test if any vertex in the region and obstacles shares an x-coordinate """
    seen_x = []

    for v in region.vertices:
        if len(seen_x) == 0:
            seen_x.append(v)
            continue

        for v2 in seen_x:
            if v == v2:
                continue

            if abs(v.x - v2.x) < 1e-6:
                #print(f'{v} = {v2}')
                return True

        seen_x.append(v)


    for o in obstacles:
        for v in o.vertices:
            for v2 in seen_x:
                if abs(v.x - v2.x) < 1e-6:
                    return True
            seen_x.append(v)
    return False

def optimized_sweep_line(region, obstacles):
    """ Optimizes sweep line algorithm """
    # Step 1) Collect all edges from the region and obstacles
    combined_edges = region.edges

    sum_of_widths_list = []
    sub_polygons_list = []

    for o in obstacles:
        combined_edges += o.edges

    # print(f'n_combined_edges = {len(combined_edges)}')

    # Step 2) Run the sweep line for each edge
    for i, e in enumerate(combined_edges):

        region_copy = copy_region(region)
        obstacles_copy = copy_obstacles(obstacles)

        #plot_obstacles([region_copy], obstacles_copy, False, 'copy')

        # plot_obstacles([region_copy], obstacles_copy, False, '')

        # print(f'Checking Edge {i}')
        # try:
        # Define the edge vector
        v = e.v_to.get_array() - e.v_from.get_array()
        v = v.flatten()

        # Compute the angle of the vector
        phi = np.arctan2(v[1], v[0])

        # Compute the angle to rotate to the upwards direction
        theta = np.pi / 2 - phi  # In radians
        theta_deg = np.degrees(theta)  # In degrees

        #print(f'\t{theta=}')

        # Rotate the polygon and obstacles
        region_copy.rotate(theta_deg)

        for o in obstacles_copy:
            o.rotate(theta_deg)
        #plot_obstacles([region_copy], obstacles_copy, False, 'rotated')
        # Fix vertices with equal x-coordinates
        # rotated_region, rotated_obstacles, angle_deg = rotate_system(region_copy, obstacles_copy)
        angle_deg = rotate_system(region_copy, obstacles_copy)
        #angle_deg = 0
        #print(f'\t{angle_deg=}')

        # plot_obstacles([region_copy], obstacles_copy, True, 'Rotated')
        try:
            sub_polygons = decompose_sweep_line(region_copy, obstacles_copy)
        except:
            print('CONTINUING')
            continue
        #sub_polygons_valid = []
        sub_polygons_valid = []
        for p in sub_polygons:
        #    if is_well_formed(p)[0]:
            p.rotate(-theta_deg - angle_deg)
            sub_polygons_valid.append(p)

        sow = sum_of_widths(sub_polygons_valid)

        # print(f'\tSum of widths = {sow}')
        # print(f'\tTheta deg = {theta_deg}')

        sum_of_widths_list.append(sow)
        # figures_list.append(fig)
        sub_polygons_list.append(sub_polygons_valid)
        # except Exception as e:  # Catch all exceptions
        #    print(str(e))  # Print the exception message
        #    traceback.print_exc()
        #    sum_of_widths_list.append(np.inf)
        # figures_list.append(None)
        #    sub_polygons_list.append(None)

    best_index = np.argmin(sum_of_widths_list)
    best_sub_polygons = sub_polygons_list[best_index]

    print(f'best_index = {best_index}')
    print(f'min_sum_of_width = {sum_of_widths_list[best_index]}')
    print(f'num_sub-polygons = {len(best_sub_polygons)}')

    return best_sub_polygons