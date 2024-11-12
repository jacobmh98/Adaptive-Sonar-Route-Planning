import copy

import numpy as np
from matplotlib import pyplot as plt

from decomposition import dot, cross, get_center_of_polygon, optimize_polygons, find_shared_edge, polygons_are_adjacent, \
    points_are_equal, remove_collinear_vertices, remove_equal_points
from Polygon import Edge, Vertex, Polygon
from global_variables import *

from rtree import index

from load_data import generate_new_data

def compute_intersection(ray_start, ray_dir, seg_A, seg_B):
    # Vector from A to B (the direction of the segment)
    seg_dir = seg_B - seg_A

    # Matrix form: [v_x, -(x_B - x_A)] and [v_y, -(y_B - y_A)]
    matrix = np.array([
        [ray_dir[0], -(seg_B[0] - seg_A[0])],
        [ray_dir[1], -(seg_B[1] - seg_A[1])]
    ])

    # Vector difference (A - P_0)
    diff = seg_A - ray_start

    # Solve for t and u (using np.linalg.solve to solve the system of linear equations)
    try:
        t, u = np.linalg.solve(matrix, diff)
    except np.linalg.LinAlgError:
        # If the matrix is singular, the lines are parallel or coincident
        return None

    # Check if the solution is valid (t >= 0 for ray, u in [0, 1] for segment)
    if t >= 0 and 0 < u < 1:
        # Return the intersection point
        intersection_point = ray_start + t * ray_dir
        return intersection_point
    else:
        # No valid intersection
        return None

def compute_intersection_edges(e, e2):
    """
        Compute the intersection point of two line segments defined by points p1, p2 and p3, p4.

        Parameters:
            p1, p2: Tuple[float, float] - Endpoints of the first segment
            p3, p4: Tuple[float, float] - Endpoints of the second segment

        Returns:
            Tuple[float, float] or None: The intersection point (x, y) if the segments intersect, else None.
        """

    x1, y1 = e.v_from.get_array().flatten()
    x2, y2 = e.v_to.get_array().flatten()
    x3, y3 = e2.v_from.get_array().flatten()
    x4, y4 = e2.v_to.get_array().flatten()

    # Calculate the determinants
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denominator == 0:
        return None  # Lines are parallel or collinear, no intersection

    # Calculate the numerators
    t_num = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)
    u_num = (x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)

    # Calculate the parameters t and u
    t = t_num / denominator
    u = u_num / denominator

    # Check if the intersection point is within both line segments
    if 0 <= t <= 1 and 0 <= u <= 1:
        # Compute the intersection point
        intersection_x = x1 + t * (x2 - x1)
        intersection_y = y1 + t * (y2 - y1)
        return (intersection_x, intersection_y)

    return None  # Segments do not intersect within the segment bounds

def decompose_sweep_line(sub_polygon, obstacles):
    """ Decompose a convex around a contained obstacle"""
    # Combining the vertices and edges of the polygon and obstacle
    combined_vertices = sub_polygon.vertices
    combined_edges = sub_polygon.edges

    for o in obstacles:
        combined_vertices += o.vertices
        combined_edges += o.edges

    """    if obstacle is None:
        combined_vertices = sub_polygon.vertices
        combined_edges = sub_polygon.edges
    else:
        combined_vertices = sub_polygon.vertices + obstacle.vertices
        combined_edges = sub_polygon.edges + obstacle.edges"""

    # Insertion Sort for the events based on x-coordinate in ascending order
    combined_vertices_sorted = []

    for i, v in enumerate(combined_vertices):
        if i == 0:
            combined_vertices_sorted.append(v)
            continue

        for j, v2 in enumerate(combined_vertices_sorted):
            if j == i:
                continue

            if v2.x >= v.x:
                combined_vertices_sorted.insert(j, v)
                break

            if j == len(combined_vertices_sorted) - 1:
                combined_vertices_sorted.append(v)

    # Define the event type for each vertex
    for v in combined_vertices_sorted:
        if abs(angle(v) - np.pi) < epsilon:
            if v.prev.x > v.x and v.next.x < v.x:
                v.type = COLLINEAR_CEILING
            else:
                v.type = COLLINEAR_FLOOR
            continue

        if v.x < v.next.x and v.x < v.prev.x:
            if angle(v) < np.pi:
                v.type = OPEN
            else:
                v.type = SPLIT

        if v.x > v.next.x and v.x > v.prev.x:
            if angle(v) < np.pi:
                v.type = CLOSE
            else:
                v.type = MERGE

        if v.prev.x < v.x and v.next.x > v.x:
            if angle(v) < np.pi:
                v.type = FLOOR_CONVEX
            else:
                v.type = FLOOR_CONCAVE

        if v.prev.x > v.x and v.next.x < v.x:
            if angle(v) < np.pi:
                v.type = CEIL_CONVEX
            else:
                v.type = CEIL_CONCAVE

    # Visiting events in ascending order
    cells = []
    active_cells = []
    break_here = False

    #for v in combined_vertices_sorted:
        #print(f'{v} {v.type}')

    for v in combined_vertices_sorted:
        if break_here:
            break

        if v.type == OPEN:
            print(f'OPEN at {v}')
            # Opening new cell containing ceiling list and floor list
            cell = ([], [v])
            cells.append(cell)
            active_cells.append(True)
        elif v.type == CEIL_CONVEX:
            print(f'CEIL_CONVEX at {v}')
            i, cell = find_cell(v, cells, active_cells, True)
            cell[0].append(v)
        elif v.type == FLOOR_CONVEX:
            print(f'FLOOR_CONVEX at {v}')
            i, cell = find_cell(v, cells, active_cells, False)
            cell[1].append(v)
        elif v.type == SPLIT:
            print(f'SPLIT at {v}')
            # Shooting rays upwards and downwards from v
            ray_start = v.get_array().flatten()  # Ray starting point P0
            ray_dir = np.array([[0], [1]]).flatten()  # Ray direction vector
            edge_up = None
            edge_down = None
            v_up = None
            v_down = None

            intersections_up = []
            intersections_up_dist = []
            intersections_up_edges = []

            intersections_down = []
            intersections_down_dist = []
            intersections_down_edges = []

            for edge in combined_edges:
                seg_A = edge.v_from.get_array().flatten()  # Segment point A
                seg_B = edge.v_to.get_array().flatten()

                intersection_up = compute_intersection(ray_start, ray_dir, seg_A, seg_B)
                intersection_down = compute_intersection(ray_start, -ray_dir, seg_B, seg_A)

                if intersection_up is not None:
                    intersections_up.append(intersection_up)
                    dist = np.linalg.norm(intersection_up - v.get_array().flatten())
                    intersections_up_edges.append(edge)
                    intersections_up_dist.append(dist)

                if intersection_down is not None:
                    intersections_down.append(intersection_down)
                    dist = np.linalg.norm(intersection_down - v.get_array().flatten())
                    intersections_down_dist.append(dist)
                    intersections_down_edges.append(edge)

            intersection_up_index = np.argmin(intersections_up_dist)
            intersection_up = intersections_up[intersection_up_index]
            intersection_up_edge = intersections_up_edges[intersection_up_index]

            v_up = Vertex(-1, intersection_up[0], intersection_up[1])
            v_up.edge_from_v_is_hard = intersection_up_edge.v_from.edge_from_v_is_hard
            edge_up = intersection_up_edge

            intersection_down_index = np.argmin(intersections_down_dist)
            intersection_down = intersections_down[intersection_down_index]
            intersection_down_edge = intersections_down_edges[intersection_down_index]

            v_down = Vertex(-1, intersection_down[0], intersection_down[1])
            v_down.edge_from_v_is_hard = intersection_down_edge.v_from.edge_from_v_is_hard
            edge_down = intersection_down_edge

            # Handling the upwards intersection
            combined_edges.remove(edge_up)
            edge_up.v_from.next = v_up
            edge_up.v_to.prev = v_up
            v_up.prev = edge_up.v_from
            v_up.next = edge_up.v_to
            combined_edges.append(Edge(edge_up.v_from, v_up))
            combined_edges.append(Edge(v_up, edge_up.v_to))

            i, cell = find_cell(v, cells, active_cells, True, edge_up.v_to)
            cell[0].append(v_up)

            # Handling the downwards intersection
            combined_edges.remove(edge_down)
            edge_down.v_from.next = v_down
            edge_down.v_to.prev = v_down
            v_down.prev = edge_down.v_from
            v_down.next = edge_down.v_to
            combined_edges.append(Edge(edge_down.v_from, v_down))
            combined_edges.append(Edge(v_down, edge_down.v_to))

            #TODO cell = find_cell()
            i2, cell = find_cell(v, cells, active_cells, False, edge_down.v_from)
            cell[1].append(v_down)

            if i == i2:
                active_cells[i] = False
            else:
                active_cells[i] = False
                active_cells[i2] = False

            # Fixing the previous cell

            top_cell = ([v_up],[v])
            bottom_cell = ([v],[v_down])
            cells.append(top_cell)
            cells.append(bottom_cell)
            active_cells.append(True)
            active_cells.append(True)
        elif v.type == CEIL_CONCAVE:
            print(f'CEIL_CONCAVE at {v}')
            # Shooting ray downwards from v
            ray_start = v.get_array().flatten()  # Ray starting point P0
            ray_dir = np.array([[0], [1]]).flatten()  # Ray direction vector

            intersections = []
            intersection_distances = []
            intersection_edges = []

            for edge in combined_edges:
                seg_A = edge.v_from.get_array().flatten() # Segment point A
                seg_B = edge.v_to.get_array().flatten()

                intersection = compute_intersection(ray_start, -ray_dir, seg_A, seg_B)

                if intersection is not None:
                    intersections.append(intersection)
                    dist = np.linalg.norm(intersection - v.get_array().flatten())
                    intersection_distances.append(dist)
                    intersection_edges.append(edge)

            intersection_index = np.argmin(intersection_distances)
            intersection = intersections[intersection_index]
            intersection_edge = intersection_edges[intersection_index]

            v_down = Vertex(-1, intersection[0], intersection[1])
            v_down.edge_from_v_is_hard = intersection_edge.v_from.edge_from_v_is_hard

            i, cell = find_cell(v, cells, active_cells, False, intersection_edge.v_from)
            cell[0].append(v)
            cell[1].append(v_down)
            active_cells[i] = False

            new_cell = ([v], [v_down])
            cells.append(new_cell)
            active_cells.append(True)

            #print(f'{cell=}')
            #print(f'ray from {v} intersects {edge}')
            combined_edges.remove(intersection_edge)
            intersection_edge.v_from.next = v_down
            intersection_edge.v_to.prev = v_down
            v_down.prev = intersection_edge.v_from
            v_down.next = intersection_edge.v_to
            combined_edges.append(Edge(intersection_edge.v_from, v_down))
            combined_edges.append(Edge(v_down, intersection_edge.v_to))

            # TODO consider also adding v here"""
            # i, cell = find_cell(edge.v_from, cells)
            # cell[1].append(v_down)
        elif v.type == FLOOR_CONCAVE:
            print(f'FLOOR_CONCAVE at {v}')
            # Shooting ray upwards from v
            ray_start = v.get_array().flatten()  # Ray starting point P0
            ray_dir = np.array([[0], [1]]).flatten()  # Ray direction vector
            intersections = []
            intersection_distances = []
            intersection_edges = []

            for edge in combined_edges:
                seg_A = edge.v_from.get_array().flatten()  # Segment point A
                seg_B = edge.v_to.get_array().flatten()

                intersection = compute_intersection(ray_start, ray_dir, seg_A, seg_B)

                if intersection is not None:
                    intersections.append(intersection)
                    dist = np.linalg.norm(intersection - v.get_array().flatten())
                    intersection_distances.append(dist)
                    intersection_edges.append(edge)

            intersection_index = np.argmin(intersection_distances)
            intersection = intersections[intersection_index]
            intersection_edge = intersection_edges[intersection_index]

            v_up = Vertex(-1, intersection[0], intersection[1])

            v_up.edge_from_v_is_hard = intersection_edge.v_from.edge_from_v_is_hard

            i, cell = find_cell(v, cells, active_cells, True, intersection_edge.v_to)
            cell[0].append(v_up)
            cell[1].append(v)
            active_cells[i] = False

            new_cell = ([v_up], [v])
            cells.append(new_cell)
            active_cells.append(True)

            # print(f'{cell=}')

            combined_edges.remove(intersection_edge)
            intersection_edge.v_from.next = v_up
            intersection_edge.v_to.prev = v_up
            v_up.next = intersection_edge.v_to
            v_up.prev = intersection_edge.v_from
            #TODO MISSING v_up prev and next
            combined_edges.append(Edge(intersection_edge.v_from, v_up))
            combined_edges.append(Edge(v_up, intersection_edge.v_to))
        elif v.type == MERGE:
            print(f'MERGE at {v}')
            # Shooting rays upwards and downwards from v
            ray_start = v.get_array().flatten()  # Ray starting point P0
            ray_dir = np.array([[0], [1]]).flatten()  # Ray direction vector

            edge_up = None
            edge_down = None
            v_up = None
            v_down = None

            intersections_up = []
            intersections_up_dist = []
            intersections_up_edges = []

            intersections_down = []
            intersections_down_dist = []
            intersections_down_edges = []

            for edge in combined_edges:
                seg_A = edge.v_from.get_array().flatten()  # Segment point A
                seg_B = edge.v_to.get_array().flatten()

                intersection_up = compute_intersection(ray_start, ray_dir, seg_A, seg_B)
                intersection_down = compute_intersection(ray_start, -ray_dir, seg_B, seg_A)

                if intersection_up is not None:
                    """v_up = Vertex(-1, intersection_up[0], intersection_up[1])
                    v_up.edge_from_v_is_hard = edge.v_from.edge_from_v_is_hard
                    edge_up = edge"""
                    #print(f"intersects upwards with {edge} at {v_up}")

                    intersections_up.append(intersection_up)
                    dist = np.linalg.norm(intersection_up - v.get_array().flatten())
                    intersections_up_edges.append(edge)
                    intersections_up_dist.append(dist)

                if intersection_down is not None:
                    """v_down = Vertex(-1, intersection_down[0], intersection_down[1])
                    v_down.edge_from_v_is_hard = edge.v_from.edge_from_v_is_hard
                    edge_down = edge"""

                    intersections_down.append(intersection_down)
                    dist = np.linalg.norm(intersection_down - v.get_array().flatten())
                    intersections_down_dist.append(dist)
                    intersections_down_edges.append(edge)
                    #print(f'intersects downwards with {edge} at {v_down}')

            intersection_up_index = np.argmin(intersections_up_dist)
            intersection_up = intersections_up[intersection_up_index]
            intersection_up_edge = intersections_up_edges[intersection_up_index]

            v_up = Vertex(-1, intersection_up[0], intersection_up[1])
            v_up.edge_from_v_is_hard = intersection_up_edge.v_from.edge_from_v_is_hard
            edge_up = intersection_up_edge

            intersection_down_index = np.argmin(intersections_down_dist)
            intersection_down = intersections_down[intersection_down_index]
            intersection_down_edge = intersections_down_edges[intersection_down_index]

            v_down = Vertex(-1, intersection_down[0], intersection_down[1])
            v_down.edge_from_v_is_hard = intersection_down_edge.v_from.edge_from_v_is_hard
            edge_down = intersection_down_edge

            #print(f'{edge_up=}')
            #print(f'{edge_down=}')
            # Handling upwards intersection
            combined_edges.remove(edge_up)
            edge_up.v_from.next = v_up
            edge_up.v_to.prev = v_up
            v_up.prev = edge_up.v_from
            v_up.next = edge_up.v_to
            combined_edges.append(Edge(edge_up.v_from, v_up))
            combined_edges.append(Edge(v_up, edge_up.v_to))
            # TODO HERE

            i, cell = find_cell(v, cells, active_cells, True, edge_up.v_to)
            #print(f'{v_up=} belongs to {cell=}')
            cell[0].append(v_up)
            cell[1].append(v)
            active_cells[i] = False

            # Handling downwards intersection
            combined_edges.remove(edge_down)
            edge_down.v_from.next = v_down
            edge_down.v_to.prev = v_down
            v_down.prev = edge_down.v_from
            v_down.next = edge_down.v_to
            combined_edges.append(Edge(edge_down.v_from, v_down))
            combined_edges.append(Edge(v_down, edge_down.v_to))

            i, cell = find_cell(v, cells, active_cells, False, edge_down.v_from)
            cell[0].append(v)
            cell[1].append(v_down)
            active_cells[i] = False

            new_cell = ([v_up], [v_down])
            cells.append(new_cell)
            active_cells.append(True)

        elif v.type == CLOSE:
            print(f'CLOSE at {v}')
            i, cell = find_cell(v, cells, active_cells, False)
            cell[1].append(v)
            active_cells[i] = False
        elif v.type == COLLINEAR_CEILING:
            print(f'COLLINEAR_CEILING at {v}')
            i, cell = find_cell(v, cells, active_cells, True)
            cell[0].append(v)
        elif v.type == COLLINEAR_FLOOR:
            print(f'COLLINEAR_FLOOR at {v}')
            i, cell = find_cell(v, cells, active_cells, False)
            cell[1].append(v)

        print(f'\t {active_cells}')
        for c in cells:
            print(f'\t {c}')
    sub_polygons = []

    for cell in cells:
        vertices = []

        # Appending the floor vertices of the cell
        for i in range(0, len(cell[1])):
            v = cell[1][i]
            new_v = Vertex(len(vertices), v.x, v.y)
            new_v.edge_from_v_is_hard = v.edge_from_v_is_hard
            vertices.append(new_v)

        # Appending the ceiling vertices of the cell in reverse order
        for i in range(len(cell[0]) - 1, -1, -1):
            v = cell[0][i]
            new_v = Vertex(len(vertices), v.x, v.y)
            new_v.edge_from_v_is_hard = v.edge_from_v_is_hard
            vertices.append(new_v)

        for i in range(len(vertices)):
            vertices[i].edge_from_v_is_hard = vertices[i].edge_from_v_is_hard and not abs(vertices[i].x - vertices[(i + 1) % len(vertices)].x) < epsilon

        P = Polygon(vertices)
        sub_polygons.append(P)

    return sub_polygons

def find_bounding_polygons(sub_polygons, obstacle):
    """ Finds the sub-polygons where that an obstacle intersects """
    # Initialize an R-tree index
    idx = index.Index()

    # Insert bounding boxes for each sub-polygon into the R-tree
    for i, p in enumerate(sub_polygons):
        idx.insert(i, p.bbox)

    # Query the R-tree to find the sub-polygons whose bounding box intersects the obstacles bounding box
    intersecting_ids = list(idx.intersection(obstacle.bbox))
    sub_polygons_filtered = []
    sub_polygons_filtered_indices = []

    # Iterate through the sub-polygons that shares the bounding box with the obstacle
    for i in intersecting_ids:
        p = sub_polygons[i]
        skip = False

        # Check if a vertex is within the polygon (Point-In-Polygon test)
        for o in obstacle.vertices:
            cross_products = np.empty(len(p.vertices))

            for j, v in enumerate(p.vertices):
                vec1 = v.next.get_array() - v.get_array()
                vec2 = o.get_array() - v.get_array()

                cross_products[j] = cross(vec1, vec2)

            # Check if the obstacle point is contained within the convex polygon
            if np.all(cross_products > 0):
                sub_polygons_filtered_indices.append(i)
                sub_polygons_filtered.append(p)
                skip = True
                break

        # Check for intersections between each sub-polygon and the obstacle
        for e in p.edges:
            if skip:
                break

            for e2 in obstacle.edges:
                intersection = compute_intersection_edges(e, e2)

                if intersection is not None:
                    sub_polygons_filtered_indices.append(i)
                    sub_polygons_filtered.append(p)
                    skip = True
                    break



    return sub_polygons_filtered_indices, sub_polygons_filtered

def compute_obstacle_region(sub_polygons_filtered, obstacle):
    """ Compute the minimum region based on x-coordinate that contains the obstacle """
    split_index = np.argmin(obstacle.vertices_matrix()[0, :])
    split_vertex = obstacle.vertices[split_index]

    # Determine which sub-polygon contains the SPLIT vertex
    for i, p in enumerate(sub_polygons_filtered):
        cross_products = np.empty(len(p.edges))

        for j, e in enumerate(p.edges):
            vec1 = e.v_to.get_array() - e.v_from.get_array()
            vec2 = split_vertex.get_array() - e.v_from.get_array()

            cross_products[j] = cross(vec1, vec2)

        if np.all(cross_products > 0) or np.all(cross_products < 0):
            open_polygon = p
            break

    # Shoot rays upwards and downwards from the SPLIT vertex
    ray_start = split_vertex.get_array().flatten()  # Ray starting point P0
    ray_dir = np.array([[0], [1]]).flatten()  # Ray direction vector

    for edge in open_polygon.edges:
        # Compute the intersection in both directions between the rays and edges if the polygon
        seg_A = edge.v_from.get_array().flatten() # Segment point A
        seg_B = edge.v_to.get_array().flatten() # Segment point B

        intersection_up = compute_intersection(ray_start, ray_dir, seg_A, seg_B)
        intersection_down = compute_intersection(ray_start, -ray_dir, seg_B, seg_A)

        if intersection_up is not None:
            v_up = Vertex(-1, intersection_up[0], intersection_up[1])

        if intersection_down is not None:
            v_down = Vertex(-1, intersection_down[0], intersection_down[1])

    # Find the OPEN vertex in the OPEN polygon
    open_index = np.argmin(open_polygon.vertices_matrix()[0, :])
    open_vertex = open_polygon.vertices[open_index]

    # Start a new cell
    left_cell = ([], [open_vertex])

    # Appending vertices to the floor list where the x-coordinate is less than the SPLIT vertex
    v_next = open_vertex.next
    while v_next.x < split_vertex.x:
        left_cell[1].append(v_next)
        v_next = v_next.next
    left_cell[1].append(v_down)

    # Appending vertices to the ceiling and ceiling list where the x-coordinate is less than the SPLIT vertex
    v_prev = open_vertex.prev
    while v_prev.x < split_vertex.x:
        left_cell[0].append(v_prev)
        v_prev = v_prev.prev
    left_cell[0].append(v_up)

    sub_polygons = []
    for cell in cells:
        vertices = []

        # Appending the floor vertices of the cell
        for i in range(0, len(cell[1])):
            vertices.append(Vertex(len(vertices), cell[1][i].x, cell[1][i].y))

        # Appending the ceiling vertices of the cell
        for i in range(len(cell[0]) - 1, -1, -1):
            vertices.append(Vertex(len(vertices), cell[0][i].x, cell[0][i].y))
        P = Polygon(vertices)
        sub_polygons.append(P)
    print(left_cell)

def find_shared_edge_all(P1, P2):
    """ Determine if two polygons share an edge either by complete or partial adjacency"""
    for e in P1.edges:
        for e2 in P2.edges:
            # Check for complete adjacency between the polygons
            if (points_are_equal(e.v_from.get_array(), e2.v_from.get_array()) and points_are_equal(e.v_to.get_array(),
                                                                                                   e2.v_to.get_array())) or \
                    (points_are_equal(e.v_from.get_array(), e2.v_to.get_array()) and points_are_equal(
                        e.v_to.get_array(), e2.v_from.get_array())):
                # print(f'complete overlap between {i} and {j}')
                return e, e2

            # Define the vector for each edge
            vec1 = e.v_to.get_array() - e.v_from.get_array() + 1e-6
            vec2 = e2.v_to.get_array() - e2.v_from.get_array() + 1e-6

            dot_product = dot(vec1, vec2)
            neg_mag = -np.linalg.norm(vec1) * np.linalg.norm(vec2)

            # Check if the vectors are collinear
            if dot_product - epsilon_xl <= neg_mag <= dot_product + epsilon_xl:
                # Parametrize the line from vec1: l(t) = P + t v

                # Compute the intersection with the y-axis for each line
                t1 = - e.v_from.x / vec1[0]
                e_intersects_y = e.v_from.y + t1 * vec1[1]

                t2 = - e2.v_from.x / vec2[0]
                e2_intersects_y = e2.v_from.y + t2 * vec2[1]

                # Check if the two lines intersects y in the same point
                if points_are_equal(e_intersects_y, e2_intersects_y, epsilon_xl):
                    # Check for partial adjacency between the polygons (if the projected intervals overlap)
                    t1 = 0
                    t2 = dot(e.v_to.get_array() - e.v_from.get_array(), vec1) / dot(vec1, vec1)
                    s1 = dot(e2.v_from.get_array() - e.v_from.get_array(), vec1) / dot(vec1, vec1)
                    s2 = dot(e2.v_to.get_array() - e.v_from.get_array(), vec1) / dot(vec1, vec1)

                    if max(t1, t2) > min(s1, s2) and max(s1, s2) > min(t1, t2):
                        return e, e2
    return None

def merge_filtered_sub_polygons(sub_polygons_filtered, sub_polygons, mask):
    """ Merging polygons with shared edges without considering angles """
    merged = True

    while merged:
        merged = False

        for i, P_i in enumerate(sub_polygons_filtered):
            for j, P_j in enumerate(sub_polygons_filtered):
                if j <= i:
                    continue

                shared_edges = find_shared_edge_all(P_i, P_j)

                if shared_edges != None:
                    e, e2 = shared_edges
                    #print(f'SHARED EDGE BETWEEN P{i} {e} and P{j} {e2}')

                    combined_polygon_vertices = []

                    if e.edge_length < e2.edge_length:
                        inner_edge = e
                        outer_edge = e2
                    else:
                        inner_edge = e2
                        outer_edge = e

                    v = Vertex(-1, inner_edge.v_from.x, inner_edge.v_from.y)
                    v_next = outer_edge.v_to.next

                    v.edge_from_v_is_hard = outer_edge.v_to.edge_from_v_is_hard
                    combined_polygon_vertices.append(v)

                    while v_next != outer_edge.v_from:
                        v = Vertex(-1, v_next.x, v_next.y)
                        v.edge_from_v_is_hard = v_next.edge_from_v_is_hard
                        combined_polygon_vertices.append(v)
                        v_next = v_next.next

                    #combined_polygon_vertices.append(Vertex(-1, inner_edge.v_to.x, inner_edge.v_to.y))

                    v_next = inner_edge.v_to

                    while v_next != inner_edge.v_from:
                        v = Vertex(-1, v_next.x, v_next.y)
                        v.edge_from_v_is_hard = v_next.edge_from_v_is_hard
                        combined_polygon_vertices.append(v)
                        v_next = v_next.next

                    #print(f"\t MERGIN P{i} and P{j}")
                    P = Polygon(combined_polygon_vertices)
                    #plot_obstacles([P], [], False)

                    #if not is_well_formed(P):
                    #    break

                    sub_polygons_filtered[i] = P
                    sub_polygons_filtered.pop(j)
                    merged = True
                    break
                if merged:
                    break

    # Remove collinear vertices for the sub-polygon
    #p = sub_polygons_filtered[0]
    p = remove_equal_points(sub_polygons_filtered[0])
    #p = remove_collinear_vertices(p)
    #p.compute_bounding_box()
    #sub_polygons_filtered[i] = p

    #for i, v in enumerate(p.vertices):
    #    v.index = i

    sub_polygons_extract = []

    # Append all the other sub-polygons not affected by the obstacle
    for i, poly in enumerate(sub_polygons):
        if i not in mask:
            sub_polygons_extract.append(i)

    return sub_polygons_extract, p

def decompose_around_obstacle(filtered_sub_polygons, obstacle):
    """ Computes the convex decomposition around the obstacle """
    merge_filtered_sub_polygons(filtered_sub_polygons)

def plot_obstacles(sub_polygons, obstacles, include_points=True):
    fig, ax = plt.subplots(1, 1)
    count = 0
    for i, p in enumerate(sub_polygons):
        for e in p.edges:
            if e.is_hard_edge:
                ax.plot([e.v_from.x, e.v_to.x], [e.v_from.y, e.v_to.y], f'r-')
            else:
                ax.plot([e.v_from.x, e.v_to.x], [e.v_from.y, e.v_to.y], f'k-')

        if include_points:
            for v in p.vertices:
                plt.text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex
                if v.edge_from_v_is_hard:
                    plt.scatter(v.x, v.y, color='red')
                else:
                    plt.scatter(v.x, v.y, color='black')
                count += 1
        c_x, c_y = get_center_of_polygon(p)
        ax.text(c_x - 0.1, c_y, f'P{i}', color='r', fontsize=7)

    for o in obstacles:
        for e in o.edges:
            if e.is_hard_edge:
                ax.plot([e.v_from.x, e.v_to.x], [e.v_from.y, e.v_to.y], f'r-')
            else:
                ax.plot([e.v_from.x, e.v_to.x], [e.v_from.y, e.v_to.y], f'k-')


        #x_coords, y_coords = o.get_coords()

        #ax.plot(x_coords, y_coords, f'k-', marker='o' if include_points else None)
        #ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], f'k-')

        if include_points:
            for v in o.vertices:
                plt.text(v.x, v.y, f'{count + v.index}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex

                if v.edge_from_v_is_hard:
                    plt.scatter(v.x, v.y, color='red')
                else:
                    plt.scatter(v.x, v.y, color='black')

    ax.set_aspect('equal')
    plt.show()
    return fig

def find_cell(v, cells, active_cells, direction_up, v2=None):
    for i, cell in enumerate(cells):
        if not active_cells[i]:
            continue
        if v.type == SPLIT and (v2 in cell[0] or v2 in cell[1]):
            return i, cell
        if v.type == CEIL_CONVEX and (v.next in cell[0] or v.next in cell[1]):
            return i, cell
        if v.type == FLOOR_CONVEX and (v.prev in cell[0] or v.prev in cell[1]):
            return i, cell
        if v.type == CEIL_CONCAVE and (v2 in cell[0] or v2 in cell[1]):
            return i, cell
        if v.type == FLOOR_CONCAVE and (v.prev in cell[0] or v.prev in cell[1]) and (v2 in cell[1] or v2 in cell[0]):
            return i, cell
        if v.type == MERGE:
            if direction_up and v2 in cell[0]:
                return i, cell
            if not direction_up and v2 in cell[1]:
                return i, cell
        if v.type == COLLINEAR_CEILING and (v.next in cell[0] or v.next in cell[1]):
            return i, cell
        if v.type == COLLINEAR_FLOOR and (v.prev in cell[0] or v.prev in cell[1]):
            return i, cell
        if v.type == CLOSE and v.prev in cell[1] and v.next in cell[0]:
            return i, cell

def combined_algorithms(region, obstacles):
    # Decompose the region without considering obstacles
    sub_polygons = generate_new_data(copy.deepcopy(region))

    # Divide the sub-polygons into clusters that are affected by the obstacles
    sub_polygons_filtered_masks = []
    sub_polygons_filtered = []
    obstacles_affected = []

    for o in obstacles:
        filtered_mask, filtered = find_bounding_polygons(sub_polygons, o)
        common_found = False

        # plot_obstacles(filtered, obstacles, False)

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
        sub_polygons_extract, merged_sub_polygon = merge_filtered_sub_polygons(copy.deepcopy(filtered),
                                                                               copy.deepcopy(sub_polygons),
                                                                               sub_polygons_filtered_masks[i])

        merged_sub_polygon_decomposed = decompose_sweep_line(merged_sub_polygon, obstacles_affected[i])
        decomposed_polygons += merged_sub_polygon_decomposed

        # plot_obstacles(merged_sub_polygon_decomposed, obstacles, False)

        for p in sub_polygons_extract:
            if p not in extracted_sub_polygons_mask and p not in dont_include_mask:
                extracted_sub_polygons_mask.append(p)
                extracted_sub_polygons.append(sub_polygons[p])

        #plot_obstacles(extracted_sub_polygons + [merged_sub_polygon], obstacles, False)

def angle(v):
    """ Compute interior or exterior (outward-facing angles)  """
    vec1 = v.prev.get_array() - v.get_array()
    vec2 = v.next.get_array() - v.get_array()

    angle1 = np.arccos(np.round(dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)), 3))
    angle2 = 2 * np.pi - angle1

    # If v is part of the obstacle the exterior angle should be computed
    if v.is_obstacle:
        # If the cross product is positive the angle is ccw (positive rotation), and the ext angle < pi
        if cross(vec1, vec2) < 0:
            return angle1
        # If the cross product is negative the angle is cw (negative rotation), and the ext angle > pi
        else:
            return angle2
    else:
        # If the cross product is positive the angle is ccw (positive rotation), and the ext angle < pi
        if cross(vec1, vec2) < 0:
            return angle1
        # If the cross product is negative the angle is cw (negative rotation), and the ext angle > pi
        else:
            return angle2

        #return np.arccos(dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)))