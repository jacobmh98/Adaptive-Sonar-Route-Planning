from copy import copy

import numpy as np
from matplotlib import pyplot as plt
from networkx.classes import edges

from decomposition import dot, cross
from Polygon import Edge, VertexType as VT, Vertex
from global_variables import *


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

def asd(region, obstacle):
    # Combining the vertices and edges of the polygon and obstacle
    combined_vertices = region.vertices + obstacle.vertices
    combined_edges = region.edges + obstacle.edges

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
    for v in combined_vertices:
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
    for v in combined_vertices_sorted:
        if v.type == OPEN:
            # Opening new cell containing ceiling list and floor list
            cell = ([], [v])

            cells.append(cell)
            active_cells.append(True)
            print('OPEN')
        elif v.type == CEIL_CONVEX:
            print('CEIL_CONVEX')
            i, cell = find_cell(v, cells, active_cells)
            cell[0].append(v)
        elif v.type == FLOOR_CONVEX:
            print('FLOOR_CONVEX')
            i, cell = find_cell(v, cells, active_cells)
            cell[1].append(v)
        elif v.type == SPLIT:
            print('SPLIT')
            # Shooting rays upwards and downwards from v
            ray_start = v.get_array().flatten()  # Ray starting point P0
            ray_dir = np.array([[0], [1]]).flatten()  # Ray direction vector

            for edge in combined_edges:
                seg_A = edge.v_from.get_array().flatten()  # Segment point A
                seg_B = edge.v_to.get_array().flatten()

                intersection_up = compute_intersection(ray_start, ray_dir, seg_A, seg_B)
                intersection_down = compute_intersection(ray_start, -ray_dir, seg_B, seg_A)

                if intersection_up is not None:
                    v_up = Vertex(-1, intersection_up[0], intersection_up[1])
                    combined_edges.remove(edge)
                    edge.v_from.next = v_up
                    edge.v_to.prev = v_up
                    v_up.prev = edge.v_from
                    v_up.next = edge.v_to
                    combined_edges.append(Edge(edge.v_from, v_up))
                    combined_edges.append(Edge(v_up, edge.v_to))

                    cells[-1][0].append(v_up)

                if intersection_down is not None:
                    v_down = Vertex(-1, intersection_down[0], intersection_down[1])
                    combined_edges.remove(edge)
                    edge.v_from.next = v_down
                    edge.v_to.prev = v_down
                    v_down.prev = edge.v_from
                    v_down.next = edge.v_to
                    combined_edges.append(Edge(edge.v_from, v_down))
                    combined_edges.append(Edge(v_down, edge.v_to))

                    # TODO consider also adding v here
                    cells[-1][1].append(v_down)
            active_cells[i] = False
            top_cell = ([v_up],[v])
            bottom_cell = ([v],[v_down])
            cells.append(top_cell)
            cells.append(bottom_cell)
            active_cells.append(True)
            active_cells.append(True)
        elif v.type == CEIL_CONCAVE:
            print('CEIL_CONCAVE')
            # Shooting ray downwards from v
            ray_start = v.get_array().flatten()  # Ray starting point P0
            ray_dir = np.array([[0], [1]]).flatten()  # Ray direction vector

            for edge in combined_edges:
                seg_A = edge.v_from.get_array().flatten() # Segment point A
                seg_B = edge.v_to.get_array().flatten()

                intersection = compute_intersection(ray_start, -ray_dir, seg_A, seg_B)

                if intersection is not None:
                    v_down = Vertex(-1, intersection[0], intersection[1])

                    i, cell = find_cell(v, cells, active_cells, edge.v_from)
                    cell[0].append(v)
                    cell[1].append(v_down)
                    active_cells[i] = False

                    new_cell = ([v], [v_down])
                    cells.append(new_cell)
                    active_cells.append(True)

                    #print(f'{cell=}')
                    #print(f'ray from {v} intersects {edge}')
                    combined_edges.remove(edge)
                    edge.v_from.next = v_down
                    edge.v_to.prev = v_down
                    v_down.prev = edge.v_from
                    v_down.next = edge.v_to
                    combined_edges.append(Edge(edge.v_from, v_down))
                    combined_edges.append(Edge(v_down, edge.v_to))
                    
                    # TODO consider also adding v here"""
                    # i, cell = find_cell(edge.v_from, cells)
                    # cell[1].append(v_down)
        elif v.type == FLOOR_CONCAVE:
            print('FLOOR_CONCAVE')
            # Shooting ray upwards from v
            ray_start = v.get_array().flatten()  # Ray starting point P0
            ray_dir = np.array([[0], [1]]).flatten()  # Ray direction vector

            for edge in combined_edges:
                seg_A = edge.v_from.get_array().flatten()  # Segment point A
                seg_B = edge.v_to.get_array().flatten()

                intersection = compute_intersection(ray_start, ray_dir, seg_A, seg_B)

                if intersection is not None:
                    v_up = Vertex(-1, intersection[0], intersection[1])

                    i, cell = find_cell(v, cells, active_cells, edge.v_to)
                    cell[0].append(v_up)
                    cell[1].append(v)
                    active_cells[i] = False

                    new_cell = ([v_up], [v])
                    cells.append(new_cell)
                    active_cells.append(True)

                    # print(f'{cell=}')

                    combined_edges.remove(edge)
                    edge.v_from.next = v_up
                    edge.v_to.prev = v_up
                    v_up.next = edge.v_to
                    v_up.prev = edge.v_from
                    #TODO MISSING v_up prev and next
                    combined_edges.append(Edge(edge.v_from, v_up))
                    combined_edges.append(Edge(v_up, edge.v_to))
        elif v.type == MERGE:
            print('MERGE')
            # Shooting rays upwards and downwards from v
            ray_start = v.get_array().flatten()  # Ray starting point P0
            ray_dir = np.array([[0], [1]]).flatten()  # Ray direction vector

            for edge in combined_edges:
                seg_A = edge.v_from.get_array().flatten()  # Segment point A
                seg_B = edge.v_to.get_array().flatten() # Segment point B

                intersection_up = compute_intersection(ray_start, ray_dir, seg_A, seg_B)
                intersection_down = compute_intersection(ray_start, -ray_dir, seg_B, seg_A)

                if intersection_up is not None:
                    v_up = Vertex(-1, intersection_up[0], intersection_up[1])
                    print(f'{v} intersects {edge} at {intersection_up}')
                    combined_edges.remove(edge)
                    edge.v_from.next = v_up
                    edge.v_to.prev = v_up
                    v_up.prev = edge.v_from
                    v_up.next = edge.v_to
                    combined_edges.append(Edge(edge.v_from, v_up))
                    combined_edges.append(Edge(v_up, edge.v_to))
                    i, cell = find_cell(v, cells, active_cells, edge.v_to)
                    cell[0].append(v_up)
                    cell[1].append(v)
                    active_cells[i] = False

                if intersection_down is not None:
                    v_down = Vertex(-1, intersection_down[0], intersection_down[1])
                    print(f'{v} intersects {edge} at {intersection_down}')
                    combined_edges.remove(edge)
                    edge.v_from.next = v_down
                    edge.v_to.prev = v_down
                    v_down.prev = edge.v_from
                    v_down.next = edge.v_to
                    combined_edges.append(Edge(edge.v_from, v_down))
                    combined_edges.append(Edge(v_down, edge.v_to))
                    i, cell = find_cell(v, cells, active_cells, edge.v_from)
                    cell[0].append(v)
                    cell[1].append(v_down)
                    active_cells[i] = False

            new_cell = ([v_up], [v_down])
            cells.append(new_cell)
            active_cells.append(True)
        elif v.type == CLOSE:
            print('CLOSE')
            cells[-1][1].append(v)
            active_cells[-1] = False

    print()
    print(active_cells)
    for cell in cells:
        print(cell)

def plot_obstacles(sub_polygons, obstacles):
    fig, ax = plt.subplots(1, 1)

    for p in sub_polygons:
        x_coords, y_coords = p.get_coords()

        ax.plot(x_coords, y_coords, f'k-', marker='o')
        ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], f'k-')

        for v in p.vertices:
            plt.text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex

    for o in obstacles:
        x_coords, y_coords = o.get_coords()

        ax.plot(x_coords, y_coords, f'k-', marker='o')
        ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], f'k-')

        for v in o.vertices:
            plt.text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex

    ax.set_aspect('equal')
    plt.show()

def find_cell(v, cells, active_cells, v2=None):
    for i, cell in enumerate(cells):
        if not active_cells[i]:
            continue

        if v.type == CEIL_CONVEX and (v.next in cell[0] or v.next in cell[1]):
            return i, cell
        if v.type == FLOOR_CONVEX and (v.prev in cell[0] or v.prev in cell[1]):
            return i, cell
        if v.type == CEIL_CONCAVE and (v2 in cell[0] or v2 in cell[1]):
            return i, cell
        if v.type == FLOOR_CONCAVE and (v2 in cell[1] or v2 in cell[0]):
            return i, cell
        if v.type == MERGE and (v2 in cell[0] or v2 in cell[1]):
            return i, cell

def angle(v):
    """ Compute interior or exterior (outward-facing angles)  """
    vec1 = v.prev.get_array() - v.get_array()
    vec2 = v.next.get_array() - v.get_array()

    # If v is part of the obstacle the exterior angle should be computed
    if v.is_obstacle:
        angle1 = np.arccos(dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)))
        angle2 = 2 * np.pi - angle1

        # If the cross product is positive the angle is ccw (positive rotation), and the ext angle < pi
        if cross(vec1, vec2) < 0:
            return angle1
        # If the cross product is negative the angle is cw (negative rotation), and the ext angle > pi
        else:
            return angle2
    else:
        return np.arccos(dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)))