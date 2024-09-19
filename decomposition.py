from json.encoder import INFINITY

import numpy as np
import json

from Polygon import Vertex, Polygon
from functions import *

# Reading the test data
f = open('test_data/complex_polygon.json')
#f = open('test_data/intersection_in_vertex.json')
data = json.load(f)
vertices_data = data['area']['coordinates']

# Defining the initial polygon and the boúnding box
vertices = []
min_x = INFINITY
max_x = -INFINITY
min_y = INFINITY
max_y = -INFINITY

for i, v in enumerate(vertices_data):
    if v[0] < min_x:
        min_x = v[0]

    if v[0] > max_x:
        max_x = v[0]

    if v[1] < min_y:
        min_y= v[1]

    if v[1] > max_y:
        max_y = v[1]

    vertices.append(Vertex(i, v[0], v[1]))

polygon = Polygon(vertices)
#polygon.plot()

# Compute the concave vertices
polygon.concave_vertices = compute_concave_vertices(polygon)
print(f'concave vertices = {polygon.concave_vertices}')

# Go through each concave vertex
for i, cv in enumerate(polygon.concave_vertices):
    if cv.index != 1: #tmp
        continue

    print(f"checking for {cv.index=} with coord = ({cv.x}, {cv.y})")
    split_polygons = []

    # Check lines which passes the concave vertex i and parallels edge e
    for j, e in enumerate(polygon.edges):
        intersection_points = []
        intersection_edges = []
        intersection_directions = []

        # Define a vector from the vertices in edge e end its negation
        vec = e.v_to.get_array() - e.v_from.get_array()

        # Go through each edge in the polygon
        for e2 in polygon.edges:
            if e == e2:
                continue

            # Parameterize the line segment and vector
            vx = vec[0, 0]
            vy = vec[1, 0]

            x0 = cv.x
            y0 = cv.y

            x1 = e2.v_from.x
            y1 = e2.v_from.y

            x2 = e2.v_to.x
            y2 = e2.v_to.y

            s_denominator = (y1 * vx - y2 * vx - vy * x1 + vy * x2)

            if s_denominator == 0:
                s = 0
            else:
                s = - ((y0 * vx - y1 * vx - vy * x0 + vy * x1) / s_denominator)
            t = - ((s * x1 - s * x2 + x0 - x1) / vx)

            """
            print(f'{x0=}')
            print(f'{y0=}')

            print(f'{x1=}')
            print(f'{y1=}')

            print(f'{x2=}')
            print(f'{y2=}')

            print(f'{vx=}')
            print(f'{vy=}')
            """

            # Test if the line e intersects the edge e2
            if 0 <= s <= 1:
                # Compute the coordinates of the intersection point
                intersection_P = cv.get_array() + t * vec

                # Ignore the intersection point that happens in the concave vertex
                if points_are_equal(intersection_P, cv.get_array()):
                    continue

                # Ignore the intersection that happens in the adjacent vertices
                if t < 0 and e.v_from == e2.v_to:
                    continue
                if t > 0 and e.v_to == e2.v_from:
                    continue

                print(f'\t{e} intersects {e2} at ({intersection_P[0,0]}, {intersection_P[1,0]}) with direction t={t}')
                intersection_points.append(intersection_P)
                intersection_edges.append(e2)
                intersection_directions.append(t)

        print(f'\t\tNum intersections = {len(intersection_points)}')
        # Split the polygon into sub-polygons at a single intersection point
        if len(intersection_points) == 1:

            P1, P2 = split_polygon_single(intersection_edges[0], intersection_points[0], cv)
            split_polygons.append((P1, P2))
        # Split the polygon into sub-polygons given multiple intersection points
        else:
            # Split the intersections depending on direction from the concave vertex
            intersection_directions = np.array([intersection_directions]).flatten()
            indices_dir1 = np.where(intersection_directions >= 0)[0]
            indices_dir2 = np.where(intersection_directions < 0)[0]

            print(f'\t\t{intersection_directions=}')
            print(f'\t\t{indices_dir1=}')
            print(f'\t\t{indices_dir2=}')

            # Intersection happens in both directions from cv
            if len(indices_dir1) > 0 and len(indices_dir2) > 0:
                print(f'\t\tIntersection in both dirs')
                index_pos = compute_best_t_value(intersection_directions, True)
                index_neg = compute_best_t_value(intersection_directions, False)

                P1, P2 = split_polygon_mult_dirs(intersection_edges[index_pos], intersection_edges[index_neg], intersection_points[index_pos], intersection_points[index_neg])
                split_polygons.append((P1, P2))
            # If intersections only occur in positive direction
            elif len(indices_dir1) > 0 and len(indices_dir2) == 0:
                index_pos = compute_best_t_value(intersection_directions, True)

                P1, P2 = split_polygon_single(intersection_edges[index_pos], intersection_points[index_pos], cv)
                split_polygons.append((P1, P2))
            elif len(indices_dir2) > 0 and len(indices_dir1) == 0:
                index_neg = compute_best_t_value(intersection_directions, False)

                print(f'num intersections = {len(intersection_points)}')

                P1, P2 = split_polygon_single(intersection_edges[index_neg], intersection_points[index_neg], cv)
                split_polygons.append((P1, P2))
            # TODO fix if intersections happens at a vertex, should then continue onto next intersection etc.
    plot_results(split_polygons, cv.index)
    print(f'\tn_intersections = {len(split_polygons)}')
    break
    #plot_results(split_polygons, cv.index)