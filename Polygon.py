from enum import Enum

import numpy as np
import matplotlib.pyplot as plt

class VertexType(Enum):
    FLOOR_CONVEX = 1
    CEIL_CONVEX = 2
    FLOOR_CONCAVE = 3
    CEIL_CONCAVE = 4
    OPEN = 5
    SPLIT = 6
    MERGE = 7
    CLOSE = 8

class Vertex:
    def __init__(self, i, x, y, is_obstacle = False):
        self.index = i
        self.x = x
        self.y = y
        self.v = np.array([[x], [y]])
        self.prev = None
        self.next = None
        self.is_obstacle = is_obstacle
        self.edge_from_v_is_hard = False
        self.type = None

    def get_array(self):
        return np.array([[self.x], [self.y]])

    def __repr__(self):
        return f"Vertex({self.index}, {self.x}, {self.y}, obs={self.is_obstacle}, edge_from_v_is_hard={self.edge_from_v_is_hard})"

class Edge:
    def __init__(self, v1, v2, is_hard_edge = False):
        self.v_from = v1
        self.v_to = v2
        self.prev = None
        self.next = None
        self.is_hard_edge = is_hard_edge
        self.edge_length = np.linalg.norm(self.v_from.get_array() - self.v_to.get_array())
        if (v2.x - v1.x) == 0 or (v2.y - v1.y) == 0:
            self.slope = 0
        else:
            self.slope = (v2.y - v1.y) / (v2.x - v1.x)

    def __repr__(self):
        return f"Edge({self.v_from.index}, {self.v_to.index}, length={self.edge_length}, obs={self.v_from.is_obstacle}, is_hard={self.is_hard_edge})"

class Polygon:
    def __init__(self, v, is_obstacle=False):
        self.vertices = v
        self.number_vertices = len(self.vertices)
        self.concave_vertices = []
        self.edges = []
        self.i = None
        self.is_obstacle = is_obstacle
        self.bbox = None

        for i in range(len(self.vertices)):
            v1 = self.vertices[i]
            v2 = self.vertices[(i + 1) % self.number_vertices]
            e = Edge(v1, v2)
            self.edges.append(e)

            e.is_hard_edge = v1.edge_from_v_is_hard

            v1.prev = self.vertices[(i - 1) % self.number_vertices]
            v1.next = self.vertices[(i + 1) % self.number_vertices]

        for j in range(len(self.edges)):
            self.edges[j].prev = self.edges[(j - 1) % len(self.edges)]
            self.edges[j].next = self.edges[(j + 1) % len(self.edges)]

        #if hard_edges is not None:
        #for edge in hard_edges:
        #    self.edges[edge[0]].is_hard_edge = True
    def compute_bounding_box(self):
        vertices = self.vertices_matrix()
        # Bounding box: min_x, min_y, max_x, max_y
        self.bbox = (np.min(vertices[0, :]), np.min(vertices[1, :]), np.max(vertices[0, :]), np.max(vertices[1, :]))

    def get_coords(self):
        """ Get the vertices as a list of x-coordinates and y-coordinates"""
        x_coords = []
        y_coords = []

        for v in self.vertices:
            x_coords.append(v.x)
            y_coords.append(v.y)

        return x_coords, y_coords

    def set_index(self, i):
        self.i = i

    def get_index(self):
        return self.i

    def vertices_matrix(self):
        """ Function to get the vertices in the polygon in a 2xn list """
        x_coords, y_coords = self.get_coords()

        return np.array([x_coords, y_coords])

    def plot(self, color='k', title=''):
        x_coords = []
        y_coords = []
        fig, ax = plt.subplots(1, 1)

        for v in self.vertices:
            x_coords.append(v.x)
            y_coords.append(v.y)
            plt.text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex

        ax.plot(x_coords, y_coords, f'{color}-', marker='o')
        ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], f'{color}-')
        ax.set_title(f'{title}')
        ax.set_aspect('equal')
        plt.show()

    def get_mate(self, b):
        """ Find b's neighbour b_mate (counterclockwise neighbour)
        :param self: Polygon
        :param b: int, index of vertex b
        :return neighbour: int, index of b's neighbour vertex b_mate
        """
        n = len(self.vertices)

        if b == (n - 1):  # If b is the last vertex in the polygon
            return 0
        else:
            return b + 1

    def get_previous_vertex(self, b):
        """ Find b's previous neighbour
        :param self: Polygon
        :param b: int, index of vertex b
        :return neighbour: int, index of b's previous neighbour
        """
        n = len(self.vertices)

        if b == 0:  # If b is the first vertex
            return n - 1
        else:
            return b - 1

    def calculate_centroid(self):
        """Calculate the centroid of the polygon."""
        x_coords, y_coords = self.get_coords()
        centroid_x = sum(x_coords) / len(x_coords)
        centroid_y = sum(y_coords) / len(y_coords)
        return np.array([centroid_x, centroid_y])

    def line_intersection(self, p1, p2, q1, q2, epsilon=1e-9):
        """ Find intersection between two line segments (p1, p2) and (q1, q2) with floating-point tolerance """
        r = np.array(p2) - np.array(p1)
        s = np.array(q2) - np.array(q1)
        r = r.flatten()
        s = s.flatten()

        r_cross_s = np.cross(r, s)
        if abs(r_cross_s) < epsilon:
            return None  # Lines are parallel or collinear


        p1_q1 = np.array(q1) - np.array(p1)
        t = np.cross(p1_q1, s) / r_cross_s
        u = np.cross(p1_q1, r) / r_cross_s

        # Check if the intersection is within the bounds of the line segments
        if (0 <= t <= 1 or abs(t) < epsilon or abs(t - 1) < epsilon) and \
                (0 <= u <= 1 or abs(u) < epsilon or abs(u - 1) < epsilon):
            intersection_point = p1 + t * r
            return intersection_point
        return None

    def is_point_in_intersections(self, point, intersections, epsilon=1e-9):
        """ Check if a point is already in the intersections array, using a tolerance (epsilon) to handle floating-point precision.
        :param point: The intersection point to check (array-like)
        :param intersections: The list of existing intersection points
        :param epsilon: Tolerance for comparing floating-point numbers
        :return: True if the point is already in the intersections array, False otherwise
        """
        for existing_point in intersections:
            if np.linalg.norm(np.array(point) - np.array(existing_point)) < epsilon:
                return True
        return False

    def find_intersections(self, vector):
        """ Find all intersections of a vector with the polygon edges """
        intersections = []
        for edge in self.edges:
            intersection = self.line_intersection(
                [vector[0][0], vector[0][1]],
                [vector[1][0], vector[1][1]],
                [edge.v_from.x, edge.v_from.y],
                [edge.v_to.x, edge.v_to.y]
            )
            if intersection is not None and not self.is_point_in_intersections(intersection, intersections):
                intersections.append(intersection)

        # Edge case where vector just intersects 1 point, add the same point as intersection point
        if len(intersections) == 1:
            intersections.append(intersections[0])

        # Edge case where multiple intersections found (Duplicates around vertices)
        elif len(intersections) > 2:
            unique_points = []
            seen = set()
            for point in intersections:
                point_tuple = tuple(point)  # Convert the numpy array to a tuple
                if point_tuple not in seen:
                    unique_points.append(point)
                    seen.add(point_tuple)  # Track the unique points as tuples
            intersections = unique_points

        return intersections

