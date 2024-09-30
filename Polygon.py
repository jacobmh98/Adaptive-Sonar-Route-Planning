import numpy as np
import matplotlib.pyplot as plt

class Vertex:
    def __init__(self, i, x, y):
        self.index = i
        self.x = x
        self.y = y
        self.v = np.array([[x], [y]])
        self.prev = None
        self.next = None

    def get_array(self):
        return np.array([[self.x], [self.y]])

    def __repr__(self):
        return f"Vertex({self.index}, {self.x}, {self.y})"

class Edge:
    def __init__(self, v1, v2):
        self.v_from = v1
        self.v_to = v2
        self.prev = None
        self.next = None
        if (v2.x - v1.x) == 0:
            self.slope = 0
        else:
            self.slope = (v2.y - v1.y) / (v2.x - v1.x)

    def __repr__(self):
        return f"Edge({self.v_from.index}, {self.v_to.index}, {self.slope})"

class Polygon:
    def __init__(self, v):
        self.vertices = v
        self.number_vertices = len(self.vertices)
        self.concave_vertices = []
        self.edges = []

        for i in range(len(self.vertices)):
            v1 = self.vertices[i]
            v2 = self.vertices[(i + 1) % self.number_vertices]
            self.edges.append(Edge(v1, v2))

            v1.prev = self.vertices[(i - 1) % self.number_vertices]
            v1.next = self.vertices[(i + 1) % self.number_vertices]

        for j in range(len(self.edges)):
            self.edges[j].prev = self.edges[(j - 1) % len(self.edges)]
            self.edges[j].next = self.edges[(j + 1) % len(self.edges)]

    def get_coords(self):
        """ Get the vertices as a list of x-coordinates and y-coordinates"""
        x_coords = []
        y_coords = []

        for v in self.vertices:
            x_coords.append(v.x)
            y_coords.append(v.y)

        return x_coords, y_coords

    def vertices_matrix(self):
        """ Function to get the vertices in the polygon in a 2xn list """
        x_coords, y_coords = self.get_coords()

        return np.array([x_coords, y_coords])

    def plot(self, color='k'):
        x_coords = []
        y_coords = []
        fig, ax = plt.subplots(1, 1)

        for v in self.vertices:
            x_coords.append(v.x)
            y_coords.append(v.y)
            plt.text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex

        ax.plot(x_coords, y_coords, f'{color}-', marker='o')
        ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], f'{color}-')
        ax.set_aspect('equal')
        plt.show()

    def line_intersection(self, p1, p2, q1, q2, epsilon=1e-9):
        """ Find intersection between two line segments (p1, p2) and (q1, q2) with floating-point tolerance """
        r = np.array(p2) - np.array(p1)
        s = np.array(q2) - np.array(q1)

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

    def find_intersections(self, vector_start, vector_end):
        """ Find all intersections of a vector with the polygon edges """
        intersections = []
        for edge in self.edges:
            intersection = self.line_intersection(
                [vector_start[0], vector_start[1]],
                [vector_end[0], vector_end[1]],
                [edge.v_from.x, edge.v_from.y],
                [edge.v_to.x, edge.v_to.y]
            )
            if intersection is not None:
                intersections.append(intersection)

        # Edge case where vector just intersects 1 point, add the same point as intersection point
        if len(intersections) == 1:
            intersections.append(intersections[0])
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
