import numpy as np
import matplotlib.pyplot as plt

class Vertex:
    def __init__(self, i, x, y):
        self.index = i
        self.x = x
        self.y = y
        self.v = np.array([x, y])
        self.prev = None
        self.next = None

    def __repr__(self):
        return f"Vertex({self.index}, {self.x}, {self.y})"

class Edge:
    def __init__(self, v1, v2):
        self.v_from = v1
        self.v_to = v2

    def __repr__(self):
        return f"Edge({self.v_from.index}, {self.v_to.index})"

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

    def plot(self):
        x_coords = []
        y_coords = []
        fig = plt.figure()

        for v in self.vertices:
            x_coords.append(v.x)
            y_coords.append(v.y)
            plt.text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex

        plt.plot(x_coords, y_coords, 'b-', marker='o')
        plt.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'b-')
        plt.show()

    def line_intersection(self, p1, p2, q1, q2):
        """ Find intersection between two line segments (p1, p2) and (q1, q2) """
        # Convert points to vectors
        r = np.array(p2) - np.array(p1)
        s = np.array(q2) - np.array(q1)

        # Cross product r x s
        r_cross_s = np.cross(r, s)

        if r_cross_s == 0:
            # Lines are parallel
            return None

        # Vector from p1 to q1
        p1_q1 = np.array(q1) - np.array(p1)

        # Find intersection parameters
        t = np.cross(p1_q1, s) / r_cross_s
        u = np.cross(p1_q1, r) / r_cross_s

        # Check if ntersection point is within both line segments
        if 0 <= t <= 1 and 0 <= u <= 1:
            intersection_point = p1 + t * r
            return intersection_point
        else:
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

        # Edge case where vector just intersects 1 point
        if len(intersections) == 1:
            intersections.append(intersections[0])
        return intersections