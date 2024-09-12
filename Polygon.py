import numpy as np
import matplotlib.pyplot as plt

class Vertex:
    def __init__(self, i, x, y):
        self.index = i
        self.x = x
        self.y = y
        self.prev = None
        self.next = None

    def __repr__(self):
        return f"Vertex({self.index}, {self.x}, {self.y})"

class Edge:
    def __init__(self, v1, v2):
        self.v_from = v1
        self.v_to = v2
        self.slope = (v2.y - v1.y) / (v2.x - v1.x)

    def __repr__(self):
        return f"Edge({self.v_from.index}, {self.v_to.index}, m={self.slope:.1f})"

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

    def compute_concave_vertices(self):
        """ Compute the concave vertices in the polygon """

        for i in range(self.number_vertices):
            # Find the adjacent vertices (ccw order)
            v_left = self.vertices[(i - 1) % self.number_vertices]
            v = self.vertices[i]
            v_right = self.vertices[(i + 1) % self.number_vertices]

            # Computing the concave judgement matrix
            S_vi = np.linalg.det(np.array([[v_left.x, v_left.y, 1],
                                           [v.x, v.y, 1],
                                           [v_right.x, v_right.y, 1]]))

            # Test if the vertex is concave and add it to the list if true
            if S_vi < 0:
                self.concave_vertices.append(v)

    def get_coords(self):
        """ Get the vertices as a list of x-coordinates and y-coordinates"""
        x_coords = []
        y_coords = []

        for v in self.vertices:
            x_coords.append(v.x)
            y_coords.append(v.y)

        return x_coords, y_coords