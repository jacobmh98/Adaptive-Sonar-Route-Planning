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

    def remove_vertex(self, v):
        self.vertices.remove(v)
        self.__init__(self.vertices)

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

    def plot(self, color):
        x_coords = []
        y_coords = []
        fig = plt.figure()

        for v in self.vertices:
            x_coords.append(v.x)
            y_coords.append(v.y)
            plt.text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex

        plt.plot(x_coords, y_coords, f'{color}-', marker='o')
        plt.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], f'{color}-')
        plt.show()