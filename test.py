import numpy as np
from matplotlib import pyplot as plt

from Polygon import Polygon, Vertex
from functions import polygons_are_adjacent2
def plot1():
    """ plot intersections """
    x_coords =[0.4, 0.1, 0.5, 0.9, 0.8, 0.2]
    y_coords = [0.4, 0.7, 0.9, 0.2, 0.1, 0.1]
    l1_x = [0, 1]
    l1_y = [0, 1]

    x_coords2 =[0.4, 0.1, 0.5, 0.9, 0.9, 0.8, 0.2]
    y_coords2 = [0.4, 0.7, 0.9, 0.3,0.2, 0.1, 0.1]
    l2_x = [0.4, 0.4]
    l2_y = [0, 1]

    x_coords3 =[0.45, 0.5, 0.3, 0.1, 0.5, 0.9, 0.9, 0.8, 0.2]
    y_coords3 = [0.35, 0.5, 0.5, 0.7, 0.9, 0.3,0.2, 0.1, 0.1]
    l3_x = [0.45, 0.45]
    l3_y = [0, 1]

    fig, ax = plt.subplots(1, 1)

    """for v in self.vertices:
                x_coords.append(v.x)
                y_coords.append(v.y)
                plt.text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex
    """
    ax.plot(x_coords3, y_coords3, f'k-', marker='o')
    ax.plot([x_coords3[-1], x_coords3[0]], [y_coords3[-1], y_coords3[0]], f'k-')
    ax.plot(l3_x, l3_y, linestyle='dotted', color='red')
    ax.set_aspect('equal')
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    plt.show()


def scale_points(x_coords, y_coords, x_min, x_max, y_min, y_max):

    x_scaled = []
    y_scaled = []

    for i in range(len(x_coords)):
        x_scaled.append((x_coords[i] - x_min) / (x_max - x_min) if x_max != x_min else 0)
        y_scaled.append((y_coords[i] - y_min) / (y_max - y_min) if y_max != y_min else 0)

    # Apply the scaling to all points
    return x_scaled, y_scaled

def get_center_of_polygon(x_coords, y_coords):
    """ Compute the center of a polygon"""
    return np.sum(x_coords) / len(x_coords), np.sum(y_coords) / len(y_coords)

def plot2():
    """ asd """
    p1_x = [2, 1, 3, 4]
    p1_y = [5, 3, 3, 5]

    p2_x = [3, 4, 4]
    p2_y = [3, 1, 5]

    p3_x = [4, 5, 4]
    p3_y = [1, 3, 5]

    p4_x = [4.6, 5, 4]
    p4_y = [3.8, 5, 5]

    p5_x = [4, 3, 4, 5] # merge
    p5_y = [5, 3, 1, 3] #merge

    min_x = np.min(np.array(p1_x + p2_x + p3_x + p4_x))
    min_y = np.min(np.array(p1_y + p2_y + p3_y + p4_y))
    max_x = np.max(np.array(p1_x + p2_x + p3_x + p4_x))
    max_y = np.max(np.array(p1_y + p2_y + p3_y + p4_y))

    print(f'{min_x=}, {max_x=}, {min_y=}, {max_y=}')

    p1_x, p1_y = scale_points(p1_x, p1_y, min_x, max_x, min_y, max_y)
    p2_x, p2_y = scale_points(p2_x, p2_y, min_x, max_x, min_y, max_y)
    p3_x, p3_y = scale_points(p3_x, p3_y, min_x, max_x, min_y, max_y)
    p4_x, p4_y = scale_points(p4_x, p4_y, min_x, max_x, min_y, max_y)
    p5_x, p5_y = scale_points(p5_x, p5_y, min_x, max_x, min_y, max_y)

    vertices_P1 = []
    for i in range(len(p1_x)):
        vertices_P1.append(Vertex(i, p1_x[i], p1_y[i]))
    vertices_P2 = []
    for i in range(len(p2_x)):
        vertices_P2.append(Vertex(i, p2_x[i], p2_y[i]))
    vertices_P3 = []
    for i in range(len(p3_x)):
        vertices_P3.append(Vertex(i, p3_x[i], p3_y[i]))
    vertices_P4 = []
    for i in range(len(p4_x)):
        vertices_P4.append(Vertex(i, p4_x[i], p4_y[i]))
    P1 = Polygon(vertices_P1)
    P2 = Polygon(vertices_P2)
    P3 = Polygon(vertices_P3)
    P4 = Polygon(vertices_P4)

    fig, ax = plt.subplots(1, 1)
    ax.plot(p1_x, p1_y, 'k-o')
    ax.plot([p1_x[-1], p1_x[0]], [p1_y[-1], p1_y[0]], f'k-')
    ax.plot(p2_x, p2_y, 'k-o')
    ax.plot([p2_x[-1], p2_x[0]], [p2_y[-1], p2_y[0]], f'k-')
    ax.plot(p3_x, p3_y, 'k-o')
    ax.plot([p3_x[-1], p3_x[0]], [p3_y[-1], p3_y[0]], f'k-')
    ax.plot(p4_x, p4_y, 'k-o')
    ax.plot([p4_x[-1], p4_x[0]], [p4_y[-1], p4_y[0]], f'k-')
    ax.plot(p5_x, p5_y, 'k-o')
    ax.plot([p5_x[-1], p5_x[0]], [p5_y[-1], p5_y[0]], f'k-')

    ax.text(get_center_of_polygon(p1_x, p1_y)[0], get_center_of_polygon(p1_x, p1_y)[1], 'P1', color='red')
    ax.text(get_center_of_polygon(p2_x, p2_y)[0], get_center_of_polygon(p2_x, p2_y)[1], 'P2', color='red')
    ax.text(get_center_of_polygon(p3_x, p3_y)[0], get_center_of_polygon(p3_x, p3_y)[1], 'P3', color='red')
    ax.text(get_center_of_polygon(p4_x, p4_y)[0], get_center_of_polygon(p4_x, p4_y)[1], 'P4', color='red')
    #ax.text(get_center_of_polygon(p5_x, p5_y)[0], get_center_of_polygon(p5_x, p5_y)[1], 'P5', color='red')

    ax.set_aspect('equal')

    plt.show()

    return P1, P2, P3, P4

P1, P2, P3, P4 = plot2()

polygons_are_adjacent2(P3, P4, 3, 4)