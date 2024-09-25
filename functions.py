from asyncio import current_task
import numpy as np

def compute_polygon_centroid(pts):
    n = len(pts)

from Polygon import *
from scipy.spatial import ConvexHull
import networkx as nx

def compute_concave_vertices(P):
    """ Function to compute the concave vertices in a polygon """
    concave_vertices = []

    return [x/n, y/n]

def get_rectangle_width_height(coords):
    width = 0  # x
    height = 0  # y
    for coord in coords:  # coord = [x, y]
        if coord[0] > width:
            width = coord[0]
        if coord[1] > height:
            height = coord[1]
    return width, height

def sweeping_path_rectangle(path_width, coords, speed):
    path = []
    y = coords[0][0] + path_width/2
    start = [0, y]  # Start bottom left, but with height of center of path width
    path.append(start)
    forward = True  # Flag to determine path direction
    width, height = get_rectangle_width_height(coords)  # Finding rectangle dimensions
    point_counter = 0

    while True:
        current_pos = path[point_counter]

        if forward:  # Left to right direction
            if current_pos[0] < (width - path_width/2):
                path.append([round(current_pos[0] + speed, 2), current_pos[1]])
                point_counter += 1

            else:  # Hit right edge of rectangle minus half path width
                if round(current_pos[1] + path_width,2) < height:  # If space for 1 path, move up and change dir, finish after
                    path.append([current_pos[0], round(current_pos[1] + path_width, 2)])
                    point_counter += 1

                elif round(current_pos[1] + path_width/2, 2) < height:  # For complete coverage must check half path width
                    path.append([current_pos[0], round(current_pos[1] + path_width/2, 2)])
                    point_counter += 1
                    print("her")

                else:  # If no space, then move to edge and break
                    while current_pos[0] < width:  # Moving right, stops when hitting edge
                        path.append([round(current_pos[0] + speed, 2), current_pos[1]])
                        point_counter += 1
                        current_pos = path[point_counter]
                    break

                forward = False  # If top have not been hit, change direction
                continue

        if not forward:  # Right to left direction
            if current_pos[0] > (0 + path_width/2):
                path.append([round(current_pos[0] - speed, 2), current_pos[1]])
                point_counter += 1

            else:
                if round(current_pos[1] + path_width,2) < height: # If space for 1 path, move up and change dir, finish after
                    path.append([current_pos[0], round(current_pos[1] + path_width, 2)])
                    point_counter += 1

                elif round(current_pos[1] + path_width/2, 2) < height:  # For complete coverage must check half path width
                    path.append([current_pos[0], round(current_pos[1] + path_width/2, 2)])
                    point_counter += 1

                else:  # If no space, then move to edge and break
                    while current_pos[0] > 0:  # Moving left, stops when hitting edge
                        path.append([round(current_pos[0] - speed, 2), current_pos[1]])
                        point_counter += 1
                        current_pos = path[point_counter]
                    break
                forward = True  # Change direction

    return path
