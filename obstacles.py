from matplotlib import pyplot as plt

from Polygon import Polygon


def asd(polygon, obstacle):
    # Go through each vertex in the obstacle
    # Sort each vertex by its x-value

    # Insertion Sort
    sorted_obstacle_vertices = []

    for i, v in enumerate(obstacle.vertices):
        if i == 0:
            sorted_obstacle_vertices.index(v)
            continue

        for j, v2 in enumerate(sorted_obstacle_vertices):
            if v.x >= v2.x:
                sorted_obstacle_vertices.insert(j, v)
                break

            if j == len(sorted_obstacle_vertices) - 1:
                sorted_obstacle_vertices.append(v)

    obstacle.plot()
    polygon.plot()

    # Divide each vertex into its category
        # OPEN
        # CLOSE
        # SPLIT
        # MERGE
        # FLOOR_CONVEX
        # FLOOR_CONCAVE
        # CEIL_CONVEX
        # CEIL_CONCAVE

    # Create a binary tree
    # Handle edge cases later
    None

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