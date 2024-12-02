import matplotlib.pyplot as plt
import numpy as np
import decomposition
import networkx as nx

""" Temporary plot functions """
def plot_results(split_polygons, depth, cv, Di):
    """ Create a figure with a grid of sub-plots """
    cols = 5
    rows = int(np.ceil(len(split_polygons) / cols))

    fig, ax = plt.subplots(rows, cols)

    r = 0
    c = 0
    count = 0

    for (P1, P2) in split_polygons:
        P1_coords = P1.vertices_matrix()
        P2_coords = P2.vertices_matrix()

        ax[r, c].plot(P1_coords[0, :], P1_coords[1, :], 'b-')
        ax[r, c].plot([P1_coords[0, :][-1], P1_coords[0, :][0]], [P1_coords[1, :][-1], P1_coords[1, :][0]], 'b-')
        ax[r, c].plot(P2_coords[0, :], P2_coords[1, :], 'r-')
        ax[r, c].plot([P2_coords[0, :][-1], P2_coords[0, :][0]], [P2_coords[1, :][-1], P2_coords[1, :][0]], 'r-')
        ax[r, c].set_title(f'D{cv},{count} = {np.round(Di[count], 1)}')
        ax[r, c].axis('equal')
        count += 1

        # ax[r, c].quiver(cv.x, cv.y, vec[0], vec[1], angles='xy', scale_units='xy', scale=1, color='r', width=0.015)
        # ax[plot_r, c].quiver(cv.x, cv.y, v_dir2[0], v_dir2[1], angles='xy', scale_units='xy', scale=1, color='g')
        c += 1

        if c != 0 and c % cols == 0:
            r += 1
            c = 0
    fig.tight_layout()
    plt.show()

def plot_results2(P, P1, P2, depth, cv, edge, Dij):
    fig, ax = plt.subplots(1, 4)

    P1_coords = P1.vertices_matrix()
    P2_coords = P2.vertices_matrix()
    P_coords = P.vertices_matrix()

    for v in P.vertices:
        ax[0].text(v.x, v.y, f'{v.index}', fontsize=12, ha='right', color='red')  # Draw the index near the vertex

    ax[0].plot(P_coords[0, :], P_coords[1, :], color='black', marker='o')
    ax[0].plot(P_coords[0, :], P_coords[1, :], 'k-')
    ax[0].plot(P_coords[0, :], P_coords[1, :], 'k-')
    ax[0].plot([P_coords[0, :][-1], P_coords[0, :][0]], [P_coords[1, :][-1], P_coords[1, :][0]], 'k-')
    #ax[0].plot(P2_coords[0, :], P2_coords[1, :], 'r-')
    #ax[0].plot([P2_coords[0, :][-1], P2_coords[0, :][0]], [P2_coords[1, :][-1], P2_coords[1, :][0]], 'r-')
    ax[0].set_title(f'P')
    ax[0].axis('equal')

    #ax[1].plot(P1_coords[0, :], P1_coords[1, :], color='blue', marker='o')
    ax[1].plot(P1_coords[0, :], P1_coords[1, :], 'b-o')
    ax[1].plot([P1_coords[0, :][-1], P1_coords[0, :][0]], [P1_coords[1, :][-1], P1_coords[1, :][0]], 'b-o')
    ax[1].plot(P2_coords[0, :], P2_coords[1, :], 'r-o')
    ax[1].plot([P2_coords[0, :][-1], P2_coords[0, :][0]], [P2_coords[1, :][-1], P2_coords[1, :][0]], 'r-o')
    ax[1].set_title(f'P1 & P2')
    ax[1].axis('equal')

#    ax[2].plot(P1_coords[0, :], P1_coords[1, :], color='black', marker='o')
    ax[2].plot(P1_coords[0, :], P1_coords[1, :], 'b-o')
    ax[2].plot([P1_coords[0, :][-1], P1_coords[0, :][0]], [P1_coords[1, :][-1], P1_coords[1, :][0]], 'b-o')
    ax[2].set_title(f'P1')
    ax[2].axis('equal')

    #ax[3].plot(P2_coords[0, :], P2_coords[1, :], color='black', marker='o')
    ax[3].plot(P2_coords[0, :], P2_coords[1, :], 'r-o')
    ax[3].plot([P2_coords[0, :][-1], P2_coords[0, :][0]], [P2_coords[1, :][-1], P2_coords[1, :][0]], 'r-o')
    ax[3].set_title(f'P2')
    ax[3].axis('equal')

    #print(f'\t{depth=}')
    #print(f'\t{cv=}')
    #print(f'\t{edge=}')
    #print(f'\tD_ij={np.round(Dij, 1)}')
    #print(f'P1 = {P1.vertices_matrix()}')
    #print(f'P2 = {P2.vertices_matrix()}')
    #print(f'\tP1 = {P1.vertices}')
    #print(f'\tP2 = {P2.vertices}')

    fig.tight_layout()
    #mng = plt.get_current_fig_manager()
    #mng.full_screen_toggle()

    plt.show()

def plot_results3(sub_polygons, include_points = False):
    fig, ax = plt.subplots(1,1)

    for i, poly in enumerate(sub_polygons):
        x_coords, y_coords = poly.get_coords()

        c_x, c_y = decomposition.get_center_of_polygon(poly)

        if include_points:
            ax.plot(x_coords, y_coords, 'k-o')

            for c in range(len(x_coords)):
                ax.text(x_coords[c], y_coords[c], f'{c}')

        else:
            ax.plot(x_coords, y_coords, 'k-')

        ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'k-')

        ax.text(c_x - 0.1, c_y, f'P{i}', color='r', fontsize=7)

        #ax.plot(x_coords, y_coords, 'k-')
        #ax.plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'k-o')
        ax.set_aspect('equal')
        #ax.set_title('Antwerpen Decomposition')
    plt.show()

def plot_graph(sub_polygons):
    fig, ax = plt.subplots(1)
    G = nx.Graph()

    # Go through each edge in p_i and compare with each edge in p_j
    for i, p_i in enumerate(sub_polygons):
        G.add_node(f'P{i}')
        for j, p_j in enumerate(sub_polygons):
            # Ignore if the two polygons are equal
            if i >= j:
                continue

            # Test if the two polygons p_i and p_j are adjacent (either complete or partial)
            if decomposition.polygons_are_adjacent(p_i, p_j, i, j):
                # Update the adjacent matrix
                G.add_edge(f'P{i}', f'P{j}')


    pos = nx.spring_layout(G)  # Layout for node positions
    nx.draw(G, pos, ax=ax, with_labels=True, node_color='lightblue', edge_color='gray', node_size=500, font_size=10)
    #ax.set_title('Undirected Graph')
    plt.show()

def plot_polygons(P, sub_polygons, G):
    fig, ax = plt.subplots(1, 3)
    x_coords, y_coords = P.get_coords()
    ax[0].plot(x_coords, y_coords, 'k-')
    ax[0].plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'k-')
    ax[0].set_title('Initial Polygon')
    ax[0].set_aspect('equal')

    for i, p in enumerate(sub_polygons):
        c_x, c_y = get_center_of_polygon(p)
        x_coords, y_coords = p.get_coords()
        ax[1].plot(x_coords, y_coords, 'k-')
        ax[1].plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'k-')
        ax[1].text(c_x-0.1, c_y, f'P{i}', color='r', fontsize=7)
    ax[1].set_title('Decomposition of Polygon')
    ax[1].set_aspect('equal')

    pos = nx.spring_layout(G)  # Layout for node positions
    nx.draw(G, pos, ax=ax[2], with_labels=True, node_color='lightblue', edge_color='gray', node_size=500, font_size=10)
    ax[2].set_title('Undirected Graph')
    #ax[2].set_aspect('equal')
    """fig, ax = plt.subplots(1, len(sub_polygons))
    for i, p in enumerate(sub_polygons):
        c_x, c_y = get_center_of_polygon(p)
        x_coords, y_coords = p.get_coords()
        ax[i].plot(x_coords, y_coords, 'k-')
        ax[i].plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'k-')
        ax[i].text(c_x, c_y, f'{i}')
        ax[i].set_title(f'P{i}')
        ax[i].set_aspect('equal')"""
    plt.show()

def plot_polygons2(P, sub_polygons, optimized_sub_polygons):
    fig, ax = plt.subplots(1, 3)
    x_coords, y_coords = P.get_coords()
    ax[0].plot(x_coords, y_coords, 'k-')
    ax[0].plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'k-')
    ax[0].set_title('Initial Polygon')
    ax[0].set_aspect('equal')

    for i, p in enumerate(sub_polygons):
        c_x, c_y = get_center_of_polygon(p)
        x_coords, y_coords = p.get_coords()
        ax[1].plot(x_coords, y_coords, 'k-')
        ax[1].plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'k-')
        ax[1].text(c_x-0.1, c_y, f'P{i}', color='r', fontsize=7)
    ax[1].set_title('Decomposition of Polygon')
    ax[1].set_aspect('equal')

    for i, p in enumerate(optimized_sub_polygons):
        c_x, c_y = get_center_of_polygon(p)
        x_coords, y_coords = p.get_coords()
        ax[2].plot(x_coords, y_coords, 'k-')
        ax[2].plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'k-')
        ax[2].text(c_x - 0.1, c_y, f'P{i}', color='r', fontsize=7)
    ax[2].set_title('Merging Polygons')
    ax[2].set_aspect('equal')
    #ax[2].set_aspect('equal')
    """fig, ax = plt.subplots(1, len(sub_polygons))
    for i, p in enumerate(sub_polygons):
        c_x, c_y = get_center_of_polygon(p)
        x_coords, y_coords = p.get_coords()
        ax[i].plot(x_coords, y_coords, 'k-')
        ax[i].plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], 'k-')
        ax[i].text(c_x, c_y, f'{i}')
        ax[i].set_title(f'P{i}')
        ax[i].set_aspect('equal')"""
    plt.show()

def save_polygon(P, P1, P2, depth, n, D_ij, color='k', name='fig'):
    fig, ax = plt.subplots(1, 3)

    x_coords, y_coords = P.get_coords()
    ax[0].plot(x_coords, y_coords, f'{color}-', marker='o')
    ax[0].plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], f'{color}-')
    ax[0].set_title(f'{depth=}, {n=}, D_ij = {np.round(D_ij, 1)}')
    ax[0].set_aspect('equal')

    x_coords, y_coords = P1.get_coords()
    ax[1].plot(x_coords, y_coords, f'{color}-', marker='o')
    ax[1].plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], f'{color}-')
    ax[1].set_title(f'P1')
    ax[1].set_aspect('equal')

    x_coords, y_coords = P2.get_coords()
    ax[2].plot(x_coords, y_coords, f'{color}-', marker='o')
    ax[2].plot([x_coords[-1], x_coords[0]], [y_coords[-1], y_coords[0]], f'{color}-')
    ax[2].set_title(f'P2')
    ax[2].set_aspect('equal')

    plt.savefig(f'./figs/{name}{id(P)}.png')
    plt.close()