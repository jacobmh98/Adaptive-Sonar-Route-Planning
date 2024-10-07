import numpy as np


def closest_vertex_to_intersection(intersections, poly):
    """
    Find the closest vertex-intersection pair and return the vertex from the polygon and the intersection point.

    :param intersections: List of intersection pairs (each pair contains two points [(x1, y1), (x2, y2)])
    :param poly: Polygon object
    :return: Tuple containing the closest vertex (as a Vertex object) and the closest intersection point (as a NumPy array)
    """
    min_distance = float('inf')
    target_vertex = None
    closest_intersection = None
    intersection_pair_index = None

    for i, intersection_pair in enumerate(intersections):
        for vertex in poly.vertices:
            vertex_coords = np.array([vertex.x, vertex.y])

            # Check both points in the intersection pair
            for intersection in intersection_pair:
                distance = np.linalg.norm(vertex_coords - intersection)

                # Update the closest vertex-intersection pair if a smaller distance is found
                if distance < min_distance:
                    min_distance = distance
                    target_vertex = vertex
                    closest_intersection = intersection
                    intersection_pair_index = i

    return target_vertex, closest_intersection, intersection_pair_index


def check_and_connect(path, p1, p2):
    """
    Adds points to the path in the correct order.

    :param path: List of points, containing the current path
    :param p1: Point one (NumPy array)
    :param p2: Point two (NumPy array)
    :return path: List of points with p1 and p2 added in correct order
    """
    n = len(path) - 1

    if n < 0:
        # Start path by adding the closest point
        if np.linalg.norm(p1) <= np.linalg.norm(p2):
            path.extend([p1, p2])  # Extend the path with p1 followed by p2
        else:
            path.extend([p2, p1])  # Extend the path with p2 followed by p1
    else:
        # Continue path at the next closest point
        if np.linalg.norm(path[n] - p1) <= np.linalg.norm(path[n] - p2):
            path.append(p1)
            path.append(p2)
        else:
            path.append(p2)
            path.append(p1)

    return path


def connect_path(polygons, total_intersections):
    """
    Connects the paths of the polygons by adding intersections in the correct order based on the closest vertex.

    :param polygons: List of Polygon objects
    :param total_intersections: List of intersection pairs for each polygon
    :return: Total path connecting all polygons as a flat NumPy array
    """
    total_path = []

    for i, poly in enumerate(polygons[:-1]):
        print(f'Processing polygon {i}')

        # Find the target vertex and closest intersection for the current polygon
        target, closest_intersection, closest_intersection_index = closest_vertex_to_intersection(
            total_intersections[i], polygons[i + 1])

        print(
            f"Closest vertex: {target}, Closest intersection: {closest_intersection}, Index: {closest_intersection_index}")

        # Check if the last intersection in the list is closer to the target vertex
        last_intersection = total_intersections[i][-1][1]
        first_intersection = total_intersections[i][0][0]

        distance_to_last = np.linalg.norm(np.array([target.x, target.y]) - last_intersection)
        distance_to_first = np.linalg.norm(np.array([target.x, target.y]) - first_intersection)

        # Determine direction to add intersections based on proximity
        if distance_to_last < distance_to_first:
            print("Adding intersections in reverse order")
            total_path = add_remaining_intersections(total_path, total_intersections[i], closest_intersection_index,
                                                     reverse=True)
        else:
            print("Adding intersections in normal order")
            total_path = add_remaining_intersections(total_path, total_intersections[i], closest_intersection_index,
                                                     reverse=False)

        # Add the closest vertex as the next point in the path
        total_path.append(np.array([target.x, target.y]))

    # Convert total_path to a flat NumPy array
    return np.array(total_path)


