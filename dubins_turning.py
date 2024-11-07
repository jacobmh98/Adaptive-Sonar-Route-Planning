import numpy as np


def dubins_arc(center, start_angle, end_angle, radius, direction, num_points=20):
    """
    Generate points along a Dubins arc with a fixed radius.
    """
    if direction == 'L':  # Left turn
        if end_angle < start_angle:
            end_angle += 2 * np.pi
    else:  # Right turn
        if end_angle > start_angle:
            end_angle -= 2 * np.pi

    angles = np.linspace(start_angle, end_angle, num_points)
    arc_points = np.array([
        [center[0] + radius * np.cos(angle), center[1] + radius * np.sin(angle)]
        for angle in angles
    ])
    return arc_points


def dubins_straight(start, end, num_points=10):
    """
    Generate points along a straight line between start and end.
    """
    return np.linspace(start, end, num_points)


def calculate_heading(point1, point2):
    """
    Calculate the heading angle (in radians) from point1 to point2.
    """
    return np.arctan2(point2[1] - point1[1], point2[0] - point1[0])


def dubins_path_segment(start, start_heading, end, end_heading, radius):
    """
    Generate the shortest Dubins path segment between two points with specific start and end headings.
    """
    paths = {
        'LSL': dubins_LSL(start, start_heading, end, end_heading, radius),
        'RSR': dubins_RSR(start, start_heading, end, end_heading, radius),
        'LSR': dubins_LSL(start, start_heading, end, end_heading, radius, left_turn_first=False),
        'RSL': dubins_RSR(start, start_heading, end, end_heading, radius, left_turn_first=False),
        'LRL': dubins_LRL(start, start_heading, end, end_heading, radius),
        'RLR': dubins_RLR(start, start_heading, end, end_heading, radius)
    }

    shortest_path_type = min(paths, key=lambda k: paths[k][1])
    shortest_path = paths[shortest_path_type][0]

    return shortest_path


def dubins_path_with_heading(path_points, radius):
    """
    Generate a full Dubins path for a series of points with fixed turning radius.
    Calculates start and end headings for each segment based on adjacent points.
    """
    full_dubins_path = []

    for i in range(1, len(path_points) - 1):
        start = path_points[i]
        end = path_points[i + 1]

        # Calculate start and end headings based on adjacent points
        start_heading = calculate_heading(path_points[i - 1], start)
        end_heading = calculate_heading(end, path_points[i + 2]) if (i + 2) < len(path_points) else start_heading

        # Generate the Dubins path segment for this part of the path
        dubins_segment_path = dubins_path_segment(start, start_heading, end, end_heading, radius)

        # Append to the full path, excluding the last point to avoid duplicates
        full_dubins_path.extend(dubins_segment_path[:-1])

    # Add the final end point to complete the path
    full_dubins_path.append(path_points[-1])

    return np.array(full_dubins_path)


def dubins_LSL(start, start_heading, end, end_heading, radius, left_turn_first=True):
    """
    Calculate an LSL (or LSR if left_turn_first=False) Dubins path from start to end with fixed turning radius.
    """
    if left_turn_first:
        turn1_direction, turn2_direction = 'L', 'L'
    else:
        turn1_direction, turn2_direction = 'L', 'R'

    turn_center1 = np.array([start[0] + radius * np.sin(start_heading),
                             start[1] - radius * np.cos(start_heading)])
    turn_center2 = np.array([end[0] + radius * np.sin(end_heading),
                             end[1] - radius * np.cos(end_heading)])

    # Calculate angles for each arc
    start_angle1 = start_heading
    end_angle1 = np.arctan2(turn_center2[1] - turn_center1[1], turn_center2[0] - turn_center1[0])

    # First arc
    arc1 = dubins_arc(turn_center1, start_angle1, end_angle1, radius, turn1_direction)

    # Straight segment between arcs
    straight_segment_start = arc1[-1]
    straight_segment_end = np.array([turn_center2[0] - radius * np.cos(end_heading),
                                     turn_center2[1] - radius * np.sin(end_heading)])
    straight_segment = dubins_straight(straight_segment_start, straight_segment_end)

    # Second arc
    start_angle2 = end_angle1
    arc2 = dubins_arc(turn_center2, start_angle2, end_heading, radius, turn2_direction)

    # Combine path segments
    path = np.vstack([arc1, straight_segment, arc2])
    path_length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
    return path, path_length


def dubins_RSR(start, start_heading, end, end_heading, radius, left_turn_first=True):
    """
    Calculate an RSR (or RSL if left_turn_first=False) Dubins path from start to end with fixed turning radius.
    """
    if left_turn_first:
        turn1_direction, turn2_direction = 'R', 'R'
    else:
        turn1_direction, turn2_direction = 'R', 'L'

    turn_center1 = np.array([start[0] - radius * np.sin(start_heading),
                             start[1] + radius * np.cos(start_heading)])
    turn_center2 = np.array([end[0] - radius * np.sin(end_heading),
                             end[1] + radius * np.cos(end_heading)])

    # Calculate angles for each arc
    start_angle1 = start_heading
    end_angle1 = np.arctan2(turn_center2[1] - turn_center1[1], turn_center2[0] - turn_center1[0])

    # First arc
    arc1 = dubins_arc(turn_center1, start_angle1, end_angle1, radius, turn1_direction)

    # Straight segment between arcs
    straight_segment_start = arc1[-1]
    straight_segment_end = np.array([turn_center2[0] + radius * np.cos(end_heading),
                                     turn_center2[1] + radius * np.sin(end_heading)])
    straight_segment = dubins_straight(straight_segment_start, straight_segment_end)

    # Second arc
    start_angle2 = end_angle1
    arc2 = dubins_arc(turn_center2, start_angle2, end_heading, radius, turn2_direction)

    # Combine path segments
    path = np.vstack([arc1, straight_segment, arc2])
    path_length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
    return path, path_length


def dubins_LRL(start, start_heading, end, end_heading, radius):
    """
    Calculate an LRL Dubins path from start to end with fixed turning radius.
    """
    turn_center1 = np.array([start[0] + radius * np.sin(start_heading),
                             start[1] - radius * np.cos(start_heading)])
    turn_center2 = np.array([end[0] + radius * np.sin(end_heading),
                             end[1] - radius * np.cos(end_heading)])

    # First and second arcs
    arc1 = dubins_arc(turn_center1, start_heading, start_heading + np.pi, radius, 'L')
    arc2 = dubins_arc(turn_center2, end_heading + np.pi, end_heading, radius, 'L')

    # Combine path segments
    path = np.vstack([arc1, arc2])
    path_length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
    return path, path_length


def dubins_RLR(start, start_heading, end, end_heading, radius):
    """
    Calculate an RLR Dubins path from start to end with fixed turning radius.
    """
    turn_center1 = np.array([start[0] - radius * np.sin(start_heading),
                             start[1] + radius * np.cos(start_heading)])
    turn_center2 = np.array([end[0] - radius * np.sin(end_heading),
                             end[1] + radius * np.cos(end_heading)])

    # First and second arcs
    arc1 = dubins_arc(turn_center1, start_heading, start_heading + np.pi, radius, 'R')
    arc2 = dubins_arc(turn_center2, end_heading + np.pi, end_heading, radius, 'R')

    # Combine path segments
    path = np.vstack([arc1, arc2])
    path_length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
    return path, path_length