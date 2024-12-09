import numpy as np
import matplotlib.pyplot as plt
from global_variables import *
from shapely.geometry import Polygon as ShapelyPolygon, MultiPolygon, LineString
from shapely.ops import unary_union

def compute_distance(path, transit_flags):
    """ Function to compute the length of the path

    :param path: List of points
    :param transit_flags: List of flags indicating if a point is transit
    :return: Total distance between all the points in the path
    """

    total_distance = 0.0
    path_distance = 0.0
    transit_distance = 0.0

    # Iterate over consecutive points in the path
    for i in range(len(path) - 1):
        # Compute the Euclidean distance between consecutive points
        dist = np.linalg.norm(path[i + 1] - path[i])
        total_distance += dist

        # Check flags for current and next points
        is_transit = transit_flags[i] == "transit" and transit_flags[i + 1] == "transit"

        if is_transit:
            transit_distance += dist
        else:
            path_distance += dist

    return total_distance, path_distance, transit_distance


def compute_turns(path):
    """ Function to compute the number of turns in the path, and classify them as three different turn types

    :param path: List of points
    :return total_turns: The total number of turns as int
    :return hard_turns: The total number of hard turns (<45) as int
    :return medium_turns: The total number of medium turns (45-90) as int
    :return soft_turns: The total number of soft turns (>90) as int
    """
    hard_turns = 0
    medium_turns = 0
    soft_turns = 0

    for i in range(1, len(path) - 1):
        # Compute vectors for consecutive points
        vector1 = path[i] - path[i - 1]
        vector2 = path[i + 1] - path[i]

        # Normalize the vectors
        unit_vector1 = vector1 / (np.linalg.norm(vector1) + epsilon)
        unit_vector2 = vector2 / (np.linalg.norm(vector2) + epsilon)

        # Compute the dot product and find the angle between vectors
        dot_product = np.dot(unit_vector1, unit_vector2)
        dot_product = np.clip(dot_product, -1.0, 1.0)  # Account for floating point precision error, using clip instead of epsilon value
        angle_rad = np.arccos(dot_product)  # Angle in radians
        angle_deg = np.degrees(angle_rad)  # Convert to degrees

        # Classify the turns based on the angle
        if angle_deg < 45 or angle_deg > 180:
            hard_turns += 1
        elif 45 <= angle_deg <= 90:
            medium_turns += 1
        else:
            soft_turns += 1

    total_turns = hard_turns + medium_turns + soft_turns

    return total_turns, hard_turns, medium_turns, soft_turns


def compute_covered_area(region, obstacles, path, transit_flags, current_path_width):
    """Compute the coverage of the path inside the region, accounting for obstacles and transit lines.

    :param region: Main region polygon as an instance of the Polygon class
    :param obstacles: List of Polygon instances representing obstacles (can be empty)
    :param path: List of points (as tuples or arrays) representing the path
    :param transit_flags: List of flags indicating if a segment is transit
    :param current_path_width: Float of chosen path width
    :return: Tuple containing the covered area as a Shapely Polygon and the coverage percentage (float)
    """
    # Convert the region into a Shapely Polygon
    region_coords = [(v.x, v.y) for v in region.vertices]
    region_shape = ShapelyPolygon(region_coords)

    # Combine obstacles into a single Shapely Geometry
    obstacles_shapes = unary_union([ShapelyPolygon([(v.x, v.y) for v in obstacle.vertices]) for obstacle in obstacles])

    # Subtract obstacles from the region to create the effective region
    effective_region = region_shape.difference(obstacles_shapes)

    # Create active path segments, excluding transit lines
    active_path_segments = []
    for i in range(len(path) - 1):
        # Exclude segments where both points are marked as transit
        if not (transit_flags[i] == "transit" and transit_flags[i + 1] == "transit"):
            active_path_segments.append(LineString([path[i], path[i + 1]]))

    if not active_path_segments:
        # No active segments, return empty polygon and zero coverage
        return ShapelyPolygon(), 0.0

    # Buffer the active path segments to represent the covered area
    buffered_path = unary_union(
        [seg.buffer(current_path_width / 2.0, resolution=16) for seg in active_path_segments]
    )

    # Restrict the buffered path to the effective region
    covered_area_with_obstacles = buffered_path.intersection(region_shape)
    covered_area = covered_area_with_obstacles.difference(obstacles_shapes)

    # Calculate the coverage percentage, excluding obstacle areas
    effective_region_area = effective_region.area
    coverage_percentage = (covered_area.area / effective_region_area) * 100 if effective_region_area > 0 else 0

    return covered_area, coverage_percentage


def compute_outlier_area(polygon, path, transit_flags, current_path_width):
    """ Computes the path area outside the polygon, excluding transit lines.

    :param polygon: Polygon
    :param path: List of points
    :param transit_flags: List of flags indicating if a segment is transit
    :param current_path_width: Float of chosen path width
    :return outlying_area: Shapely Polygon that lies outside the given polygon
    """
    # Filter out transit segments
    active_path_segments = []
    for i in range(len(path) - 1):
        # Only exclude segments where both points are marked as transit
        if not (transit_flags[i] == "transit" and transit_flags[i + 1] == "transit"):
            active_path_segments.append(LineString([path[i], path[i + 1]]))

    if not active_path_segments:
        return ShapelyPolygon()  # No active segments, return empty polygon

    # Buffer the active path segments
    buffered_path = unary_union([seg.buffer(current_path_width / 2.0) for seg in active_path_segments])

    # Convert your Polygon class to a Shapely Polygon
    poly_coords = [(v.x, v.y) for v in polygon.vertices]
    poly_shape = ShapelyPolygon(poly_coords)

    # Calculate the wasted area outside the polygon
    outlying_area = buffered_path.difference(poly_shape)

    return outlying_area


def compute_overlap_area(polygon, obstacles, path, transit_flags, current_path_width):
    """ Computes the overlapping buffered line segments inside the polygon, excluding obstacles and transit lines.

    :param polygon: Polygon
    :param obstacles: List of Polygon instances representing obstacles
    :param path: List of points
    :param transit_flags: List of flags indicating if a segment is transit
    :param current_path_width: Float of chosen path width
    :return: A tuple (overlap_buffered_lines, overlap_union)
        - overlap_buffered_lines: List of overlapping buffered LineString objects.
        - overlap_union: Shapely Polygon representing the union of overlapping buffered segments.
    """
    current_path_width = current_path_width - 0.1  # Small reduction of path width for edge case overlap scenarios
    # Filter out transit segments
    active_path_segments = []
    for i in range(len(path) - 1):
        # Only exclude segments where both points are marked as transit
        if not (transit_flags[i] == "transit" and transit_flags[i + 1] == "transit"):
            active_path_segments.append(LineString([path[i], path[i + 1]]))

    if not active_path_segments:
        return [], ShapelyPolygon()  # No active segments, return empty list and polygon

    # Buffer the active path segments
    buffered_segments = [seg.buffer(current_path_width / 2.0) for seg in active_path_segments]

    # Convert the main polygon and obstacles to Shapely shapes
    poly_coords = [(v.x, v.y) for v in polygon.vertices]
    poly_shape = ShapelyPolygon(poly_coords)
    obstacles_shapes = unary_union([ShapelyPolygon([(v.x, v.y) for v in obstacle.vertices]) for obstacle in obstacles])

    # Subtract obstacles from each buffered segment
    effective_segments = [segment.difference(obstacles_shapes) for segment in buffered_segments]

    # Restrict buffered segments to the polygon region without obstacles
    effective_region = poly_shape.difference(obstacles_shapes)
    overlap_buffered_lines = []

    for i, seg1 in enumerate(effective_segments):
        for j, seg2 in enumerate(effective_segments):
            if i >= j:  # Avoid duplicate and self-comparison
                continue

            # Check for intersection between segments
            overlap = seg1.intersection(seg2)
            if not overlap.is_empty and overlap.area > 0:
                # Restrict overlap to the effective region
                restricted_overlap = overlap.intersection(effective_region)
                if not restricted_overlap.is_empty:
                    overlap_buffered_lines.append(restricted_overlap)

    # Combine all overlaps into a single polygon
    overlap_union = unary_union(overlap_buffered_lines).buffer(0) if overlap_buffered_lines else ShapelyPolygon()

    return overlap_union, overlap_buffered_lines


def compute_path_data(poly, path, transit_flags, current_path_width, obstacles, time):
    total_distance, path_distance, transit_distance = compute_distance(path, transit_flags)

    # Computing areas from path
    covered_area, coverage_percentage = compute_covered_area(poly, obstacles, path, transit_flags, current_path_width)
    outlier_area = compute_outlier_area(poly, path, transit_flags, current_path_width)
    overlap_area, overlap_buffered_lines = compute_overlap_area(poly, obstacles, path, transit_flags, current_path_width)

    #plot_shapely_polygon_simple(covered_area, title="Covered Area")
    #plot_shapely_polygon_simple(outlier_area, title="outlier_area")
    #plot_buffered_lines(overlap_buffered_lines, polygon=None, obstacles=None)

    # Computing turns in the path
    total_turns, hard_turns, medium_turns, soft_turns = compute_turns(path)

    print_data = False
    if print_data:
        print(f'Execution time: {time}')
        print(f'Total Distance: {total_distance}')
        print(f'Path Distance: {path_distance}')
        print(f'Transit Distance: {transit_distance}')
        print(f'Coverage percentage: {round(coverage_percentage, 2)}%')
        print(f'Covered area: {covered_area.area}')
        print(f'Outlier area: {outlier_area.area}')
        print(f'Overlap area: {overlap_area.area}')
        print(f'Total turns: {total_turns}')
        print(f'Hard turns (<45 or >180): {hard_turns}')
        print(f'Medium turns (45-90): {medium_turns}')
        print(f'Soft turns (90-180): {soft_turns}')

    if store_data:
        output_file = "coverage_results.txt"

        with open(output_file, 'w') as file:
            file.write(f"Execution time: {time}\n")
            file.write(f"Coverage percentage: {round(coverage_percentage, 2)}%\n")
            file.write(f"Covered area: {covered_area.area}\n")
            file.write(f"Wasted area: {outlier_area.area}\n")
            file.write(f"Overlap area: {overlap_area.area}\n\n")
            file.write(f"Total Distance: {total_distance}\n")
            file.write(f"Path Distance: {path_distance}\n")
            file.write(f"Transit Distance: {transit_distance}\n")
            file.write(f"Total turns: {total_turns}\n")
            file.write(f"Hard turns (<45 or >180): {hard_turns}\n")
            file.write(f"Medium turns (45-90): {medium_turns}\n")
            file.write(f"Soft turns (91-180): {soft_turns}\n")

    # Preparing to return the data
    result = {
        'type': 'coverage_statistics',
        'coverage_percentage': coverage_percentage,
        'covered_area': covered_area,
        'overlapped_area': overlap_area,
        'overlapped_lines': overlap_buffered_lines,
        'outlying_area': outlier_area,
        'total_distance': total_distance,
        'path_distance': path_distance,
        'transit_distance': transit_distance,
        'total_turns': total_turns,
        'hard_turns': hard_turns,
        'medium_turns': medium_turns,
        'soft_turns': soft_turns
    }
    return result