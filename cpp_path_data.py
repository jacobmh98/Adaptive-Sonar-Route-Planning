import numpy as np
from global_variables import *
from shapely.ops import unary_union
from shapely.geometry import Polygon as ShapelyPolygon, LineString


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
        if angle_deg < 45:
            hard_turns += 1
        elif 45 <= angle_deg < 90:
            medium_turns += 1
        else:
            soft_turns += 1

    total_turns = hard_turns + medium_turns + soft_turns

    return total_turns, hard_turns, medium_turns, soft_turns


def compute_covered_area(region, obstacles, path, transit_flags, current_path_width):
    """Compute the coverage of the path inside the region, excluding obstacle areas and transit lines.

    :param region: Main region polygon as an instance of the Polygon class
    :param obstacles: List of Polygon instances representing obstacles
    :param path: List of points (as tuples or arrays) representing the path
    :param transit_flags: List of flags indicating if a segment is transit
    :param current_path_width: Float of chosen path width
    :return: Tuple containing the covered area as a Shapely Polygon and the coverage percentage (float)
    """
    # Filter out transit segments and retain non-transit path segments
    active_path_segments = []
    for i in range(len(path) - 1):
        if transit_flags[i] != "transit" or transit_flags[i + 1] != "transit":
            active_path_segments.append(LineString([path[i], path[i + 1]]))

    if not active_path_segments:
        return ShapelyPolygon(), 0.0  # No active segments, return empty polygon and 0%

    # Buffer the active path segments
    buffered_path = unary_union([seg.buffer(current_path_width / 2.0) for seg in active_path_segments])

    # Convert the main region polygon to a Shapely Polygon
    region_coords = [(v.x, v.y) for v in region.vertices]
    region_shape = ShapelyPolygon(region_coords)

    # Subtract obstacle areas from the buffered path
    obstacles_shapes = [ShapelyPolygon([(v.x, v.y) for v in obstacle.vertices]) for obstacle in obstacles]
    obstacle_union = unary_union(obstacles_shapes)
    buffered_path_excluding_obstacles = buffered_path.difference(obstacle_union)

    # Subtract obstacle areas from the main region
    effective_region = region_shape.difference(obstacle_union)

    # Compute the area covered within the effective region (region - obstacles)
    covered_area = effective_region.intersection(buffered_path_excluding_obstacles)

    # Calculate the coverage percentage of the effective area
    coverage_percentage = (covered_area.area / effective_region.area) * 100 if effective_region.area > 0 else 0

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
    active_path_segments = [
        LineString([path[i], path[i + 1]])
        for i in range(len(path) - 1)
        if transit_flags[i] != "transit" and transit_flags[i + 1] != "transit"
    ]

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
    """ Computes the path overlap area inside the polygon, excluding obstacles and transit lines.

    :param polygon: Polygon
    :param obstacles: List of Polygon instances representing obstacles
    :param path: List of points
    :param transit_flags: List of flags indicating if a segment is transit
    :param current_path_width: Float of chosen path width
    :return overlap_area: Shapely Polygon of the path overlap inside the polygon
    """
    # Filter out transit segments
    active_path_segments = [
        LineString([path[i], path[i + 1]])
        for i in range(len(path) - 1)
        if transit_flags[i] != "transit" and transit_flags[i + 1] != "transit"
    ]

    if not active_path_segments:
        return ShapelyPolygon()  # No active segments, return empty polygon

    # Buffer the active path segments
    buffered_segments = [seg.buffer(current_path_width / 2.0) for seg in active_path_segments]

    # Convert the main polygon and obstacles to Shapely shapes
    poly_coords = [(v.x, v.y) for v in polygon.vertices]
    poly_shape = ShapelyPolygon(poly_coords)
    obstacles_shapes = unary_union([ShapelyPolygon([(v.x, v.y) for v in obstacle.vertices]) for obstacle in obstacles])

    # Subtract obstacles from each buffered segment directly
    effective_segments = [segment.difference(obstacles_shapes) for segment in buffered_segments]

    # Calculate overlaps only within the effective region (polygon without obstacles)
    effective_region = poly_shape.difference(obstacles_shapes)
    overlap_areas = []

    for i in range(len(effective_segments)):
        for j in range(i + 1, len(effective_segments)):
            # Find intersections (overlaps) between effective segments
            segment_overlap = effective_segments[i].intersection(effective_segments[j])
            if not segment_overlap.is_empty:
                # Restrict the overlap area to the actual intersection
                actual_overlap_within_polygon = segment_overlap.intersection(effective_region)
                if not actual_overlap_within_polygon.is_empty and actual_overlap_within_polygon.area > 0:
                    overlap_areas.append(actual_overlap_within_polygon)

    # Combine all detected overlaps into a single area
    if overlap_areas:
        overlap_area = unary_union(overlap_areas).buffer(0)  # Combine into a single polygon
    else:
        overlap_area = ShapelyPolygon()  # Empty if no overlaps

    return overlap_area



def compute_path_data(poly, path, transit_flags, current_path_width, obstacles, time):
    total_distance, path_distance, transit_distance = compute_distance(path, transit_flags)

    covered_area, coverage_percentage = compute_covered_area(poly, obstacles, path, transit_flags, current_path_width)
    outlier_area = compute_outlier_area(poly, path, transit_flags, current_path_width)
    overlap_area = compute_overlap_area(poly, obstacles, path, transit_flags, current_path_width)

    print(f'Execution time: {time}')
    print(f'Total Distance: {total_distance}')
    print(f'Path Distance: {path_distance}')
    print(f'Transit Distance: {transit_distance}')
    print(f'Coverage percentage: {round(coverage_percentage, 2)}%')
    print(f'Covered area: {covered_area.area}')
    print(f'Outlier area: {outlier_area.area}')
    print(f'Overlap area: {overlap_area.area}')

    # Computing turns in the path
    total_turns, hard_turns, medium_turns, soft_turns = compute_turns(path)

    print(f'Total turns: {total_turns}')
    print(f'Hard turns (<45): {hard_turns}')
    print(f'Medium turns (45-90): {medium_turns}')
    print(f'Soft turns (>90): {soft_turns}')

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
            file.write(f"Hard turns (<45): {hard_turns}\n")
            file.write(f"Medium turns (45-90): {medium_turns}\n")
            file.write(f"Soft turns (>90): {soft_turns}\n")

    # Preparing to return the data
    result = {
        'type': 'coverage_statistics',
        'coverage_percentage': coverage_percentage,
        'covered_area': covered_area,
        'overlapped_area': overlap_area,
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