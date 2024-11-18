import numpy as np
import plot_cpp
from global_variables import *
from shapely.ops import unary_union
from shapely.geometry import Polygon as ShapelyPolygon, LineString


def compute_distance(path, transit_flags):
    """ Function to compute the length of the path

    :param path: List of points
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
        is_transit = transit_flags[i] == "transit" or transit_flags[i + 1] == "transit"

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


def compute_covered_area_with_obstacles(region, obstacles, path, current_path_width):#, current_overlap_distance):
    """
    Compute the coverage of the path inside the region, excluding obstacle areas.

    :param region: Main region polygon as an instance of the Polygon class
    :param obstacles: List of Polygon instances representing obstacles
    :param path: List of points (as tuples or arrays) representing the path
    :return: Tuple containing the covered area as a Shapely Polygon and the coverage percentage (float)
    """
    # Convert path points to a LineString and buffer it to create a coverage area
    path = LineString(path)
    buffered_path = path.buffer((current_path_width) / 2.0)

    # Convert the main region polygon to a Shapely Polygon
    region_coords = [(v.x, v.y) for v in region.vertices]
    region_shape = ShapelyPolygon(region_coords)

    # Subtract obstacle areas from the main region
    obstacles_shapes = [ShapelyPolygon([(v.x, v.y) for v in obstacle.vertices]) for obstacle in obstacles]
    effective_region = region_shape.difference(unary_union(obstacles_shapes))

    # Compute the area covered within the effective region (region - obstacles)
    covered_area = effective_region.intersection(buffered_path)

    # Calculate the coverage percentage of the effective area
    coverage_percentage = (covered_area.area / effective_region.area) * 100

    return covered_area, coverage_percentage


def compute_outlier_area(polygon, path, current_path_width):
    """ Computes the path area outside the polygon

    :param polygon: Polygon
    :param path: List of points
    :return outlying_area: Shapely Polygon that lies outside the given polygon
    """
    path = LineString(path)
    buffered_path = path.buffer((current_path_width) / 2.0)

    # Convert your Polygon class to a Shapely Polygon
    poly_coords = [(v.x, v.y) for v in polygon.vertices]
    poly_shape = ShapelyPolygon(poly_coords)

    # Calculate the wasted area outside the polygon
    outlying_area = buffered_path.difference(poly_shape)

    return outlying_area


def compute_overlap_area(polygon, obstacles, path, current_path_width):
    """ Computes the path overlap area inside the polygon, excluding obstacles.

    :param polygon: Polygon
    :param obstacles: List of Polygon instances representing obstacles
    :param path: List of points
    :return overlap_area: Shapely Polygon of the path overlap inside the polygon
    """
    # Convert the path into buffered segments
    path_segments = [LineString([path[i], path[i + 1]]) for i in range(len(path) - 1)]
    buffered_segments = [seg.buffer((current_path_width) / 2.0) for seg in path_segments]

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
                # Restrict overlap area to within the effective region
                actual_overlap_within_polygon = segment_overlap.intersection(effective_region)
                if not actual_overlap_within_polygon.is_empty and actual_overlap_within_polygon.area > 0:
                    overlap_areas.append(actual_overlap_within_polygon)

    # Combine all detected overlaps into a single area
    if overlap_areas:
        overlap_area = unary_union(overlap_areas).buffer(0)
    else:
        overlap_area = ShapelyPolygon()  # Empty if no overlaps

    return overlap_area


def compute_path_data(poly, path, transit_flags, current_path_width, obstacles, time):
    total_distance, path_distance, transit_distance = compute_distance(path, transit_flags)

    covered_area, coverage_percentage = compute_covered_area_with_obstacles(poly, obstacles, path, current_path_width)
    outlier_area = compute_outlier_area(poly, path, current_path_width)
    overlap_area = compute_overlap_area(poly, obstacles, path, current_path_width)

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

    #plot_cpp.plot_coverage(poly, path, current_path_width, covered_area, outlier_area, overlap_area, obstacles, polygons)

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