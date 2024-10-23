import numpy as np
import coverage_plots
from global_variables import *
from shapely.ops import unary_union
from shapely.geometry import Polygon as ShapelyPolygon, LineString

def compute_total_distance(path):
    """ Function to compute the length of the path

    :param path: List of points
    :return: Total distance between all the points in the path
    """
    total_distance = 0.0
    # Loop through each consecutive pair of points and compute the distance
    for i in range(len(path) - 1):
        total_distance += np.linalg.norm(path[i + 1] - path[i])
    return total_distance

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
        unit_vector1 = vector1 / np.linalg.norm(vector1)
        unit_vector2 = vector2 / np.linalg.norm(vector2)

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


# Coverage functions
def compute_covered_area(polygon, path):
    """ Function to compute the coverage of the path inside the given polygon, and
        return this as an area and the percentage it covers the polygon

    :param polygon: Polygon
    :param path: List of points
    :return: Covered area as shapely Polygon area and a float percentage
    """
    path = LineString(path)
    buffered_path = path.buffer((path_width + overlap_distance)/ 2.0)

    # Convert your Polygon class to a Shapely Polygon
    poly_coords = [(v.x, v.y) for v in polygon.vertices]
    poly_shape = ShapelyPolygon(poly_coords)

    # Find the intersection of the buffered path and the polygon (covered area)
    covered_area = poly_shape.intersection(buffered_path)

    # Calculate the percentage of the polygon that is covered
    coverage_percentage = (covered_area.area / poly_shape.area) * 100

    return covered_area, coverage_percentage

def compute_outlier_area(polygon, path):
    """ Computes the path area outside the polygon

    :param polygon: Polygon
    :param path: List of points
    :return outlying_area: Shapely Polygon that lies outside the given polygon
    """
    path = LineString(path)
    buffered_path = path.buffer((path_width + overlap_distance) / 2.0)

    # Convert your Polygon class to a Shapely Polygon
    poly_coords = [(v.x, v.y) for v in polygon.vertices]
    poly_shape = ShapelyPolygon(poly_coords)

    # Calculate the wasted area outside the polygon
    outlying_area = buffered_path.difference(poly_shape)

    return outlying_area

def compute_overlap_area(polygon, path):
    """ Computes the path overlap area inside the polygon

    :param polygon: Polygon
    :param path: List of points
    :return overlap_area: Shapely Polygon of the path overlap inside the polygon
    """
    # Convert the list of path points to a LineString object
    path_segments = [LineString([path[i], path[i + 1]]) for i in range(len(path) - 1)]
    buffered_segments = [seg.buffer((path_width + overlap_distance) / 2.0) for seg in path_segments]

    # Convert your Polygon class to a Shapely Polygon
    poly_coords = [(v.x, v.y) for v in polygon.vertices]
    poly_shape = ShapelyPolygon(poly_coords)

    overlap_areas = []
    for i in range(len(buffered_segments)):
        for j in range(i + 1, len(buffered_segments)):
            # Detect overlaps between buffered segments
            segment_overlap = buffered_segments[i].intersection(buffered_segments[j])
            if not segment_overlap.is_empty:
                # Only count overlap areas that are strictly within the polygon
                actual_overlap_within_polygon = segment_overlap.intersection(poly_shape)
                if not actual_overlap_within_polygon.is_empty and actual_overlap_within_polygon.area > 0:
                    overlap_areas.append(actual_overlap_within_polygon)

    # Combine all detected overlaps into a single area
    if overlap_areas:
        overlap_area = unary_union(overlap_areas).buffer(0)
    else:
        overlap_area = ShapelyPolygon()  # Empty if no overlaps

    return overlap_area

def compute_path_data(poly, path, time):
    covered_area, coverage_percentage = compute_covered_area(poly, path)
    outlier_area = compute_outlier_area(poly, path)
    overlap_area = compute_overlap_area(poly, path)

    print(f'Execution time: {time}')
    print(f'Coverage percentage: {round(coverage_percentage, 2)}%')
    print(f'Covered area: {covered_area.area}')
    print(f'Outlier area: {outlier_area.area}')
    print(f'Overlap area: {overlap_area.area}')
    coverage_plots.visualize_coverage_wasted_and_overlap(poly, path, covered_area, outlier_area, overlap_area)
    print()

    # Computing turns in the path
    distance = compute_total_distance(path)
    total_turns, hard_turns, medium_turns, soft_turns = compute_turns(path)

    print(f'Distance: {distance}')
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
            file.write(f"Distance: {distance}\n")
            file.write(f"Total turns: {total_turns}\n")
            file.write(f"Hard turns (<45): {hard_turns}\n")
            file.write(f"Medium turns (45-90): {medium_turns}\n")
            file.write(f"Soft turns (>90): {soft_turns}\n")
