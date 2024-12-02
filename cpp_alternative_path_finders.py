import numpy as np

def compute_spiral_path(polygon, current_path_width, boundary):
    """
    Compute a continuous square spiral path for a given polygon, adjusting offsets dynamically
    and stopping correctly when there is no room to move in the next direction.

    :param polygon: The polygon object containing vertices and boundary.
    :param current_path_width: The distance between consecutive spiral lines.
    :param boundary: The polygon's boundary as [min_x, max_x, min_y, max_y].
    :return path: A 2D NumPy array representing the continuous spiral path.
    """
    min_x, max_x, min_y, max_y = boundary
    half_offset = current_path_width / 2  # First path uses half the offset
    full_offset = current_path_width      # Subsequent paths use the full offset
    path = []

    # Add first path (half offset)
    path.append([min_x, max_y - half_offset])  # Top-left
    path.append([max_x - half_offset, max_y - half_offset])  # Top-right
    path.append([max_x - half_offset, min_y + half_offset])  # Bottom-right
    path.append([min_x + half_offset, min_y + half_offset])  # Bottom-left

    # Adjust bounding box inward for the next loop
    min_x += half_offset
    max_x -= half_offset
    min_y += half_offset
    max_y -= half_offset

    # Spiral inward using the full offset
    while True:
        # Check if there is room to move right (to top-right corner)
        if max_x - min_x > full_offset:
            next_top_left = [min_x, max_y - full_offset]
            path.append(next_top_left)
        else:
            break

        # Check if there is room to move down (to bottom-right corner)
        if max_y - min_y > full_offset:
            next_top_right = [max_x - full_offset, max_y - full_offset]
            path.append(next_top_right)
        else:
            break

        # Check if there is room to move left (to bottom-left corner)
        if max_x - min_x > full_offset:
            next_bot_right = [max_x - full_offset, min_y + full_offset]
            path.append(next_bot_right)
        else:
            break

        # Check if there is room to move up (to top-left corner)
        if max_y - min_y > full_offset:
            next_bot_left = [min_x + full_offset, min_y + full_offset]
            path.append(next_bot_left)
        else:
            break

        # Update bounding box inward
        min_x += full_offset
        max_x -= full_offset
        min_y += full_offset
        max_y -= full_offset

    # Convert the path to a NumPy array before returning
    return np.array(path[:-2])
