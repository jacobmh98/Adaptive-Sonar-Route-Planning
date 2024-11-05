import numpy as np

def compute_total_distance(path):
    total_distance = 0.0
    # Loop through each consecutive pair of points and compute the distance
    for i in range(len(path) - 1):
        total_distance += np.linalg.norm(path[i + 1] - path[i])
    return total_distance


def calculate_turns_and_classify(path):
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