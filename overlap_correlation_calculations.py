import pandas as pd
import numpy as np

def read_coverage_data(file_path):
    """
    Reads the coverage data CSV file without headers and assigns column names.

    :param file_path: Path to the CSV file
    :return: Pandas DataFrame containing the data
    """
    try:
        # Read the CSV file without headers
        data = pd.read_csv(file_path, header=None)

        # Assign column names explicitly
        data.columns = ["path_width", "overlap_distance", "coverage_percent", "turns", "path_distance"]

        print("Data successfully loaded.")
        return data
    except FileNotFoundError:
        print(f"Error: The file at {file_path} was not found.")
        return None
    except Exception as e:
        print(f"An error occurred while reading the file: {e}")
        return None

def aggregate_metrics(data):
    """
    Aggregates the data by overlap distance to calculate average metrics.

    :param data: Pandas DataFrame with columns ["path_width", "overlap_distance", "coverage_percent", "turns", "path_distance"]
    :return: Aggregated DataFrame
    """
    # Group by overlap distance and calculate mean for all metrics
    aggregated = data.groupby("overlap_distance").agg(
        {
            "coverage_percent": "mean",
            "turns": "mean",
            "path_distance": "mean"
        }
    ).reset_index()

    # Normalize metrics for relative comparison
    aggregated["normalized_coverage"] = aggregated["coverage_percent"] / aggregated["coverage_percent"].max()
    aggregated["normalized_turns"] = aggregated["turns"] / aggregated["turns"].max()
    aggregated["normalized_distance"] = aggregated["path_distance"] / aggregated["path_distance"].max()

    return aggregated

def calculate_optimal_overlap(aggregated, w_c=50, w_t=10, w_d=1.0):
    """ Calculates an efficiency score to find the optimal overlap distance and normalizes it to 0-100.

    :param aggregated: Aggregated DataFrame with averaged metrics.
    :param w_c: Weight for coverage percentage.
    :param w_t: Weight for turns.
    :param w_d: Weight for path distance.
    :return: Best overlap distance and corresponding metrics.
    """
    # Composite score
    aggregated["efficiency_score"] = (
        w_c * aggregated["normalized_coverage"]
        - w_t * aggregated["normalized_turns"]
        - w_d * aggregated["normalized_distance"]
    )

    # Normalize the efficiency score to a 0-100 scale
    min_score = aggregated["efficiency_score"].min()
    max_score = aggregated["efficiency_score"].max()
    aggregated["normalized_efficiency_score"] = (
        (aggregated["efficiency_score"] - min_score) / (max_score - min_score) * 100
    )

    # Find the best overlap distance
    best_overlap = aggregated.loc[aggregated["normalized_efficiency_score"].idxmax()]
    return best_overlap, aggregated

def analyze_overlap(file_path, w_c=1000, w_t=10, w_d=1.0):
    """
    Executes the analysis pipeline: load data, aggregate metrics, and find the best overlap distance.

    :param file_path: Path to the CSV file.
    :param w_c: Weight for coverage percentage.
    :param w_t: Weight for turns.
    :param w_d: Weight for path distance.
    :return: Best overlap distance and corresponding metrics.
    """
    # Step 1: Read data
    data = read_coverage_data(file_path)
    if data is None:
        return None

    # Step 2: Aggregate metrics by overlap distance
    aggregated = aggregate_metrics(data)

    # Step 3: Find optimal overlap distance and normalize efficiency score
    best_overlap, aggregated = calculate_optimal_overlap(aggregated, w_c, w_t, w_d)

    print("\nAggregated Metrics by Overlap Distance:")
    print(aggregated.to_string(index=False))  # Print the entire DataFrame without truncation

    print("\nOptimal Overlap Distance Metrics:")
    print(best_overlap)

    return best_overlap, aggregated
