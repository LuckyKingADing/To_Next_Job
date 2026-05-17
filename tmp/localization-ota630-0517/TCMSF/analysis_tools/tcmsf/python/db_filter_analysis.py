import pandas as pd
import numpy as np
from matplotlib import pyplot as plt


def scope_by_time(left_t, right_t, data, timestamp_col=0):
    """Filter data by time range"""
    timestamp = data[:, timestamp_col]
    condition_ = (timestamp > left_t) & (timestamp < right_t)
    return data[condition_, :]


def load_db_data(csv_path="data/tmp/db_debug_state.csv"):
    """Load DB filter debug data"""
    try:
        data = np.array(pd.read_csv(csv_path, header=None))
        print(f"Loaded data successfully, {data.shape[0]} records")
        return data
    except Exception as e:
        print(f"Failed to load data: {e}")
        return None


def plot_db_filter(data, left_t=None, right_t=None):
    """Plot DB filter results"""
    if data is None or data.shape[0] == 0:
        print("No valid data")
        return

    # Column definitions
    # col 0: timestamp
    # col 1: last_measurement (raw measurement)
    # col 2: filtered_distance (filtered value)
    # col 3: variance (P)
    # col 4: delta_mileage (displacement)

    timestamp = data[:, 0]
    measurement = data[:, 1]
    filtered = data[:, 2]
    variance = data[:, 3]

    # Time range filtering
    if left_t is None:
        left_t = timestamp[0]
    if right_t is None:
        right_t = timestamp[-1]

    data_scope = scope_by_time(left_t, right_t, data)
    if data_scope.shape[0] == 0:
        print("No data after filtering")
        return

    timestamp = data_scope[:, 0]
    measurement = data_scope[:, 1]
    filtered = data_scope[:, 2]
    variance = data_scope[:, 3]

    # Print statistics
    print(f"\n=== Statistics ===")
    print(f"Time range: {timestamp[0]:.2f} - {timestamp[-1]:.2f} s")
    print(f"Duration: {timestamp[-1] - timestamp[0]:.2f} s")
    print(f"Measurement range: {measurement.min():.4f} - {measurement.max():.4f} m")
    print(f"Filtered range: {filtered.min():.4f} - {filtered.max():.4f} m")

    # Calculate residual and std
    residual = measurement - filtered
    std_dev = np.sqrt(variance)

    # Create figure with 2 subplots
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Plot 1: Measurement vs Filtered
    ax1 = axes[0]
    ax1.plot(timestamp, measurement, "b-", alpha=0.5, label="Raw Measurement")
    ax1.plot(timestamp, filtered, "r-", linewidth=1.5, label="Filtered Value")
    ax1.set_ylabel("Distance (m)")
    ax1.legend(loc="upper right")
    ax1.set_title("DB Filter: Measurement vs Filtered")
    ax1.grid(True, alpha=0.3)

    # Plot 2: Residual with sigma bounds
    ax2 = axes[1]
    ax2.plot(timestamp, residual, "b-", linewidth=1.5, label="Residual")
    ax2.fill_between(
        timestamp, -std_dev, std_dev, color="green", alpha=0.3, label="±1 sigma"
    )
    ax2.fill_between(
        timestamp,
        -2 * std_dev,
        2 * std_dev,
        color="green",
        alpha=0.15,
        label="±2 sigma",
    )
    ax2.axhline(y=0, color="black", linestyle="-", alpha=0.5)
    ax2.set_ylabel("Residual (m)")
    ax2.set_xlabel("Timestamp (s)")
    ax2.legend(loc="upper right")
    ax2.set_title("Residual vs Sigma")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def main():
    # Load data
    data = load_db_data()

    if data is None:
        return

    # Print time range info
    timestamp = data[:, 0]
    print(f"\nData time range: {timestamp[0]:.2f} - {timestamp[-1]:.2f} s")
    print(f"Total duration: {timestamp[-1] - timestamp[0]:.2f} s")

    LEFT_T = timestamp[0]
    RIGHT_T = timestamp[-1]

    # Plot filter results
    plot_db_filter(data, LEFT_T, RIGHT_T)


if __name__ == "__main__":
    main()
