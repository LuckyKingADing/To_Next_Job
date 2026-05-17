import pandas as pd
import numpy as np
from matplotlib import pyplot as plt


def scope_by_time(left_t, right_t, data, timestamp_col=0):
    """Filter data by time range"""
    timestamp = data[:, timestamp_col]
    condition_ = (timestamp > left_t) & (timestamp < right_t)
    return data[condition_, :]


def load_sdmap_data(csv_path="data/tmp/sdmap_state.csv"):
    """Load sdmap_state debug data"""
    try:
        data = np.array(pd.read_csv(csv_path, header=None))
        print(f"Loaded data successfully, {data.shape[0]} records, {data.shape[1]} columns")
        return data
    except Exception as e:
        print(f"Failed to load data: {e}")
        return None


def plot_lat_values(data, left_t=None, right_t=None):
    """Plot VehMap_lat, VehDb_lat, MapDb_lat"""
    if data is None or data.shape[0] == 0:
        print("No valid data")
        return

    # Column definitions (last 3 columns):
    # col -3: VehMap_lat_ (vehicle to map projection lateral distance)
    # col -2: VehDb_lat_ (vehicle to DB centerline lateral distance)
    # col -1: MapDb_lat_ (map projection to DB centerline lateral distance)

    timestamp = data[:, 0]
    veh_map_lat = data[:, -3]
    veh_db_lat = data[:, -2]
    map_db_lat = data[:, -1]

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
    veh_map_lat = data_scope[:, -3]
    veh_db_lat = data_scope[:, -2]
    map_db_lat = data_scope[:, -1]

    # Print statistics
    print(f"\n=== Statistics ===")
    print(f"Time range: {timestamp[0]:.2f} - {timestamp[-1]:.2f} s")
    print(f"Duration: {timestamp[-1] - timestamp[0]:.2f} s")
    print(f"VehMap_lat range: {veh_map_lat.min():.4f} - {veh_map_lat.max():.4f} m")
    print(f"VehDb_lat range: {veh_db_lat.min():.4f} - {veh_db_lat.max():.4f} m")
    print(f"MapDb_lat range: {map_db_lat.min():.4f} - {map_db_lat.max():.4f} m")

    # Verify relation: MapDb_lat ≈ -VehMap_lat + VehDb_lat
    computed_map_db = -veh_map_lat + veh_db_lat
    error = map_db_lat - computed_map_db
    print(f"\nRelation verification: MapDb_lat = -VehMap_lat + VehDb_lat")
    print(f"Max error: {np.abs(error).max():.6f} m")

    # Create figure
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Plot 1: Three lateral distances
    ax1 = axes[0]
    ax1.plot(timestamp, veh_map_lat, "b-", linewidth=1.5, label="VehMap_lat (Vehicle to Map)")
    ax1.plot(timestamp, veh_db_lat, "r-", linewidth=1.5, label="VehDb_lat (Vehicle to DB)")
    ax1.plot(timestamp, map_db_lat, "g-", linewidth=1.5, label="MapDb_lat (Map to DB)")
    ax1.axhline(y=0, color="black", linestyle="-", alpha=0.5)
    ax1.set_ylabel("Lateral Distance (m)")
    ax1.legend(loc="upper right")
    ax1.set_title("SDMap State: Lateral Distances")
    ax1.grid(True, alpha=0.3)

    # Plot 2: Relation verification error
    ax2 = axes[1]
    ax2.plot(timestamp, error, "m-", linewidth=1.5, label="Error")
    ax2.axhline(y=0, color="black", linestyle="-", alpha=0.5)
    ax2.set_ylabel("Error (m)")
    ax2.set_xlabel("Timestamp (s)")
    ax2.legend(loc="upper right")
    ax2.set_title("Relation Verification Error: MapDb_lat - (-VehMap_lat + VehDb_lat)")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def main():
    data = load_sdmap_data()

    if data is None:
        return

    timestamp = data[:, 0]
    print(f"\nData time range: {timestamp[0]:.2f} - {timestamp[-1]:.2f} s")
    print(f"Total duration: {timestamp[-1] - timestamp[0]:.2f} s")

    LEFT_T = timestamp[0]
    RIGHT_T = timestamp[-1]

    plot_lat_values(data, LEFT_T, RIGHT_T)


if __name__ == "__main__":
    main()