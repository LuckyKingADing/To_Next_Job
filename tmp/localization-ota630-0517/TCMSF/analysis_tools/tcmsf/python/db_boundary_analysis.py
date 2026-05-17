"""
DbData CSV 分析脚本

CSV 格式 (29列):
- veh2dr_translation: tx, ty, tz (列 0-2)
- boundary_0: pt0_x, pt0_y, pt1_x, pt1_y, ..., pt4_x, pt4_y (列 3-12, 左边界5个形点)
- boundary_1: pt0_x, pt0_y, pt1_x, pt1_y, ..., pt4_x, pt4_y (列 13-22, 右边界5个形点)
- projection: proj_x, proj_y, distance, distance_smoothed, segment_idx, is_valid (列 23-28)

可视化内容:
- 所有时刻的左边界形点 (boundary_0)
- 所有时刻的右边界形点 (boundary_1)
- 所有时刻的投影点 (projection.proj_point)
- 所有时刻的车辆位置 (veh2dr_translation)
"""

import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
import argparse


def load_db_boundary_csv(csv_path):
    """加载 DbData CSV 文件"""
    try:
        # CSV 无 header，29 列
        data = pd.read_csv(csv_path, header=None)
        print(f"Loaded {len(data)} records from {csv_path}")
        return data
    except Exception as e:
        print(f"Failed to load data: {e}")
        return None


def extract_boundary_points(data):
    """
    从 CSV 数据中提取边界点

    Returns:
        boundary_0_all: (N*5, 2) 左边界所有形点
        boundary_1_all: (N*5, 2) 右边界所有形点
        proj_points: (N, 2) 投影点
        veh_positions: (N, 2) 车辆位置 (tx, ty)
        distances: (N,) 投影距离
        distances_smoothed: (N,) 平滑后的投影距离
        is_valid: (N,) 投影有效性
    """
    N = len(data)

    # veh2dr_translation (列 0-2)
    veh_positions = data.iloc[:, 0:2].values  # tx, ty

    # boundary_0: 列 3-12, 5个点 × 2坐标 = 10列
    boundary_0_cols = data.iloc[:, 3:13].values
    boundary_0_all = boundary_0_cols.reshape(N * 5, 2)

    # boundary_1: 列 13-22, 5个点 × 2坐标 = 10列
    boundary_1_cols = data.iloc[:, 13:23].values
    boundary_1_all = boundary_1_cols.reshape(N * 5, 2)

    # projection: 列 23-28
    proj_x = data.iloc[:, 23].values
    proj_y = data.iloc[:, 24].values
    proj_points = np.column_stack([proj_x, proj_y])

    distances = data.iloc[:, 25].values  # distance
    distances_smoothed = data.iloc[:, 26].values  # distance_smoothed
    is_valid = data.iloc[:, 28].values.astype(bool)  # is_valid

    return (
        boundary_0_all,
        boundary_1_all,
        proj_points,
        veh_positions,
        distances,
        distances_smoothed,
        is_valid,
    )


def plot_boundary_scatter(
    boundary_0_all, boundary_1_all, proj_points, veh_positions, distances, is_valid
):
    """
    绘制边界点散点图

    Args:
        boundary_0_all: 所有时刻的左边界形点
        boundary_1_all: 所有时刻的右边界形点
        proj_points: 所有时刻的投影点
        veh_positions: 所有时刻的车辆位置
        distances: 投影距离
        is_valid: 投影有效性
    """
    fig, ax = plt.subplots(figsize=(14, 10))

    # 绘制左边界点 (蓝色)
    ax.scatter(
        boundary_0_all[:, 0],
        boundary_0_all[:, 1],
        c="blue",
        s=2,
        alpha=0.3,
        label="Left Boundary (boundary_0)",
    )

    # 绘制右边界点 (红色)
    ax.scatter(
        boundary_1_all[:, 0],
        boundary_1_all[:, 1],
        c="red",
        s=2,
        alpha=0.3,
        label="Right Boundary (boundary_1)",
    )

    # 绘制车辆位置 (绿色)
    ax.scatter(
        veh_positions[:, 0],
        veh_positions[:, 1],
        c="green",
        s=8,
        alpha=0.5,
        label="Vehicle Position (veh2dr_translation)",
    )

    # 绘制有效投影点 (根据距离正负着色)
    valid_proj = proj_points[is_valid]
    valid_dist = distances[is_valid]
    if len(valid_proj) > 0:
        # 根据距离颜色映射: 负值(左侧)蓝色, 正值(右侧)红色
        colors = np.where(valid_dist < 0, "cyan", "orange")
        ax.scatter(
            valid_proj[:, 0],
            valid_proj[:, 1],
            c=colors,
            s=15,
            alpha=0.7,
            marker="x",
            label="Projection Point (valid)",
        )

    # 绘制无效投影点 (灰色)
    invalid_proj = proj_points[~is_valid]
    if len(invalid_proj) > 0:
        ax.scatter(
            invalid_proj[:, 0],
            invalid_proj[:, 1],
            c="gray",
            s=10,
            alpha=0.3,
            marker="x",
            label="Projection Point (invalid)",
        )

    ax.set_xlabel("X (DR coordinate, m)")
    ax.set_ylabel("Y (DR coordinate, m)")
    ax.set_title("DriveBoundary Points Visualization (All Observations)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")

    # 打印统计信息
    print("\n=== Statistics ===")
    print(f"Total records: {len(veh_positions)}")
    print(f"Valid projections: {is_valid.sum()} / {len(is_valid)}")
    print(
        f"Vehicle position range: X=[{veh_positions[:, 0].min():.2f}, {veh_positions[:, 0].max():.2f}], "
        f"Y=[{veh_positions[:, 1].min():.2f}, {veh_positions[:, 1].max():.2f}]"
    )
    if len(valid_dist) > 0:
        print(
            f"Valid distance range: [{valid_dist.min():.3f}, {valid_dist.max():.3f}] m"
        )

    plt.tight_layout()
    plt.show()


def plot_distance_time_series(veh_positions, distances, distances_smoothed, is_valid):
    """
    绘制 distance 和 smoothed_distance 时序图

    Args:
        veh_positions: 车辆位置，用于计算累积里程作为时间轴
        distances: 投影距离 (原始量测值)
        distances_smoothed: 平滑后的投影距离
        is_valid: 投影有效性
    """
    # 计算累积里程作为时间轴
    N = len(veh_positions)
    cumulative_mileage = np.zeros(N)
    for i in range(1, N):
        delta = np.linalg.norm(veh_positions[i] - veh_positions[i - 1])
        cumulative_mileage[i] = cumulative_mileage[i - 1] + delta

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    # Plot 1: Distance vs Smoothed Distance
    ax1 = axes[0]
    ax1.plot(cumulative_mileage, distances, "b-", alpha=0.5, linewidth=1, label="Distance (raw)")
    ax1.plot(cumulative_mileage, distances_smoothed, "r-", linewidth=1.5, label="Distance Smoothed")
    ax1.scatter(
        cumulative_mileage[is_valid],
        distances[is_valid],
        c="green",
        s=5,
        alpha=0.5,
        label="Valid",
    )
    ax1.scatter(
        cumulative_mileage[~is_valid],
        distances[~is_valid],
        c="gray",
        s=5,
        alpha=0.5,
        label="Invalid",
    )
    ax1.axhline(y=0, color="black", linestyle="-", alpha=0.3)
    ax1.set_ylabel("Distance (m)")
    ax1.legend(loc="upper right")
    ax1.set_title("Distance vs Smoothed Distance over Mileage")
    ax1.grid(True, alpha=0.3)

    # Plot 2: Residual (Distance - Smoothed)
    ax2 = axes[1]
    residual = distances - distances_smoothed
    ax2.plot(cumulative_mileage, residual, "b-", linewidth=1, label="Residual")
    ax2.axhline(y=0, color="black", linestyle="-", alpha=0.3)
    ax2.set_ylabel("Residual (m)")
    ax2.set_xlabel("Cumulative Mileage (m)")
    ax2.legend(loc="upper right")
    ax2.set_title("Residual (Distance - Smoothed)")
    ax2.grid(True, alpha=0.3)

    # 打印统计信息
    print("\n=== Distance Statistics ===")
    print(f"Total mileage: {cumulative_mileage[-1]:.2f} m")
    print(f"Distance range: [{distances.min():.3f}, {distances.max():.3f}] m")
    print(f"Smoothed distance range: [{distances_smoothed.min():.3f}, {distances_smoothed.max():.3f}] m")
    print(f"Residual range: [{residual.min():.3f}, {residual.max():.3f}] m")
    print(f"Residual std: {residual.std():.4f} m")

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="DbData CSV Visualization (Scatter Mode)"
    )
    parser.add_argument(
        "--csv", type=str, default="data/tmp/db_debug_state.csv", help="CSV file path"
    )
    args = parser.parse_args()

    # 加载数据
    data = load_db_boundary_csv(args.csv)
    if data is None:
        return

    # 提取数据
    (boundary_0_all, boundary_1_all, proj_points, veh_positions,
     distances, distances_smoothed, is_valid) = extract_boundary_points(data)

    # 绘制散点图
    plot_boundary_scatter(
        boundary_0_all, boundary_1_all, proj_points, veh_positions, distances, is_valid
    )

    # 绘制 distance 时序图
    plot_distance_time_series(veh_positions, distances, distances_smoothed, is_valid)


if __name__ == "__main__":
    main()
