"""
GNSS卫星信息自洽性验证分析脚本

功能说明：
分析卫星航向、速度、轨迹追踪的自洽性验证结果。
使用10帧间隔的两帧进行逐帧连续判断，验证：
1. 卫星航向 vs 轨迹追踪航向
2. 位移矢量方向 vs 速度积分矢量方向
3. 速度方向 vs 卫星航向

验证条件（必须全部满足才能进行自洽性验证）：
1. 缓冲区足够（至少11帧）
2. 帧间隔合理（不超过2.0秒）
3. 位移 > 5米
4. 平均速度 > 5米/秒

CSV文件格式（9列）：
列1: 时间戳
列2: 自洽性评分 (0~1)
列3: 航向与轨迹差异(度)
列4: 位移与速度差异(度)
列5: 速度与航向差异(度)
列6: 帧间隔(秒)
列7: 位移距离(米)
列8: 平均速度(m/s)
列9: 是否符合验证条件 (0/1)
"""

import pandas as pd
import numpy as np
import math

from matplotlib import pyplot as plt


def load_data(csv_path="data/tmp/gnss_consistency_state.csv"):
    """加载GNSS自洽性验证数据"""
    try:
        data = np.array(pd.read_csv(csv_path), dtype=float)
        return data
    except Exception as e:
        print(f"加载数据失败: {e}")
        return None


def plot_consistency_overview(data):
    """绘制自洽性验证概览图"""
    if data is None or len(data) == 0:
        print("无数据可绘制")
        return

    plt.rcParams["legend.loc"] = "upper right"

    SUB_NUM = 5
    plt.subplots(SUB_NUM, 1, sharex=True, figsize=(12, 10))

    # 子图1: 自洽性评分
    plt.subplot(SUB_NUM, 1, 1)
    plt.plot(data[:, 0], data[:, 1], c="blue", linewidth=0.5)
    plt.ylim([0, 1])
    plt.legend(["consistency_score"])
    plt.ylabel("Score")

    # 子图2: 三对物理量差异(度)
    plt.subplot(SUB_NUM, 1, 2)
    plt.plot(data[:, 0], data[:, 2], c="red", linewidth=0.5)
    plt.plot(data[:, 0], data[:, 3], c="green", linewidth=0.5)
    plt.plot(data[:, 0], data[:, 4], c="orange", linewidth=0.5)
    plt.legend(["hdg_traj_diff", "disp_vel_diff", "vel_hdg_diff"])
    plt.ylabel("Diff (deg)")

    # 子图3: 帧间隔(秒)
    plt.subplot(SUB_NUM, 1, 3)
    plt.plot(data[:, 0], data[:, 5], c="purple", linewidth=0.5)
    # 标记理论帧间隔(1.0s)和最大帧间隔(2.0s)
    plt.axhline(y=1.0, color="gray", linestyle="--", linewidth=0.5)
    plt.axhline(y=2.0, color="red", linestyle="--", linewidth=0.5)
    plt.ylim([0, 3.0])
    plt.legend(["frame_interval", "theory=1.0s", "max=2.0s"])
    plt.ylabel("Interval (s)")

    # 子图4: 位移距离和平均速度
    plt.subplot(SUB_NUM, 1, 4)
    plt.plot(data[:, 0], data[:, 6], c="blue", linewidth=0.5)
    plt.plot(data[:, 0], data[:, 7], c="green", linewidth=0.5)
    # 标记可靠性阈值(5m和5m/s)
    plt.axhline(y=5.0, color="red", linestyle="--", linewidth=0.5)
    plt.legend(["displacement", "avg_speed", "threshold=5"])
    plt.ylabel("Distance/Speed")

    # 子图5: 是否符合验证条件
    plt.subplot(SUB_NUM, 1, 5)
    plt.scatter(data[:, 0], data[:, 8], s=0.5, c="blue")
    plt.ylim([0, 1.5])
    plt.legend(["is_valid"])
    plt.ylabel("Valid Flag")
    plt.xlabel("Time (s)")

    plt.tight_layout()
    plt.show()


def plot_diff_distribution(data):
    """绘制差异分布统计图"""
    if data is None or len(data) == 0:
        print("无数据可绘制")
        return

    # 筛选符合验证条件的数据点
    valid_mask = data[:, 8] == 1
    valid_data = data[valid_mask]

    if len(valid_data) == 0:
        print("无有效数据点可分析")
        return

    plt.figure(figsize=(12, 4))

    # 阈值和显示范围
    threshold_deg = 3.0
    xlim_range = threshold_deg * 4  # 显示范围为阈值的四倍

    # 子图1: 航向与轨迹差异分布
    plt.subplot(1, 3, 1)
    plt.hist(valid_data[:, 2], bins=300, color="red", alpha=0.7)
    plt.axvline(x=threshold_deg, color="black", linestyle="--", linewidth=1)
    plt.axvline(x=-threshold_deg, color="black", linestyle="--", linewidth=1)
    plt.xlim([-xlim_range, xlim_range])
    plt.xlabel("hdg_traj_diff (deg)")
    plt.ylabel("Count")
    plt.title("Heading vs Trajectory Diff")
    plt.legend([f"threshold={threshold_deg}deg"])

    # 子图2: 位移与速度差异分布
    plt.subplot(1, 3, 2)
    plt.hist(valid_data[:, 3], bins=300, color="green", alpha=0.7)
    plt.axvline(x=threshold_deg, color="black", linestyle="--", linewidth=1)
    plt.axvline(x=-threshold_deg, color="black", linestyle="--", linewidth=1)
    plt.xlim([-xlim_range, xlim_range])
    plt.xlabel("disp_vel_diff (deg)")
    plt.ylabel("Count")
    plt.title("Displacement vs Velocity Diff")
    plt.legend([f"threshold={threshold_deg}deg"])

    # 子图3: 速度与航向差异分布
    plt.subplot(1, 3, 3)
    plt.hist(valid_data[:, 4], bins=300, color="orange", alpha=0.7)
    plt.axvline(x=threshold_deg, color="black", linestyle="--", linewidth=1)
    plt.axvline(x=-threshold_deg, color="black", linestyle="--", linewidth=1)
    plt.xlim([-xlim_range, xlim_range])
    plt.xlabel("vel_hdg_diff (deg)")
    plt.ylabel("Count")
    plt.title("Velocity vs Heading Diff")
    plt.legend([f"threshold={threshold_deg}deg"])

    plt.tight_layout()
    plt.show()


def analyze_frame_interval(data):
    """分析帧间隔统计"""
    if data is None or len(data) == 0:
        print("无数据可分析")
        return

    print("=" * 50)
    print("帧间隔连续性分析")
    print("=" * 50)

    frame_intervals = data[:, 5]

    # 统计帧间隔
    print(f"理论单帧间隔: 0.1s")
    print(f"参考帧间隔: 10帧")
    print(f"两帧间理论时间间隔: 1.0s")
    print(f"最大允许两帧时间间隔: 2.0s")
    print(f"实际两帧时间间隔均值: {np.mean(frame_intervals):.4f}s")
    print(f"实际两帧时间间隔标准差: {np.std(frame_intervals):.4f}s")
    print(f"实际两帧时间间隔最大值: {np.max(frame_intervals):.4f}s")
    print(f"实际两帧时间间隔最小值: {np.min(frame_intervals):.4f}s")

    # 计算帧间隔合理的比例
    frame_valid_count = np.sum(frame_intervals <= 2.0)
    print(f"帧间隔合理的数量: {frame_valid_count} / {len(data)}")

    # 计算帧间隔超过阈值的比例
    invalid_count = np.sum(frame_intervals > 2.0)
    print(f"帧间隔超过2.0s的数量: {invalid_count} / {len(data)}")


def analyze_consistency_summary(data):
    """分析自洽性统计摘要"""
    if data is None or len(data) == 0:
        print("无数据可分析")
        return

    print("=" * 50)
    print("自洽性验证统计摘要")
    print("=" * 50)

    # 总体统计
    print(f"总数据点数: {len(data)}")

    # 符合验证条件的比例
    valid_ratio = np.sum(data[:, 8] == 1) / len(data) * 100
    print(f"符合验证条件比例: {valid_ratio:.2f}%")

    # 筛选符合验证条件的数据点进行深入分析
    valid_mask = data[:, 8] == 1
    valid_data = data[valid_mask]

    if len(valid_data) > 0:
        print("\n符合验证条件的数据点分析:")
        print(f"有效数据点数: {len(valid_data)}")

        # 自洽性评分统计
        scores = valid_data[:, 1]
        print(f"自洽性评分均值: {np.mean(scores):.4f}")
        print(f"自洽性评分标准差: {np.std(scores):.4f}")
        print(f"自洽性评分范围: [{np.min(scores):.4f}, {np.max(scores):.4f}]")

        # 高自洽性(>0.8)比例
        high_consistency_ratio = np.sum(scores > 0.8) / len(valid_data) * 100
        print(f"高自洽性(>0.8)比例: {high_consistency_ratio:.2f}%")

        # 低自洽性(<0.3)比例
        low_consistency_ratio = np.sum(scores < 0.3) / len(valid_data) * 100
        print(f"低自洽性(<0.3)比例: {low_consistency_ratio:.2f}%")

        # 各项差异均值
        print(f"\n三项差异均值(有效数据):")
        print(f"  航向与轨迹差异均值: {np.mean(valid_data[:, 2]):.2f}度")
        print(f"  位移与速度差异均值: {np.mean(valid_data[:, 3]):.2f}度")
        print(f"  速度与航向差异均值: {np.mean(valid_data[:, 4]):.2f}度")

        # 位移和速度统计
        print(f"\n位移和速度统计(有效数据):")
        print(f"  平均位移: {np.mean(valid_data[:, 6]):.2f}m")
        print(f"  平均速度: {np.mean(valid_data[:, 7]):.2f}m/s")


def plot_consistency_vs_speed(data):
    """绘制自洽性与速度的关系图"""
    if data is None or len(data) == 0:
        print("无数据可绘制")
        return

    # 筛选符合验证条件的数据点
    valid_mask = data[:, 8] == 1
    valid_data = data[valid_mask]

    if len(valid_data) == 0:
        print("无有效数据点可分析")
        return

    plt.figure(figsize=(10, 4))

    # 子图1: 自洽性评分 vs 平均速度
    plt.subplot(1, 2, 1)
    plt.scatter(valid_data[:, 7], valid_data[:, 1], s=2, c="blue", alpha=0.5)
    plt.xlabel("Average Speed (m/s)")
    plt.ylabel("Consistency Score")
    plt.title("Consistency vs Speed")

    # 子图2: 各项差异 vs 平均速度
    plt.subplot(1, 2, 2)
    plt.scatter(valid_data[:, 7], valid_data[:, 2], s=1, c="red", alpha=0.3)
    plt.scatter(valid_data[:, 7], valid_data[:, 3], s=1, c="green", alpha=0.3)
    plt.scatter(valid_data[:, 7], valid_data[:, 4], s=1, c="orange", alpha=0.3)
    plt.xlabel("Average Speed (m/s)")
    plt.ylabel("Difference (deg)")
    plt.title("Differences vs Speed")
    plt.legend(["hdg_traj", "disp_vel", "vel_hdg"])

    plt.tight_layout()
    plt.show()


def main():
    """主函数"""
    # 加载数据
    data = load_data()

    if data is None:
        return

    # 分析统计摘要
    analyze_consistency_summary(data)
    analyze_frame_interval(data)

    # 绘制图形
    plot_consistency_overview(data)
    plot_diff_distribution(data)
    plot_consistency_vs_speed(data)


if __name__ == "__main__":
    main()
