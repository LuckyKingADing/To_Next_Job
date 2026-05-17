#!/usr/bin/env python3
"""
TCMSF初始化评估结果统计脚本

递归读取指定目录下所有的 init_eval_results.csv 文件，跳过 slice* 文件夹，
统计 total_init_time 的各项指标，并分析 reinit_count 情况。
"""

import os
import sys
import csv
import argparse
from pathlib import Path
from typing import List, Tuple, Set


def find_csv_files(root_dir: str) -> List[str]:
    """
    递归查找所有 init_eval_results.csv 文件，跳过 slice* 文件夹

    Args:
        root_dir: 搜索的根目录

    Returns:
        找到的CSV文件路径列表
    """
    csv_files = []
    root_path = Path(root_dir)

    for dirpath, dirnames, filenames in os.walk(root_path):
        # 过滤掉 slice* 开头的目录，不再遍历
        dirnames[:] = [d for d in dirnames if not d.startswith('slice')]

        # 检查是否有目标文件
        if 'init_eval_results.csv' in filenames:
            csv_files.append(os.path.join(dirpath, 'init_eval_results.csv'))

    return csv_files


def parse_csv_file(filepath: str) -> Tuple[List[dict], List[str]]:
    """
    解析单个CSV文件

    Args:
        filepath: CSV文件路径

    Returns:
        (数据行列表, 表头列表)
    """
    rows = []
    headers = []

    with open(filepath, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            # 跳过空行和注释行
            if not line or line.startswith('#'):
                # 尝试从注释中提取表头
                if 'slice_id,slice_start_ts' in line:
                    # 移除开头的 # 和空格
                    header_line = line.lstrip('#').strip()
                    headers = [h.strip() for h in header_line.split(',')]
                continue

            # 解析数据行
            values = line.split(',')
            if headers and len(values) == len(headers):
                row = {}
                for h, v in zip(headers, values):
                    # 尝试转换为数值类型或布尔值
                    try:
                        if v == 'True':
                            row[h] = True
                        elif v == 'False':
                            row[h] = False
                        elif '.' in v:
                            row[h] = float(v)
                        else:
                            row[h] = int(v)
                    except ValueError:
                        row[h] = v
                rows.append(row)

    return rows, headers


def filter_by_total_init_time(records: List[dict], tolerance: float = 1e-9) -> List[dict]:
    """
    根据 total_init_time 过滤重复记录
    如果两条记录的 total_init_time 差异在 tolerance 范围内，只保留一条

    Args:
        records: 原始记录列表
        tolerance: 时间差异容差

    Returns:
        过滤后的记录列表
    """
    if not records:
        return []

    # 按 total_init_time 排序
    sorted_records = sorted(records, key=lambda x: x.get('total_init_time', 0))
    filtered = [sorted_records[0]]

    for record in sorted_records[1:]:
        last_time = filtered[-1].get('total_init_time', 0)
        current_time = record.get('total_init_time', 0)

        # 如果差异大于容差，保留该记录
        if abs(current_time - last_time) > tolerance:
            filtered.append(record)

    return filtered


def write_output(output_file: str, output_lines: List[str]) -> None:
    """
    将输出内容写入文件并打印到屏幕

    Args:
        output_file: 输出文件路径
        output_lines: 输出内容列表
    """
    # 打印到屏幕
    for line in output_lines:
        print(line)

    # 写入文件
    with open(output_file, 'w', encoding='utf-8') as f:
        for line in output_lines:
            f.write(line + '\n')


def calculate_statistics(values: List[float]) -> dict:
    """
    计算统计指标

    Args:
        values: 数值列表

    Returns:
        包含各项统计指标的字典
    """
    if not values:
        return {
            'min': None,
            'max': None,
            'mean': None,
            'median': None,
            'p95': None,
            'count': 0
        }

    sorted_values = sorted(values)
    n = len(sorted_values)

    # 计算中位数
    if n % 2 == 0:
        median = (sorted_values[n // 2 - 1] + sorted_values[n // 2]) / 2
    else:
        median = sorted_values[n // 2]

    # 计算95%分位数
    p95_index = int(n * 0.95)
    p95_index = min(p95_index, n - 1)  # 确保索引不越界
    p95 = sorted_values[p95_index]

    return {
        'min': sorted_values[0],
        'max': sorted_values[-1],
        'mean': sum(values) / n,
        'median': median,
        'p95': p95,
        'count': n
    }


def format_record_line(record: dict) -> str:
    """
    格式化单条记录为输出字符串

    Args:
        record: 单条记录字典

    Returns:
        格式化后的字符串
    """
    slice_start_ts = record.get('slice_start_ts', 'N/A')
    if isinstance(slice_start_ts, float):
        time_str = f"{slice_start_ts:.6f}"
    else:
        time_str = str(slice_start_ts)

    return (f"slice_id={record.get('slice_id', 'N/A')}, "
            f"slice_start_ts={time_str}, "
            f"total_init_time={record.get('total_init_time', 'N/A'):.6f}秒, "
            f"reinit_count={record.get('reinit_count', 0)}, "
            f"is_success={record.get('is_success', 'N/A')}")


def main():
    parser = argparse.ArgumentParser(
        description='统计TCMSF初始化评估结果',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
    python init_eval_stats.py /mnt/d/dockers/rt/rtk_pvt/2026/results
    python init_eval_stats.py /mnt/d/dockers/rt/rtk_pvt/2026/results --tolerance 1e-8
        """
    )
    parser.add_argument('directory', help='要搜索的根目录路径')
    parser.add_argument('--tolerance', type=float, default=1e-9,
                        help='total_init_time去重的容差阈值 (默认: 1e-9)')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='显示详细输出')

    args = parser.parse_args()

    # 检查目录是否存在
    if not os.path.isdir(args.directory):
        print(f"错误: 目录 '{args.directory}' 不存在")
        sys.exit(1)

    # 输出文件路径
    output_file = os.path.join(args.directory, 'init_eval_stats_result.txt')

    # 收集输出内容
    output_lines = []

    output_lines.append(f"正在搜索目录: {args.directory}")
    output_lines.append(f"去重容差: {args.tolerance}")
    output_lines.append("-" * 60)

    # 查找所有CSV文件
    csv_files = find_csv_files(args.directory)
    output_lines.append(f"找到 {len(csv_files)} 个 init_eval_results.csv 文件")

    if not csv_files:
        output_lines.append("未找到任何CSV文件")
        write_output(output_file, output_lines)
        sys.exit(0)

    # 收集所有记录
    all_records = []
    records_with_reinit = []  # reinit_count > 0 的记录

    for filepath in csv_files:
        if args.verbose:
            output_lines.append(f"  处理: {filepath}")

        records, headers = parse_csv_file(filepath)

        # 在每个文件内进行过滤
        filtered_records = filter_by_total_init_time(records, args.tolerance)

        if args.verbose:
            output_lines.append(f"    原始记录: {len(records)}, 过滤后: {len(filtered_records)}")

        for record in filtered_records:
            record['_filepath'] = filepath  # 保存文件路径
            all_records.append(record)

            # 检查 reinit_count
            reinit_count = record.get('reinit_count', 0)
            if reinit_count > 0:
                records_with_reinit.append(record)

    output_lines.append(f"总共读取 {len(all_records)} 条记录 (过滤后)")

    # 过滤有效数据 (is_success == True 且 total_init_time > 0)
    valid_records = [r for r in all_records
                     if r.get('is_success') == True and r.get('total_init_time', -1) > 0]
    output_lines.append(f"有效记录 (is_success=True 且 total_init_time > 0): {len(valid_records)}")

    # 用于后续统计
    stats = None
    records_exceed_threshold = []  # total_init_time > 60 的记录

    if not valid_records:
        output_lines.append("没有有效的 total_init_time 数据")
    else:
        # 提取 total_init_time 值
        init_times = [r['total_init_time'] for r in valid_records]

        # 计算统计指标
        stats = calculate_statistics(init_times)

        # 筛选 total_init_time > 60 的记录
        threshold = 60.0
        for record in valid_records:
            if record.get('total_init_time', 0) > threshold:
                records_exceed_threshold.append(record)

        output_lines.append("")
        output_lines.append("=" * 60)
        output_lines.append("total_init_time 统计结果")
        output_lines.append("=" * 60)
        output_lines.append(f"  最小值: {stats['min']:.6f} 秒")
        output_lines.append(f"  最大值: {stats['max']:.6f} 秒")
        output_lines.append(f"  平均值: {stats['mean']:.6f} 秒")
        output_lines.append(f"  中位数: {stats['median']:.6f} 秒")
        output_lines.append(f"  95%分位数: {stats['p95']:.6f} 秒")
        output_lines.append(f"  有效记录数: {stats['count']}")

        # 输出 total_init_time > 60 的统计
        output_lines.append("")
        output_lines.append("=" * 60)
        output_lines.append("total_init_time > 60 统计")
        output_lines.append("=" * 60)
        output_lines.append(f"  阈值: {threshold:.1f} 秒")
        exceed_count = len(records_exceed_threshold)
        output_lines.append(f"  超过阈值的记录数: {exceed_count}")
        if len(valid_records) > 0:
            exceed_percentage = (exceed_count / len(valid_records)) * 100
            output_lines.append(f"  占有效记录比例: {exceed_percentage:.2f}%")

        # 列出每条超过阈值的记录
        if records_exceed_threshold:
            output_lines.append(f"  详细记录 ({exceed_count} 条):")
            # 按文件分组输出
            files_with_exceed = {}
            for record in records_exceed_threshold:
                filepath = record.get('_filepath', 'unknown')
                if filepath not in files_with_exceed:
                    files_with_exceed[filepath] = []
                files_with_exceed[filepath].append(record)

            for filepath, records in files_with_exceed.items():
                output_lines.append(f"    文件: {filepath}")
                for record in records:
                    output_lines.append(f"      {format_record_line(record)}")

    # reinit_count 统计
    output_lines.append("")
    output_lines.append("=" * 60)
    output_lines.append("reinit_count 统计")
    output_lines.append("=" * 60)

    total_records = len(all_records)
    reinit_count_sum = sum(r.get('reinit_count', 0) for r in all_records)
    reinit_record_count = len(records_with_reinit)

    if total_records > 0:
        reinit_percentage = (reinit_record_count / total_records) * 100
        output_lines.append(f"  reinit_count > 0 的记录数: {reinit_record_count}")
        output_lines.append(f"  总记录数: {total_records}")
        output_lines.append(f"  占比: {reinit_percentage:.2f}%")
        output_lines.append(f"  reinit_count 总和: {reinit_count_sum}")
    else:
        output_lines.append("  无记录")

    # 列出每条 reinit_count > 0 的记录
    if records_with_reinit:
        output_lines.append(f"  详细记录 ({reinit_record_count} 条):")
        # 按文件分组输出
        files_with_reinit = {}
        for record in records_with_reinit:
            filepath = record.get('_filepath', 'unknown')
            if filepath not in files_with_reinit:
                files_with_reinit[filepath] = []
            files_with_reinit[filepath].append(record)

        for filepath, records in files_with_reinit.items():
            output_lines.append(f"    文件: {filepath}")
            for record in records:
                output_lines.append(f"      {format_record_line(record)}")

    output_lines.append("")
    output_lines.append("=" * 60)
    output_lines.append("统计完成")
    output_lines.append("=" * 60)

    # 写入输出文件
    write_output(output_file, output_lines)
    print(f"统计结果已写入: {output_file}")


if __name__ == '__main__':
    main()