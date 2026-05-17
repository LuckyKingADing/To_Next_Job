#!/usr/bin/env python3
"""
批量收集和可视化评测结果

功能：
1. 将所有子数据集results目录下的关键文件复制到All_Results目录，
   并按 {子数据集名}_{原文件名} 格式重命名
2. 对所有子数据集的精度文件进行聚合统计，生成聚合结果文件
3. 对All_Results目录下所有精度文件（包括聚合文件和子数据集文件）进行可视化
   输出到 All_Results/vis/{prefix}/ 目录，显示 pvtlc, rtklc, GNSS 三种方案
   文件名带前缀，如: {prefix}_CEP95_Horizontal.png, {prefix}_summary_table.txt

使用方法：
    # 完整流程（收集 + 聚合 + 可视化）
    python collect_and_visualize.py --output_dir "/path/to/evaluation"

    # 只可视化已有的All_Results目录
    python collect_and_visualize.py --all_results_dir "/path/to/All_Results"

    # 预览模式
    python collect_and_visualize.py --dry-run

    # 只生成汇总表（跳过条形图）
    python collect_and_visualize.py --skip-bar

    # 只可视化特定方案
    python collect_and_visualize.py --schemes lc tc
"""

import os
import re
import shutil
import argparse
from pathlib import Path
from datetime import datetime

# 导入聚合精度统计模块
try:
    from pvt_rtk_lc_evaluation.aggregate_precision_statistics import (
        parse_precision_file,
        parse_velocity_file,
        aggregate_statistics,
        aggregate_velocity_statistics,
        format_output,
        format_velocity_output,
    )
    AGGREGATE_MODULE_AVAILABLE = True
except ImportError:
    AGGREGATE_MODULE_AVAILABLE = False
    print("警告: 无法导入聚合精度统计模块")

# 导入可视化模块
try:
    from pvt_rtk_lc_evaluation.visualize_precision import (
        parse_precision_file as vis_parse_precision_file,
        plot_all_metrics,
        write_text_summary,
        setup_chinese_font,
        MetricsData,
        SchemeData,
        SCENE_ORDER,
    )
    VISUALIZE_MODULE_AVAILABLE = True
except ImportError as e:
    VISUALIZE_MODULE_AVAILABLE = False
    print(f"警告: 无法导入可视化模块: {e}")

# matplotlib 检查
try:
    import matplotlib.pyplot as plt
    import matplotlib
    matplotlib.use('Agg')
    import numpy as np
    VISUALIZE_LIB_AVAILABLE = True
except ImportError:
    VISUALIZE_LIB_AVAILABLE = False
    print("警告: 未安装 matplotlib/numpy，将跳过可视化")


# 需要收集的关键文件列表
KEY_FILES = [
    "All_with_gnss.png",
    "All_without_gnss.png",
    "position_precision.txt",
    "velocity_precision.txt",
]

# clip子目录下的文件
CLIP_FILES = [
    "gnss_error_and_std_overview.png",
    "gnss_error_only_no_std.png",
]

# 需要从子数据集vis目录收集到All_Results根目录的关键PNG文件
KEY_VIS_PNGS = [
    "CEP95_Horizontal.png",
    "Max_Horizontal.png",
    "RMS_Horizontal.png",
]


def find_all_subdatasets(output_dir: Path):
    """扫描输出目录，找到所有子数据集文件夹"""
    subdatasets = []

    for date_folder in output_dir.iterdir():
        if not date_folder.is_dir():
            continue
        if date_folder.name == "All_Results":
            continue

        for subdataset in date_folder.iterdir():
            if not subdataset.is_dir():
                continue

            results_dir = subdataset / "results"
            if results_dir.exists() and results_dir.is_dir():
                subdatasets.append((results_dir, subdataset.name))

    return subdatasets


def collect_files(output_dir: Path, dry_run: bool = False):
    """收集所有关键文件到All_Results目录"""
    all_results_dir = output_dir / "All_Results"

    subdatasets = find_all_subdatasets(output_dir)

    if not subdatasets:
        print(f"未找到任何包含results目录的子数据集文件夹")
        return 0, 0, all_results_dir

    print(f"找到 {len(subdatasets)} 个子数据集文件夹")
    print("=" * 60)

    if not dry_run:
        all_results_dir.mkdir(parents=True, exist_ok=True)
        print(f"创建目标目录: {all_results_dir}")
    else:
        print(f"[DRY RUN] 将创建目标目录: {all_results_dir}")

    print("=" * 60)

    copied_count = 0
    missing_count = 0

    for results_dir, subdataset_name in subdatasets:
        print(f"\n处理子数据集: {subdataset_name}")
        print(f"  results路径: {results_dir}")

        for filename in KEY_FILES:
            src_file = results_dir / filename
            if src_file.exists():
                new_filename = f"{subdataset_name}_{filename}"
                dst_file = all_results_dir / new_filename

                if dry_run:
                    print(f"  [DRY RUN] 将复制: {filename} -> {new_filename}")
                else:
                    shutil.copy2(src_file, dst_file)
                    print(f"  复制: {filename} -> {new_filename}")
                copied_count += 1
            else:
                print(f"  缺失: {filename}")
                missing_count += 1

        clip_dir = results_dir / "clip"
        if clip_dir.exists() and clip_dir.is_dir():
            for filename in CLIP_FILES:
                src_file = clip_dir / filename
                if src_file.exists():
                    new_filename = f"{subdataset_name}_clip_{filename}"
                    dst_file = all_results_dir / new_filename

                    if dry_run:
                        print(f"  [DRY RUN] 将复制: clip/{filename} -> {new_filename}")
                    else:
                        shutil.copy2(src_file, dst_file)
                        print(f"  复制: clip/{filename} -> {new_filename}")
                    copied_count += 1
                else:
                    print(f"  缺失: clip/{filename}")
                    missing_count += 1
        else:
            print(f"  clip目录不存在")

    print("\n" + "=" * 60)
    print(f"文件收集完成!")
    print(f"  成功复制: {copied_count} 个文件")
    print(f"  缺失文件: {missing_count} 个")
    print(f"  目标目录: {all_results_dir}")
    if dry_run:
        print(f"\n[DRY RUN] 以上为预览，未实际执行复制操作")

    return copied_count, missing_count, all_results_dir


def aggregate_precision_from_subdatasets(output_dir: Path, all_results_dir: Path, dry_run: bool = False):
    """从所有子数据集的精度文件进行聚合统计"""
    if dry_run:
        print("\n" + "=" * 60)
        print("[DRY RUN] 将执行精度聚合统计")
        print("=" * 60)
        return None

    print("\n" + "=" * 60)
    print("开始精度聚合统计")
    print("=" * 60)

    if not AGGREGATE_MODULE_AVAILABLE:
        print("警告: 无法导入聚合精度统计模块")
        return None

    subdatasets = find_all_subdatasets(output_dir)

    if not subdatasets:
        print("未找到子数据集，跳过聚合统计")
        return None

    position_files = []
    velocity_files = []

    for results_dir, _ in subdatasets:
        position_file = results_dir / "position_precision.txt"
        velocity_file = results_dir / "velocity_precision.txt"

        if position_file.exists():
            position_files.append(str(position_file))
        if velocity_file.exists():
            velocity_files.append(str(velocity_file))

    print(f"找到 {len(position_files)} 个位置精度文件")
    print(f"找到 {len(velocity_files)} 个速度精度文件")

    aggregated_position_file = None

    # 处理位置精度聚合
    if position_files:
        print("\n正在聚合位置精度统计...")

        all_files_stats = []
        for filepath in position_files:
            try:
                stats = parse_precision_file(filepath)
                all_files_stats.append(stats)
                print(f"  已解析: {Path(filepath).parent.parent.name}")
            except Exception as e:
                print(f"  解析失败 {filepath}: {e}")

        if all_files_stats:
            aggregated_stats = aggregate_statistics(all_files_stats, skip_zero_scenes=True)
            output_text = format_output(aggregated_stats)

            aggregated_position_file = all_results_dir / "1_position_precision_aggregated.txt"
            with open(aggregated_position_file, 'w', encoding='utf-8') as f:
                f.write(output_text)

            print(f"\n✓ 位置精度聚合结果已保存到: {aggregated_position_file}")

    # 处理速度精度聚合
    if velocity_files:
        print("\n正在聚合速度精度统计...")

        all_velocity_stats = []
        for filepath in velocity_files:
            try:
                stats = parse_velocity_file(filepath)
                all_velocity_stats.append(stats)
                print(f"  已解析: {Path(filepath).parent.parent.name}")
            except Exception as e:
                print(f"  解析失败 {filepath}: {e}")

        if all_velocity_stats:
            aggregated_velocity_stats = aggregate_velocity_statistics(all_velocity_stats, skip_zero_scenes=True)
            velocity_output = format_velocity_output(aggregated_velocity_stats)

            output_file = all_results_dir / "1_velocity_precision_aggregated.txt"
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(velocity_output)

            print(f"\n✓ 速度精度聚合结果已保存到: {output_file}")

    print("\n" + "=" * 60)
    print("精度聚合统计完成!")
    print("=" * 60)

    return aggregated_position_file


def parse_precision_file_with_fallback(filepath: str):
    """
    解析精度文件，对于缺少版本标识的聚合文件做特殊处理
    确保能正确识别 pvtlc, rtklc, GNSS 三种方案
    """
    # 先尝试标准解析
    try:
        dataset_name, schemes = vis_parse_precision_file(filepath)
        if len(schemes) >= 2:  # 如果解析到多个方案，直接返回
            return dataset_name, schemes
    except Exception:
        pass

    # 如果标准解析失败或只解析到GNSS，尝试手动解析聚合文件
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    lines = content.split('\n')
    result_schemes = []

    # 查找所有方案块的版本号（聚合文件格式如 "pvtlc_xxx Statistics"）
    version_patterns = [
        (r'^pvtlc_[\w]+ Statistics', 'lc'),
        (r'^rtklc_[\w]+ Statistics', 'tc'),
        (r'^GNSS Statistics', 'gnss'),
    ]

    section_headers = []
    for i, line in enumerate(lines):
        stripped = line.strip()
        for pattern, scheme_key in version_patterns:
            if re.match(pattern, stripped):
                # 提取版本号
                match = re.match(r'^([\w]+) Statistics', stripped)
                if match:
                    version = match.group(1)
                else:
                    version = 'GNSS'
                section_headers.append((i, version, scheme_key))
                break

    # 解析每个方案块
    for idx, (start_line, version, scheme_key) in enumerate(section_headers):
        end_line = len(lines)
        if idx + 1 < len(section_headers):
            end_line = section_headers[idx + 1][0]

        block_lines = lines[start_line + 1:end_line]
        sd = SchemeData(scheme_key, version)

        skip = True
        has_vertical = 'V-rms' in ''.join(block_lines) or 'V-CEP95' in ''.join(block_lines)
        valid_metrics = MetricsData.METRICS_FULL[:16] if has_vertical else MetricsData.METRICS_FULL[:12]
        expected = 17 if has_vertical else 13

        for line in block_lines:
            stripped = line.strip()
            if not stripped:
                continue
            if set(stripped) == {'-'}:
                skip = False
                continue
            if skip:
                continue

            nums = re.findall(r'[\d.]+', stripped)
            if len(nums) < expected:
                continue

            # 提取场景名
            m = re.match(r'^(.+?)\s*[\d.]+\s+', stripped)
            if not m:
                continue
            scene = m.group(1).strip()

            values = [float(n) for n in nums]
            odom = values[0]
            data = MetricsData(odom)
            for i_m, metric in enumerate(valid_metrics):
                if i_m + 1 < len(values):
                    data.metrics[metric] = values[i_m + 1]

            sd.add_scene(scene, data)

        if sd.scenes:
            result_schemes.append(sd)

    dataset_name = Path(filepath).parent.name
    return dataset_name, result_schemes


def visualize_single_file(precision_file: Path, vis_dir: Path, prefix: str,
                           version_label: str, schemes_filter: list = None,
                           scenes_filter: list = None, visualize_types: list = None):
    """
    对单个精度文件进行可视化，确保显示所有方案（pvtlc, rtklc, GNSS）
    生成的文件名带前缀，如: {prefix}_CEP95_Horizontal.png
    """
    if not precision_file.exists():
        print(f"  文件不存在: {precision_file}")
        return 0

    print(f"\n处理: {precision_file.name}")

    # 使用带fallback的解析函数，确保聚合文件也能正确解析
    try:
        dataset_name, schemes = parse_precision_file_with_fallback(str(precision_file))
        print(f"  数据集: {dataset_name}")
        print(f"  解析到 {len(schemes)} 个方案:")
        for s in schemes:
            print(f"    - {s.name}: {s.version} ({len(s.scenes)} 个场景)")
    except Exception as e:
        print(f"  解析失败: {e}")
        import traceback
        traceback.print_exc()
        return 0

    if not schemes:
        print("  未解析到任何方案数据，跳过")
        return 0

    # 按配置过滤方案
    if schemes_filter:
        filtered_schemes = [s for s in schemes if s.name in schemes_filter]
    else:
        filtered_schemes = schemes

    print(f"  可视化方案: {[s.name for s in filtered_schemes]}")

    if not filtered_schemes:
        print("  过滤后无方案数据，跳过")
        return 0

    # 确定场景列表
    all_scenes = set()
    for s in filtered_schemes:
        all_scenes.update(s.scenes.keys())

    if scenes_filter:
        scenes = [sc for sc in SCENE_ORDER if sc in scenes_filter and sc in all_scenes]
    else:
        scenes = [sc for sc in SCENE_ORDER if sc in all_scenes]
    print(f"  场景列表: {scenes}")

    if not scenes:
        print("  无有效场景数据，跳过")
        return 0

    # 创建子目录
    sub_vis_dir = vis_dir / prefix
    sub_vis_dir.mkdir(parents=True, exist_ok=True)

    count = 0

    # 生成条形图
    if 'bar' in visualize_types:
        print("  生成条形图...")
        try:
            plot_all_metrics(filtered_schemes, scenes, sub_vis_dir, f"{version_label}_{prefix}")
            count += len(MetricsData.METRICS_FULL)

            # 重命名PNG文件，加上前缀
            for png_file in sub_vis_dir.glob("*.png"):
                old_name = png_file.name
                new_name = f"{prefix}_{old_name}"
                new_file = sub_vis_dir / new_name
                png_file.rename(new_file)

            print(f"    ✓ 生成 {len(MetricsData.METRICS_FULL)} 张条形图（已加前缀）")
        except Exception as e:
            print(f"    生成条形图失败: {e}")

    # 生成文本汇总表（带前缀命名）
    if 'table' in visualize_types:
        print("  生成文本汇总表...")
        try:
            write_text_summary(filtered_schemes, scenes, sub_vis_dir, f"{version_label}_{prefix}")
            # 重命名汇总表文件，带上前缀
            old_file = sub_vis_dir / "summary_table.txt"
            new_file = sub_vis_dir / f"{prefix}_summary_table.txt"
            if old_file.exists():
                old_file.rename(new_file)
            count += 1
            print(f"    ✓ 生成汇总表: {new_file.name}")
        except Exception as e:
            print(f"    生成汇总表失败: {e}")

    return count


def collect_key_vis_pngs(vis_dir: Path, all_results_dir: Path):
    """
    将所有子数据集vis目录下的关键PNG文件复制到All_Results根目录
    文件命名格式: {prefix}_{原文件名}.png
    例如: pvt_nocamera_2026-05-07_13-14-52_CEP95_Horizontal.png

    Args:
        vis_dir: 可视化目录路径 (All_Results/vis/)
        all_results_dir: All_Results根目录路径
    """
    if not vis_dir.exists() or not vis_dir.is_dir():
        return

    print("\n" + "=" * 60)
    print("收集关键可视化PNG文件到All_Results根目录")
    print("=" * 60)

    copied_count = 0

    # 遍历所有子数据集的vis目录（如 vis/pvt_nocamera_xxx/）
    for sub_dir in vis_dir.iterdir():
        if not sub_dir.is_dir():
            continue

        prefix = sub_dir.name  # 例如: pvt_nocamera_2026-05-07_13-14-52

        # 查找每个关键PNG文件
        for key_png in KEY_VIS_PNGS:
            # 子数据集中的文件名已经带有前缀，例如: {prefix}_CEP95_Horizontal.png
            src_file = sub_dir / f"{prefix}_{key_png}"

            if src_file.exists():
                # 目标文件名保持原样（已包含前缀），输出到All_Results根目录
                dst_file = all_results_dir / f"{prefix}_{key_png}"

                shutil.copy2(src_file, dst_file)
                print(f"  ✓ 复制: {sub_dir.name}/{src_file.name} -> All_Results/{dst_file.name}")
                copied_count += 1
            else:
                print(f"  缺失: {sub_dir.name}/{prefix}_{key_png}")

    print(f"\n✓ 共复制 {copied_count} 个关键PNG文件到 All_Results 根目录")
    print("=" * 60)


def visualize_all_precision_files(all_results_dir: Path, version_label: str,
                                   schemes_filter: list = None, scenes_filter: list = None,
                                   visualize_types: list = None, dry_run: bool = False):
    """对All_Results目录下所有精度文件进行可视化"""
    if schemes_filter is None:
        schemes_filter = ["lc", "tc", "gnss"]
    if visualize_types is None:
        visualize_types = ["bar", "table"]

    if dry_run:
        print("\n" + "=" * 60)
        print("[DRY RUN] 将执行可视化")
        print(f"  方案: {schemes_filter}")
        print(f"  可视化类型: {visualize_types}")
        if scenes_filter:
            print(f"  场景过滤: {scenes_filter}")
        print("=" * 60)
        return

    if not VISUALIZE_MODULE_AVAILABLE or not VISUALIZE_LIB_AVAILABLE:
        print("\n警告: 可视化模块或依赖库不可用，跳过可视化")
        return

    print("\n" + "=" * 60)
    print("开始可视化所有精度文件")
    print(f"  方案过滤: {schemes_filter}")
    print(f"  可视化类型: {visualize_types}")
    if scenes_filter:
        print(f"  场景过滤: {scenes_filter}")
    print("=" * 60)

    # 设置中文字体
    setup_chinese_font()

    # 创建可视化输出目录
    vis_dir = all_results_dir / "vis"
    vis_dir.mkdir(parents=True, exist_ok=True)
    print(f"可视化输出目录: {vis_dir}")

    # 找到所有精度文件
    precision_files = []

    # 1. 聚合文件（优先）
    aggregated_file = all_results_dir / "1_position_precision_aggregated.txt"
    if aggregated_file.exists():
        precision_files.append((aggregated_file, "aggregated"))

    # 2. 各子数据集的精度文件
    for f in all_results_dir.iterdir():
        if f.is_file() and f.name.endswith("_position_precision.txt") and not f.name.startswith("1_"):
            prefix = f.name.replace("_position_precision.txt", "")
            precision_files.append((f, prefix))

    print(f"\n找到 {len(precision_files)} 个精度文件待可视化")

    total_count = 0
    for precision_file, prefix in precision_files:
        count = visualize_single_file(
            precision_file, vis_dir, prefix, version_label,
            schemes_filter, scenes_filter, visualize_types
        )
        total_count += count

    print("\n" + "=" * 60)
    print(f"✓ 可视化完成! 共生成 {total_count} 个图表/文件")
    print(f"  输出目录: {vis_dir}")

    # 显示生成的目录结构
    print("\n生成的可视化目录:")
    for sub_dir in sorted(vis_dir.iterdir()):
        if sub_dir.is_dir():
            files_count = len(list(sub_dir.iterdir()))
            print(f"  {sub_dir.name}/ ({files_count} 个文件)")

    print("=" * 60)

    # 收集关键PNG文件到All_Results根目录
    collect_key_vis_pngs(vis_dir, all_results_dir)


def main():
    parser = argparse.ArgumentParser(
        description="批量收集评测结果文件到All_Results目录并进行精度聚合统计和可视化"
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default=None,
        help="评测输出根目录路径（用于收集和聚合）",
    )
    parser.add_argument(
        "--all_results_dir",
        type=str,
        default=None,
        help="All_Results目录路径（只进行可视化，不收集和聚合）",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只显示将要执行的操作，不实际执行",
    )
    parser.add_argument(
        "--skip-collect",
        action="store_true",
        help="跳过文件收集",
    )
    parser.add_argument(
        "--skip-aggregate",
        action="store_true",
        help="跳过精度聚合统计",
    )
    parser.add_argument(
        "--skip-visualize",
        action="store_true",
        help="跳过可视化",
    )
    parser.add_argument(
        "--skip-bar",
        action="store_true",
        help="跳过条形图生成",
    )
    parser.add_argument(
        "--skip-table",
        action="store_true",
        help="跳过汇总表生成",
    )
    parser.add_argument(
        "--version-label",
        type=str,
        default=None,
        help="可视化图表的版本标签",
    )
    parser.add_argument(
        "--schemes",
        type=str,
        nargs="+",
        default=["lc", "tc", "gnss"],
        choices=["lc", "tc", "gnss"],
        help="要可视化的方案（默认: lc tc gnss）",
    )
    parser.add_argument(
        "--scenes",
        type=str,
        nargs="+",
        default=None,
        help="要可视化的场景（默认: 自动检测全部场景）",
    )

    args = parser.parse_args()

    # 处理可视化类型
    visualize_types = ["bar", "table"]
    if args.skip_bar:
        visualize_types = [t for t in visualize_types if t != "bar"]
    if args.skip_table:
        visualize_types = [t for t in visualize_types if t != "table"]

    # 确定工作模式
    if args.all_results_dir:
        # 只可视化模式
        all_results_dir = Path(args.all_results_dir)
        if not all_results_dir.exists():
            print(f"错误: All_Results目录不存在: {all_results_dir}")
            return

        print(f"All_Results目录: {all_results_dir}")
        print("=" * 60)

        version_label = args.version_label or all_results_dir.parent.name

        visualize_all_precision_files(
            all_results_dir, version_label,
            schemes_filter=args.schemes,
            scenes_filter=args.scenes,
            visualize_types=visualize_types,
            dry_run=args.dry_run
        )

    elif args.output_dir:
        # 完整流程模式
        output_dir = Path(args.output_dir)
        if not output_dir.exists():
            print(f"错误: 输出目录不存在: {output_dir}")
            return

        print(f"输出目录: {output_dir}")
        print("=" * 60)

        version_label = args.version_label or output_dir.name

        # 1. 收集文件
        copied_count = 0
        all_results_dir = output_dir / "All_Results"

        if not args.skip_collect:
            copied_count, missing_count, all_results_dir = collect_files(output_dir, args.dry_run)

        # 2. 聚合精度统计
        if not args.skip_aggregate and copied_count > 0:
            aggregate_precision_from_subdatasets(output_dir, all_results_dir, args.dry_run)

        # 3. 可视化所有精度文件
        if not args.skip_visualize and (copied_count > 0 or all_results_dir.exists()):
            visualize_all_precision_files(
                all_results_dir, version_label,
                schemes_filter=args.schemes,
                scenes_filter=args.scenes,
                visualize_types=visualize_types,
                dry_run=args.dry_run
            )

    else:
        # 使用默认测试目录
        default_dir = Path("/home/dingxingyu/E2E_PingCe/data_output/All_Results")
        if default_dir.exists():
            print(f"使用默认目录: {default_dir}")
            print("=" * 60)

            version_label = args.version_label or "data_output"

            visualize_all_precision_files(
                default_dir, version_label,
                schemes_filter=args.schemes,
                scenes_filter=args.scenes,
                visualize_types=visualize_types,
                dry_run=args.dry_run
            )
        else:
            print("错误: 请指定 --output_dir 或 --all_results_dir 参数")
            return

    print("\n" + "=" * 60)
    print("所有处理完成!")
    print("=" * 60)


if __name__ == "__main__":
    main()