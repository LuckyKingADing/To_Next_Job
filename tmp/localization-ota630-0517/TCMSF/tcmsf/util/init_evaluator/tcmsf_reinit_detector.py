#!/usr/bin/env python3
"""
TCMSF 重初始化检测工具

功能：
1. 运行完整的record数据（不切片）
2. 检测 "reinit due to large pos error" 和 "reinit for sensor delay issue" 日志
3. 统计重初始化次数和时间戳

用法：
python3 tcmsf_reinit_detector.py record_path output_path [start_timestamp] [imu_config_path]

输出文件：
- reinit_report.txt: 重初始化报告（分组显示初始化日志）
- full_record_output.log: 完整运行日志
"""

import subprocess
import sys
import os
import re
import time
from datetime import datetime

def run_full_record(record_path, output_path, start_timestamp=None, imu_config_path=None, timeout=600):
    """
    运行完整的record数据，检测重初始化

    Args:
        record_path: record文件路径
        output_path: 输出路径
        start_timestamp: 起始时间戳（可选）
        imu_config_path: IMU配置文件路径（可选）
        timeout: 超时时间（秒）

    Returns:
        reinit_events: 重初始化事件列表 [(timestamp, count), ...]
    """
    os.makedirs(output_path, exist_ok=True)

    # 使用临时目录存储 evaluator 输出，避免生成不需要的 init_eval_*.csv/log 文件
    temp_output_path = "/tmp/tcmsf_reinit_temp"

    # 构建命令
    evaluator_path = "/apollo/.cache/bazel/base/execroot/apollo/bazel-out/k8-opt/bin/modules/localization/src/TCMSF/tcmsf/util/tcmsf_init_evaluator"

    # 使用 slice_count=1, slice_interval=0, slice_duration=很大值 来跑完整数据
    # 或者直接用 start_timestamp=-1 表示从头开始跑完整数据

    if start_timestamp is None:
        # 获取record的起始时间
        start_timestamp = 0  # 从0开始，让程序自己找到数据起点

    cmd = [
        evaluator_path,
        record_path,
        temp_output_path,  # 使用临时目录
        str(start_timestamp),
        "0",  # slice_interval
        "1",  # slice_count - 只运行一次完整的数据
        "999999",  # slice_duration - 非常大的值，确保跑完所有数据
        imu_config_path if imu_config_path else "",  # imu_config_path 必须有值
        "0",  # slice_id
        "1",  # continue_after_init = 1，初始化完成后继续运行
    ]

    # 设置 Apollo 环境变量
    env = os.environ.copy()
    env["CYBER_PATH"] = "/apollo/cyber_release/dist/x86_64"
    env["LD_LIBRARY_PATH"] = "/apollo/cyber_release/dist/x86_64/lib:" + env.get("LD_LIBRARY_PATH", "")
    env["GLOG_logtostderr"] = "1"

    print("\n" + "="*60)
    print("TCMSF Reinitialization Detection")
    print("="*60)
    print("Record path: {}".format(record_path))
    print("Output path: {}".format(output_path))
    print("Timeout: {}s".format(timeout))
    print("="*60 + "\n")

    print("Running full record data...")
    start_time = time.time()

    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            cwd="/apollo",
            env=env
        )

        elapsed = time.time() - start_time
        print("Completed in {:.1f} seconds".format(elapsed))

        # 保存完整输出日志
        output = proc.stdout + "\n" + proc.stderr
        log_file = os.path.join(output_path, "full_record_output.log")
        with open(log_file, 'w') as f:
            f.write("# Full Record Output Log\n")
            f.write("# Record: {}\n".format(record_path))
            f.write("# Timestamp: {}\n".format(datetime.now().isoformat()))
            f.write("# Elapsed: {:.1f}s\n".format(elapsed))
            f.write("#" + "="*50 + "\n\n")
            f.write(output)

        # 从日志中提取重初始化事件和初始化日志
        reinit_events = []
        all_init_logs = []  # 收集所有初始化相关的原始日志（包括初始化和重初始化）

        # 多种重初始化日志格式：
        # 1. "reinit due to large pos error" - gps_single_processor.cpp:71
        # 2. "reinit for sensor delay issue" - processor_interface_impl.cpp:373
        reinit_regexes = [
            re.compile(r'reinit due to large pos error'),
            re.compile(r'reinit for sensor delay issue'),
        ]

        # 初始化日志格式：
        # [startup] Dynamic ready (gnss)! imu time: XXX
        # [startup] TCMSF Start Init! imu time: XXX
        # [startup] TCMSF Ready! imu time: XXX
        startup_regex = re.compile(r'\[startup\]')

        # 详细初始化日志格式：
        # [initialization] reset attitude/velocity/position
        # att (deg), vel (m/s), reset lever, pos, initialization timestamp, initialization_count
        init_detail_regex = re.compile(r'\[initialization\]|att \(deg\)|vel \(m/s\)|reset lever|pos:|initialization_count')

        # 从 glog 输出中找时间戳
        imu_time_regex = re.compile(r'imu time:\s*([\d.]+)')
        init_timestamp_regex = re.compile(r'\[initialization\] timestamp:\s*([\d.]+)')
        initialization_count_regex = re.compile(r'initialization_count:\s*([\d]+)')

        current_imu_time = 0.0
        reinit_count = 0
        total_init_count = 0

        for line in output.split('\n'):
            line_stripped = line.strip()

            # 更新当前 IMU 时间（从各种日志中提取）
            imu_match = imu_time_regex.search(line)
            if imu_match:
                current_imu_time = float(imu_match.group(1))

            # 检测初始化时间戳
            init_ts_match = init_timestamp_regex.search(line)
            if init_ts_match:
                init_ts = float(init_ts_match.group(1))

            # 检测 initialization_count（总初始化次数）
            init_count_match = initialization_count_regex.search(line)
            if init_count_match:
                total_init_count = int(init_count_match.group(1))

            # 收集初始化日志（[startup]日志）
            if startup_regex.search(line):
                all_init_logs.append(("STARTUP", line_stripped))

            # 收集详细初始化日志（[initialization]相关）
            if init_detail_regex.search(line):
                all_init_logs.append(("INIT_DETAIL", line_stripped))

            # 检测重初始化日志
            for regex in reinit_regexes:
                if regex.search(line):
                    reinit_count += 1
                    reinit_events.append((current_imu_time, reinit_count, line_stripped))
                    all_init_logs.append(("REINIT", line_stripped))
                    print("  [Reinit #{}] Detected at imu time: {:.4f}".format(reinit_count, current_imu_time))
                    break

        # 如果没有精确匹配到重初始化日志，但有多次初始化，从 initialization_count 推断
        if len(reinit_events) == 0 and total_init_count > 1:
            print("  Note: Found initialization_count={}, implying {} reinit events".format(
                total_init_count, total_init_count - 1))

        return reinit_events, total_init_count, all_init_logs

    except subprocess.TimeoutExpired:
        print("Timeout expired (> {}s)".format(timeout))
        return [], 0, []
    except Exception as e:
        print("Error: {}".format(e))
        return [], 0, []

def write_report(reinit_events, total_init_count, all_init_logs, output_path, record_path):
    """
    写入重初始化报告
    """
    report_file = os.path.join(output_path, "reinit_report.txt")

    with open(report_file, 'w') as f:
        f.write("# TCMSF Reinitialization Detection Report\n")
        f.write("#" + "="*60 + "\n")
        f.write("# Record path: {}\n".format(record_path))
        f.write("# Generated: {}\n".format(datetime.now().isoformat()))
        f.write("#" + "="*60 + "\n\n")

        # 从 initialization_count 推算重初始化次数（首次不算重初始化）
        reinit_count_from_init_count = max(0, total_init_count - 1) if total_init_count > 0 else len(reinit_events)

        f.write("Total initialization count: {}\n".format(total_init_count))
        f.write("Reinitialization events detected: {}\n".format(len(reinit_events)))
        f.write("Estimated reinit count (from init_count): {}\n\n".format(reinit_count_from_init_count))

        # 按初始化序号分组输出日志
        f.write("# Initialization Logs (Grouped by Sequence)\n")
        f.write("#" + "="*60 + "\n\n")

        if all_init_logs:
            # 分组逻辑：
            # - 首次初始化：从开始到 initialization_count:1 + TCMSF Ready!（首次初始化完成标志）
            # - 重初始化：从 reinit trigger 到 initialization_count
            init_blocks = []  # [(init_num, [logs])]
            current_block = []
            current_init_num = 0
            i = 0

            while i < len(all_init_logs):
                log_type, log = all_init_logs[i]
                current_block.append(log)

                # 检测 initialization_count
                init_count_match = re.search(r'initialization_count:\s*([\d]+)', log)
                if init_count_match:
                    current_init_num = int(init_count_match.group(1))

                    # 首次初始化特殊处理：需要包含 TCMSF Ready! 日志
                    if current_init_num == 1:
                        # 检查下一个日志是否是 TCMSF Ready!
                        if i + 1 < len(all_init_logs):
                            next_log_type, next_log = all_init_logs[i + 1]
                            if re.search(r'\[startup\] TCMSF Ready!', next_log):
                                current_block.append(next_log)
                                i += 1  # 跳过下一个日志

                    init_blocks.append((current_init_num, current_block))
                    current_block = []

                i += 1

            # 输出每个初始化块
            for init_num, logs in init_blocks:
                # 判断是首次初始化还是重初始化
                if init_num == 1:
                    label = "First Initialization"
                else:
                    label = "Reinitialization #{}".format(init_num - 1)

                f.write("# {} (count={})\n".format(label, init_num))
                f.write("#" + "-"*40 + "\n")
                for log_line in logs:
                    f.write("{}\n".format(log_line))
                f.write("\n")

            # 输出重初始化触发日志摘要
            if reinit_events:
                f.write("\n# Reinitialization Trigger Summary\n")
                f.write("#" + "-"*40 + "\n")
                for ts, count, log in reinit_events:
                    f.write("Reinit trigger #{}: {}\n".format(count, log))
        else:
            f.write("No initialization logs found.\n")

    return report_file

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 tcmsf_reinit_detector.py record_path output_path [start_timestamp] [imu_config_path]")
        print("\nArguments:")
        print("  record_path      : record文件路径")
        print("  output_path      : 输出结果路径")
        print("  start_timestamp  : 起始时间戳（可选，默认从数据起点开始）")
        print("  imu_config_path  : IMU配置文件路径（可选）")
        print("\nExample:")
        print("  python3 tcmsf_reinit_detector.py /apollo/data/record /apollo/data/reinit_output")
        sys.exit(1)

    record_path = sys.argv[1]
    output_path = sys.argv[2]
    start_timestamp = float(sys.argv[3]) if len(sys.argv) >= 4 else None
    imu_config_path = sys.argv[4] if len(sys.argv) >= 5 else None

    # 运行完整数据检测重初始化
    reinit_events, total_init_count, init_logs = run_full_record(record_path, output_path, start_timestamp, imu_config_path)

    # 写入报告
    report_file = write_report(reinit_events, total_init_count, init_logs, output_path, record_path)

    reinit_count = max(0, total_init_count - 1) if total_init_count > 0 else len(reinit_events)
    print("\n" + "="*60)
    print("Reinitialization Detection Complete")
    print("="*60)
    print("Total initialization count: {}".format(total_init_count))
    print("Reinit events detected: {}".format(len(reinit_events)))
    print("Estimated reinit count: {}".format(reinit_count))
    print("Report: {}".format(report_file))
    print("Full log: {}".format(os.path.join(output_path, "full_record_output.log")))
    print("="*60)

if __name__ == "__main__":
    main()