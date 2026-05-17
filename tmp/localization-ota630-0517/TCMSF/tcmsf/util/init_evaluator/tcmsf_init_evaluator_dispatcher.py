#!/usr/bin/env python3
"""
TCMSF 初始化评测工具调度器

功能：
1. 循环启动子进程运行每个切片
2. 每个切片在独立进程中运行，解决 TCMSF 内部 static call_once 问题
3. 汇总所有切片结果

用法：
python3 tcmsf_init_evaluator_dispatcher.py record_path output_path start_timestamp slice_interval slice_count slice_duration [imu_config_path]

注意：
- 需要在 Apollo Docker 环境中运行
- 会启动多个子进程，每个子进程运行一个切片
- 初始化耗时 = TCMSF Ready! 时间 - TCMSF Start Init! 时间（TCMSF内部定义）

输出文件：
- init_eval_results.csv: 每个切片的详细结果
- init_eval_summary.txt: 汇总统计信息
- init_eval_log.txt: 所有切片的初始化日志
"""

import subprocess
import sys
import os
import time
import re
import shutil
from datetime import datetime

def get_record_start_time_from_probe(record_path, imu_config_path=None):
    """
    通过运行一个短时间探测来获取 record 的实际起始时间戳

    Args:
        record_path: record文件路径

    Returns:
        start_time: 实际起始时间戳（秒）
    """
    evaluator_path = "/apollo/.cache/bazel/base/execroot/apollo/bazel-out/k8-opt/bin/modules/localization/src/TCMSF/tcmsf/util/tcmsf_init_evaluator"

    # 设置环境变量
    env = os.environ.copy()
    lib_paths = [
        "/apollo/modules/localization/src/TCMSF/third_party/wgs84_to_mars/lib/x86",
        "/apollo/cyber_release/dist/x86_64/lib",
        "/apollo/third_party/glog_gflags/lib",
        "/apollo/third_party/absl/lib",
    ]
    env["LD_LIBRARY_PATH"] = ":".join(lib_paths) + ":" + env.get("LD_LIBRARY_PATH", "")

    # 创建临时输出目录
    probe_output = "/tmp/probe_output"
    os.makedirs(probe_output, exist_ok=True)

    # 运行短时间探测：slice_start=0, slice_interval=0, slice_count=1, slice_duration=5
    cmd = [
        evaluator_path,
        record_path,
        probe_output,
        "0",  # slice_start=0 (让程序从数据起点开始)
        "0",  # slice_interval
        "1",  # slice_count
        "5",  # slice_duration (短时间，只获取第一个数据点)
        imu_config_path if imu_config_path else "",
        "0",  # slice_id
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30, cwd="/apollo", env=env)

        # 从输出中提取时间戳，格式: "Slice 0: Dynamic ready at 1775804340.6329"
        # 或 "Dynamic ready (gnss)! imu time: 1775804340.6329"
        match = re.search(r'Dynamic ready.*?([\d]+\.[\d]+)', result.stdout)
        if match:
            dynamic_ready_time = float(match.group(1))
            # Dynamic ready 时间通常比数据起始时间稍晚一点
            # 减去1秒作为估算的起始时间
            start_time = dynamic_ready_time - 1.0
            print("Probe: Dynamic ready at {:.4f}, estimated start at {:.4f}".format(dynamic_ready_time, start_time))
            return start_time

        print("Warning: Could not probe record start time")
        print("Output: {}".format(result.stdout[:500]))
        return None
    except Exception as e:
        print("Warning: Probe failed: {}".format(e))
        return None
    finally:
        # 清理临时目录
        try:
            shutil.rmtree(probe_output)
        except:
            pass

def run_single_slice(record_path, output_base_path, slice_id, slice_start, slice_end, imu_config_path=None, timeout=300):
    """
    运行单个切片评测（启动独立进程）

    Args:
        record_path: record文件路径
        output_base_path: 输出基础路径
        slice_id: 切片编号
        slice_start: 切片起始时间戳
        slice_end: 切片结束时间戳
        imu_config_path: IMU配置文件路径（可选）
        timeout: 超时时间（秒），默认300秒（5分钟）

    Returns:
        (success, result_dict)
    """
    slice_output_dir = os.path.join(output_base_path, f"slice_{slice_id}")
    os.makedirs(slice_output_dir, exist_ok=True)

    # 构建命令 - 在容器内使用绝对路径
    # 注意：这里需要指向编译好的二进制文件
    evaluator_path = "/apollo/.cache/bazel/base/execroot/apollo/bazel-out/k8-opt/bin/modules/localization/src/TCMSF/tcmsf/util/tcmsf_init_evaluator"

    cmd = [
        evaluator_path,
        record_path,
        slice_output_dir,
        str(slice_start),
        "0",  # slice_interval 不重要，只运行一个切片
        "1",  # slice_count = 1
        str(slice_end - slice_start),  # slice_duration
    ]
    # imu_config_path 必须添加（为空时传空字符串），确保 slice_id 在 argv[8] 位置
    cmd.append(imu_config_path if imu_config_path else "")
    # 添加 slice_id 参数，让 C++ 程序知道当前是哪个切片
    cmd.append(str(slice_id))

    # 设置环境变量（Apollo Docker 环境）
    env = os.environ.copy()
    env["CYBER_PATH"] = "/apollo/cyber_release/dist/x86_64"
    # 设置 LD_LIBRARY_PATH 以加载依赖库
    lib_paths = [
        "/apollo/modules/localization/src/TCMSF/third_party/wgs84_to_mars/lib/x86",
        "/apollo/cyber_release/dist/x86_64/lib",
        "/apollo/third_party/glog_gflags/lib",
        "/apollo/third_party/absl/lib",
    ]
    existing_ld = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = ":".join(lib_paths) + ":" + existing_ld if existing_ld else ":".join(lib_paths)

    print(f"  [Slice {slice_id}] Running in subprocess...")

    result = {
        "slice_id": slice_id,
        "slice_start_timestamp": slice_start,
        "slice_end_timestamp": slice_end,
        "dynamic_ready_timestamp": -1.0,
        "init_start_timestamp": -1.0,
        "init_complete_timestamp": -1.0,
        "total_init_time": -1.0,
        "is_success": False,
        "final_state": 0,
        "start_lat": 0.0,
        "start_lon": 0.0,
        "start_speed": 0.0,
        "gnss_count": 0,
        "imu_count": 0,
        "reinit_count": 0
    }

    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout + 30,  # 额外30秒缓冲
            cwd="/apollo",
            env=env
        )

        # 保存原始输出日志 - 包含完整的TCMSF内部日志
        output = proc.stdout + "\n" + proc.stderr
        log_file_path = os.path.join(slice_output_dir, "slice_output.log")
        with open(log_file_path, 'w') as f:
            f.write("# Slice {} Output Log\n".format(slice_id))
            f.write("# Start: {}, End: {}\n".format(slice_start, slice_end))
            f.write("# Timestamp: {}\n".format(datetime.now().isoformat()))
            f.write("#" + "="*50 + "\n\n")
            # 保存完整的原始输出（包含TCMSF内部日志）
            f.write(output)

        # 从子进程生成的结果文件中解析
        # 子进程 tcmsf_init_evaluator 会生成 init_eval_log.txt 和 init_eval_results.csv
        slice_log_file = os.path.join(slice_output_dir, "init_eval_log.txt")
        slice_csv_file = os.path.join(slice_output_dir, "init_eval_results.csv")

        # 解析日志文件中的关键时间点
        if os.path.exists(slice_log_file):
            with open(slice_log_file, 'r') as f:
                log_content = f.read()

            # 提取关键时间点 - TCMSF内部日志格式
            dynamic_ready_regex = re.compile(r'\[startup\] Dynamic ready \(gnss\)! imu time:\s*([\d.]+)')
            start_init_regex = re.compile(r'\[startup\] TCMSF Start Init! imu time:\s*([\d.]+)')
            ready_regex = re.compile(r'\[startup\] TCMSF Ready! imu time:\s*([\d.]+)')
            result_regex = re.compile(r'Result: (SUCCESS|FAILED), total_init_time=([\d.]+)s')

            for line in log_content.split('\n'):
                # Dynamic ready
                match = dynamic_ready_regex.search(line)
                if match:
                    result["dynamic_ready_timestamp"] = float(match.group(1))

                # Start Init
                match = start_init_regex.search(line)
                if match:
                    result["init_start_timestamp"] = float(match.group(1))

                # Ready (Init Complete)
                match = ready_regex.search(line)
                if match:
                    result["init_complete_timestamp"] = float(match.group(1))
                    result["is_success"] = True
                    result["final_state"] = 3  # INIT_COMPLETED

                # 从结果行提取耗时
                match = result_regex.search(line)
                if match:
                    result["total_init_time"] = float(match.group(2))
                    if match.group(1) == "SUCCESS":
                        result["is_success"] = True
                        result["final_state"] = 3

        # 从 CSV 文件中解析 gnss_count 和 imu_count
        if os.path.exists(slice_csv_file):
            with open(slice_csv_file, 'r') as f:
                csv_content = f.read()
                # 跳过注释行，找到数据行
                for line in csv_content.split('\n'):
                    if line.startswith('#') or not line.strip():
                        continue
                    parts = line.split(',')
                    if len(parts) >= 19:
                        # CSV 格式: slice_id,...,gnss_count,imu_count,...
                        # gnss_count 在第17列（索引17），imu_count 在第18列（索引18）
                        try:
                            result["gnss_count"] = int(parts[17])
                            result["imu_count"] = int(parts[18])
                        except:
                            pass
                    break  # 只取第一行数据

        # 计算耗时（如果日志文件中没有提取到）
        # total_init_time = TCMSF Ready! time - Dynamic Ready! time
        if result["total_init_time"] < 0 and result["init_complete_timestamp"] > 0 and result["dynamic_ready_timestamp"] > 0:
            result["total_init_time"] = result["init_complete_timestamp"] - result["dynamic_ready_timestamp"]

        result["reinit_count"] = int(parts[12])

        # 如果没有成功，检查状态
        if not result["is_success"]:
            if result["init_start_timestamp"] > 0 and result["init_complete_timestamp"] < 0:
                result["final_state"] = 2  # INIT_STARTED but not completed
            elif result["dynamic_ready_timestamp"] > 0 and result["init_start_timestamp"] < 0:
                result["final_state"] = 1  # DYNAMIC_READY but not started init
            else:
                result["final_state"] = 4  # FAILED/TIMEOUT

        return result["is_success"], result

    except subprocess.TimeoutExpired:
        print(f"  [Slice {slice_id}] Timeout expired (> {timeout}s)")
        result["final_state"] = 4  # FAILED/TIMEOUT
        # 保存超时日志
        log_file_path = os.path.join(slice_output_dir, "slice_output.log")
        with open(log_file_path, 'w') as f:
            f.write(f"# Slice {slice_id} TIMEOUT\n")
            f.write(f"# Start: {slice_start}, End: {slice_end}\n")
            f.write(f"# Timeout threshold: {timeout}s\n")
        return False, result
    except Exception as e:
        print(f"  [Slice {slice_id}] Error: {e}")
        result["final_state"] = 5  # ERROR
        return False, result

def write_combined_results(results, output_path, record_path, start_timestamp, slice_interval, slice_count, slice_duration, invalid_slices=None, init_timeout=300, start_timestamp_input=None, auto_mode=False):
    """
    写入汇总结果

    输出文件：
    - init_eval_results.csv: CSV格式详细结果
    - init_eval_summary.txt: 汇总统计信息
    - init_eval_log.txt: 初始化日志汇总

    Args:
        results: 有效切片结果列表
        output_path: 输出目录
        record_path: record文件路径
        start_timestamp: 实际使用的起始时间戳
        slice_interval: 切片间隔
        slice_count: 有效切片数量
        slice_duration: 切片时长参数
        invalid_slices: 无效切片列表
        init_timeout: 初始化超时时间
        start_timestamp_input: 用户输入的起始时间戳参数
        auto_mode: 是否自动模式(slice_count=-1)
    """
    csv_file = os.path.join(output_path, "init_eval_results.csv")
    summary_file = os.path.join(output_path, "init_eval_summary.txt")
    log_file = os.path.join(output_path, "init_eval_log.txt")

    # 写入 CSV
    with open(csv_file, 'w') as f:
        f.write("# TCMSF Initialization Evaluation Results (Dispatcher Mode)\n")
        f.write("# Method: Each slice runs in separate subprocess to bypass static call_once\n")
        f.write("# Generated: {}\n".format(datetime.now().isoformat()))
        f.write("#\n")
        f.write("# slice_id,slice_start_ts,slice_end_ts,dynamic_ready_ts,init_start_ts,init_complete_ts,")
        f.write("total_init_time,final_state,is_success,start_lat,start_lon,start_speed,reinit_count\n")

        for r in results:
            f.write(f"{r['slice_id']},{r['slice_start_timestamp']:.6f},{r['slice_end_timestamp']:.6f},")
            f.write(f"{r['dynamic_ready_timestamp']:.6f},{r['init_start_timestamp']:.6f},{r['init_complete_timestamp']:.6f},")
            f.write(f"{r['total_init_time']:.6f},{r['final_state']},{r['is_success']},")
            f.write(f"{r['start_lat']:.6f},{r['start_lon']:.6f},{r['start_speed']:.6f},{r['reinit_count']:d}\n")

    # 写入汇总日志
    with open(log_file, 'w') as f:
        f.write("# TCMSF Initialization Evaluation Log\n")
        f.write("# Record path: {}\n".format(record_path))
        f.write("# Start timestamp: {}\n".format(start_timestamp))
        f.write("# Slice interval: {}s\n".format(slice_interval))
        f.write("# Slice count: {}\n".format(slice_count))
        f.write("# Init timeout: {}s\n".format(init_timeout))
        f.write("# =====================================================\n\n")

        for r in results:
            f.write("[Slice {}] Start at timestamp: {}\n".format(r['slice_id'], r['slice_start_timestamp']))

            if r['dynamic_ready_timestamp'] > 0:
                f.write("[Slice {}][startup] Dynamic ready (gnss)! imu time: {}\n".format(r['slice_id'], r['dynamic_ready_timestamp']))

            if r['init_start_timestamp'] > 0:
                f.write("[Slice {}][startup] TCMSF Start Init! imu time: {}\n".format(r['slice_id'], r['init_start_timestamp']))

            if r['init_complete_timestamp'] > 0:
                f.write("[Slice {}][startup] TCMSF Ready! imu time: {}\n".format(r['slice_id'], r['init_complete_timestamp']))

            status = "SUCCESS" if r['is_success'] else "FAILED"
            f.write("[Slice {}] Result: {}, total_init_time={:.4f}s\n\n".format(r['slice_id'], status, r['total_init_time']))

    # 写入汇总统计
    total = len(results)
    success_count = sum(1 for r in results if r['is_success'])
    all_reinit_count = sum(1 for r in results if r['reinit_count'])
    fail_count = total - success_count
    init_times = [r['total_init_time'] for r in results if r['total_init_time'] > 0]

    with open(summary_file, 'w') as f:
        f.write("# TCMSF Initialization Evaluation Summary\n")
        f.write("#" + "="*60 + "\n")
        f.write(f"# Generated: {datetime.now().isoformat()}\n")
        f.write("#" + "="*60 + "\n\n")

        # 时间戳范围信息
        f.write("# Timestamp Range Information\n")
        f.write("#" + "-"*40 + "\n")
        f.write("Record path: {}\n".format(record_path))

        # 计算实际处理的时间戳范围
        if results:
            # 有效切片的起始时间戳（第一个切片的起始）
            actual_start_ts = results[0]['slice_start_timestamp']
            # 有效切片的结束时间戳（最后一个切片的起始 + 初始化完成时间）
            last_result = results[-1]
            if last_result['init_complete_timestamp'] > 0:
                actual_end_ts = last_result['init_complete_timestamp']
            else:
                actual_end_ts = last_result['slice_start_timestamp'] + slice_interval
        else:
            actual_start_ts = start_timestamp
            actual_end_ts = start_timestamp

        # 显示参数设置和实际范围
        f.write("\n# Parameter Settings\n")
        if start_timestamp_input is not None and start_timestamp_input <= 0:
            f.write("Start timestamp parameter: 0 (auto-detect from record)\n")
        else:
            f.write("Start timestamp parameter: {:.4f}\n".format(start_timestamp_input if start_timestamp_input else start_timestamp))
        f.write("Slice interval: {:.4f} seconds\n".format(slice_interval))
        if auto_mode:
            f.write("Slice count parameter: -1 (auto, run until data ends)\n")
        else:
            f.write("Slice count parameter: {}\n".format(slice_count))

        f.write("\n# Actual Processed Timestamp Range\n")
        f.write("First valid slice start: {:.4f}\n".format(actual_start_ts))
        f.write("Last valid slice end: {:.4f}\n".format(actual_end_ts))
        if actual_end_ts > actual_start_ts:
            duration = actual_end_ts - actual_start_ts
            f.write("Total duration: {:.4f} seconds ({:.2f} minutes)\n".format(duration, duration / 60.0))

        f.write("\n#" + "="*60 + "\n")
        f.write("# Evaluation Results\n")
        f.write("#" + "-"*40 + "\n\n")

        f.write("Total slices tested: {}\n".format(total))
        f.write("Successful initializations: {}\n".format(success_count))
        f.write("Failed initializations: {}\n".format(fail_count))
        f.write("Success rate: {:.4f}%\n\n".format(success_count * 100.0 / total if total > 0 else 0.0))

        f.write("Total reinit count: {}\n".format(all_reinit_count))

        f.write("# Timing Statistics (successful cases)\n")
        f.write("# Note: total_init_time = TCMSF Ready! time - Dynamic Ready! time\n")
        if init_times:
            avg_time = sum(init_times) / len(init_times)
            min_time = min(init_times)
            max_time = max(init_times)
            sorted_times = sorted(init_times)
            median_time = sorted_times[len(sorted_times) // 2]

            f.write("Average init time: {:.4f} seconds\n".format(avg_time))
            f.write("Min init time: {:.4f} seconds\n".format(min_time))
            f.write("Max init time: {:.4f} seconds\n".format(max_time))
            f.write("Median init time: {:.4f} seconds\n".format(median_time))

            # 分位数统计
            p25 = sorted_times[int(len(sorted_times) * 0.25)]
            p75 = sorted_times[int(len(sorted_times) * 0.75)]
            p95 = sorted_times[int(len(sorted_times) * 0.95)]
            f.write("25th percentile: {:.4f} seconds\n".format(p25))
            f.write("75th percentile: {:.4f} seconds\n".format(p75))
            f.write("95th percentile: {:.4f} seconds\n".format(p95))
        else:
            f.write("No successful initializations\n")

        # 添加无效切片详细信息
        if invalid_slices and len(invalid_slices) > 0:
            f.write("\n#" + "="*60 + "\n")
            f.write("# Invalid Slices (No Data)\n")
            f.write("# These slices were excluded from statistics\n")
            f.write("#" + "-"*40 + "\n")
            for inv in invalid_slices:
                f.write("\nSlice {}: No data found\n".format(inv['slice_id']))
                f.write("  Start timestamp: {:.4f}\n".format(inv['slice_start_timestamp']))
                f.write("  Reason: {}\n".format(inv['reason']))

        f.write("\n#" + "="*60 + "\n")
        f.write("# Method: Each slice runs in separate subprocess\n")
        f.write("# This bypasses TCMSF internal static call_once mechanism\n")
        f.write("# Timeout threshold: {} seconds\n".format(init_timeout))

def main():
    """
    主函数 - 运行TCMSF初始化评测
    """
    if len(sys.argv) < 7:
        print("Usage: python3 tcmsf_init_evaluator_dispatcher.py record_path output_path start_timestamp slice_interval slice_count init_timeout [imu_config_path]")
        print("\nArguments:")
        print("  record_path      : record文件路径（单个文件或目录）")
        print("  output_path      : 输出结果路径")
        print("  start_timestamp  : 起始时间戳（Unix时间戳，如1774324300.0；0表示自动检测）")
        print("  slice_interval   : 切片间隔（秒，如40）")
        print("  slice_count      : 切片总数（正整数，如20；-1表示运行到数据结束）")
        print("  init_timeout     : 单个切片初始化超时时间（秒，如300）")
        print("  imu_config_path  : IMU配置文件路径（可选）")
        print("\nNote:")
        print("  slice_count=-1 表示持续运行直到record数据结束")
        print("  每个切片持续播放数据直到初始化成功或超时(init_timeout秒)")
        print("\nExample:")
        print("  python3 tcmsf_init_evaluator_dispatcher.py /apollo/data/record /apollo/data/output 0 40 -1 300")
        print("  python3 tcmsf_init_evaluator_dispatcher.py /apollo/data/record /apollo/data/output 1774324300 40 20 120")
        print("\nOutput Files:")
        print("  init_eval_results.csv  : 详细结果CSV")
        print("  init_eval_summary.txt  : 汇总统计")
        print("  init_eval_log.txt      : 日志汇总")
        sys.exit(1)

    record_path = sys.argv[1]
    output_path = sys.argv[2]
    start_timestamp_input = float(sys.argv[3])
    slice_interval = float(sys.argv[4])
    slice_count_input = int(sys.argv[5])
    init_timeout = float(sys.argv[6])  # 初始化超时时间（秒）
    imu_config_path = sys.argv[7] if len(sys.argv) >= 8 else None

    # slice_count = -1 表示运行到数据结束
    auto_mode = (slice_count_input == -1)

    os.makedirs(output_path, exist_ok=True)

    # 如果 start_timestamp <= 0，自动获取 record 的实际起始时间戳
    if start_timestamp_input <= 0:
        print("Auto-detecting record start timestamp...")
        record_start_time = get_record_start_time_from_probe(record_path, imu_config_path)
        if record_start_time is None:
            print("Error: Cannot determine record start time")
            sys.exit(1)
        start_timestamp = record_start_time
        print("Record start timestamp: {}".format(record_start_time))
    else:
        start_timestamp = start_timestamp_input

    print("\n" + "="*60)
    print("TCMSF Initialization Evaluation (Dispatcher Mode)")
    print("="*60)
    print("Record path: {}".format(record_path))
    print("Output path: {}".format(output_path))
    print("Start timestamp: {} (offset=0)".format(start_timestamp))
    if auto_mode:
        print("Slice count: auto (run until data ends)")
    else:
        print("Slice count: {}".format(slice_count_input))
    print("Slice interval: {}s".format(slice_interval))
    print("Init timeout: {}s".format(init_timeout))
    print("\nNote: Each slice runs in separate subprocess to bypass")
    print("      TCMSF internal static call_once mechanism")
    print("      Data is played continuously until init success or timeout")
    print("="*60 + "\n")

    results = []
    invalid_slices = []  # 记录无效切片的详细信息
    overall_start_time = time.time()
    consecutive_no_data_count = 0  # 连续无数据切片计数

    i = 0
    while True:
        # slice_start 是绝对时间戳 = 起始时间戳 + 偏移量
        slice_start = start_timestamp + i * slice_interval

        # 检查是否超出切片数量限制（非auto模式）
        if not auto_mode and i >= slice_count_input:
            break

        print("\n--- Slice {} [start: {:.1f}] ---".format(i, slice_start))

        # 运行单个切片（子进程）
        # slice_end 设为很大的值，让程序持续运行直到初始化成功或超时
        success, result = run_single_slice(
            record_path, output_path, i, slice_start, slice_start + 999999, imu_config_path, init_timeout
        )

        # auto模式下检测数据结束：如果切片失败且gnss_count和imu_count都为0，说明没有数据
        if auto_mode:
            if result['gnss_count'] == 0 and result['imu_count'] == 0:
                consecutive_no_data_count += 1
                print("  No data found in this slice (consecutive: {})".format(consecutive_no_data_count))
                # 记录无效切片信息
                invalid_slices.append({
                    'slice_id': i,
                    'slice_start_timestamp': slice_start,
                    'reason': 'No data in this time range (gnss_count=0, imu_count=0)'
                })
                if consecutive_no_data_count >= 3:
                    print("\nReached end of record data (3 consecutive slices with no data)")
                    break
            else:
                consecutive_no_data_count = 0  # 重置计数

        results.append(result)

        status = "SUCCESS" if success else "FAILED"
        print("  Result: {}, init_time={:.4f}s, gnss_count={}, imu_count={}".format(
            status, result['total_init_time'], result['gnss_count'], result['imu_count']))

        i += 1

    # 更新实际切片数量
    slice_count = len(results)

    # auto模式下，过滤掉无效数据（无数据的切片）后再统计
    if auto_mode:
        valid_results = [r for r in results if r['gnss_count'] > 0 or r['imu_count'] > 0]
        invalid_count = len(results) - len(valid_results)
        if invalid_count > 0:
            print("\nNote: {} slices with no data were excluded from statistics".format(invalid_count))
        results = valid_results
        slice_count = len(results)

    # 写入汇总结果（包含无效切片信息）
    write_combined_results(results, output_path, record_path, start_timestamp, slice_interval, slice_count, 0, invalid_slices, init_timeout=init_timeout, start_timestamp_input=start_timestamp_input, auto_mode=auto_mode)

    elapsed_time = time.time() - overall_start_time
    success_count = sum(1 for r in results if r['is_success'])

    print("\n" + "="*60)
    print("Evaluation Complete")
    print("="*60)
    print("Total time: {:.1f} seconds".format(elapsed_time))
    print("Success rate: {} / {} ({:.1f}%)".format(success_count, slice_count, success_count * 100.0 / slice_count if slice_count > 0 else 0.0))
    print("="*60)


if __name__ == "__main__":
    main()