#!/usr/bin/env python3
"""
TCMSF 初始化评测工具调度器 - 云端版本（切片测试）

功能：
1. 使用 bin/tcmsf_init_evaluator 进行切片测试
2. 自动检测数据起始时间
3. 运行时自动检测数据结束（连续3个切片无数据时停止）
4. 生成简洁的汇总报告

用法：
python3 tcmsf_init_evaluator_dispatcher_cloud.py record_path output_path start_timestamp slice_interval slice_count init_timeout [pvtlc_source]

输出文件：
- init_eval_summary.txt: 子数据集汇总（表格格式）
- init_eval_results.csv: 所有切片详细结果
- slice_*/: 每个切片的详细日志
"""

import subprocess
import sys
import os
import time
import re
import shutil
from datetime import datetime
from pathlib import Path


def get_tcmsf_offline_dir():
    """自动检测 tcmsf_offline 目录路径"""
    script_path = Path(__file__).resolve()
    current_dir = script_path.parent

    while current_dir != current_dir.parent:
        source_env_candidate = current_dir / "source_env.sh"
        if source_env_candidate.exists():
            return current_dir
        current_dir = current_dir.parent

    return script_path.parent.parent.parent


def source_environment_variables(tcmsf_offline_dir):
    """加载环境变量"""
    env = os.environ.copy()
    source_env_path = tcmsf_offline_dir / "source_env.sh"

    if source_env_path.exists():
        try:
            result = subprocess.run(
                f"source {source_env_path} && env",
                shell=True, executable='/bin/bash',
                check=True, capture_output=True, text=True
            )
            for line in result.stdout.split('\n'):
                if '=' in line:
                    key, value = line.split('=', 1)
                    env[key] = value
        except:
            pass

    # 设置 Apollo/Cyber 环境变量（关键！）
    env["CYBER_PATH"] = "/apollo/cyber_release/dist/x86_64"
    env["LD_LIBRARY_PATH"] = "/apollo/cyber_release/dist/x86_64/lib:/apollo/bazel-bin/modules/localization/src/TCMSF:" + env.get("LD_LIBRARY_PATH", "")
    env["GLOG_logtostderr"] = "1"

    lib_dir = tcmsf_offline_dir / "lib"
    if lib_dir.exists():
        env["LD_LIBRARY_PATH"] = str(lib_dir) + ":" + env.get("LD_LIBRARY_PATH", "")

    return env


def get_record_start_time(record_path, tcmsf_offline_dir, env, timeout=30):
    """探测 record 起始时间（只探测起始，结束时间由运行时检测）"""
    evaluator_path = tcmsf_offline_dir / "bin" / "tcmsf_init_evaluator"
    if not evaluator_path.exists():
        return None

    probe_output = "/tmp/probe_output_range"
    os.makedirs(probe_output, exist_ok=True)

    # 使用一个很早的时间戳（2020年初）来扫描数据
    EARLY_START_TS = 1577836800  # 2020-01-01 00:00:00 UTC

    try:
        cwd_dir = "/apollo" if os.path.exists("/apollo") else str(tcmsf_offline_dir)

        # 运行短时间探测获取起始时间
        cmd_probe_start = [str(evaluator_path), record_path, probe_output,
                           str(EARLY_START_TS), "0", "1", "10", "", "0"]

        result = subprocess.run(cmd_probe_start, capture_output=True, text=True, timeout=timeout,
                                cwd=cwd_dir, env=env)

        # 解析 Dynamic ready 时间作为起始时间
        start_time = None
        match = re.search(r'Dynamic ready.*?imu time:\s*([\d]+\.[\d]+)', result.stdout)
        if match:
            dynamic_ready_time = float(match.group(1))
            start_time = dynamic_ready_time - 1.0

        if start_time is None:
            # 尝试从日志文件读取
            log_file = os.path.join(probe_output, "init_eval_log.txt")
            if os.path.exists(log_file):
                with open(log_file, 'r') as f:
                    match = re.search(r'Dynamic ready.*?imu time:\s*([\d]+\.[\d]+)', f.read())
                    if match:
                        dynamic_ready_time = float(match.group(1))
                        start_time = dynamic_ready_time - 1.0

        return start_time

    except subprocess.TimeoutExpired:
        # 超时时尝试从 stdout 获取起始时间
        if hasattr(result, 'stdout') and result.stdout:
            match = re.search(r'Dynamic ready.*?imu time:\s*([\d]+\.[\d]+)', result.stdout)
            if match:
                return float(match.group(1)) - 1.0
        return None
    except Exception as e:
        return None
    finally:
        shutil.rmtree(probe_output, ignore_errors=True)


def run_single_slice(record_path, output_base_path, slice_id, slice_start,
                     tcmsf_offline_dir, env, pvtlc_source="pvt", timeout=300):
    """运行单个切片"""
    slice_output_dir = os.path.join(output_base_path, f"slice_{slice_id}")
    os.makedirs(slice_output_dir, exist_ok=True)

    evaluator_path = tcmsf_offline_dir / "bin" / "tcmsf_init_evaluator"

    cmd = [
        str(evaluator_path), record_path, slice_output_dir,
        str(slice_start), "0", "1", "300",
        "--pvtlc-source", pvtlc_source,
        "--slice-id", str(slice_id)
    ]

    result = {
        "slice_id": slice_id,
        "slice_start": slice_start,
        "dynamic_ready": -1.0,
        "init_start": -1.0,
        "init_complete": -1.0,
        "init_time": -1.0,
        "is_success": False,
        "is_skipped": False,  # 新增：是否被跳过
        "gnss_count": 0,
        "imu_count": 0
    }

    try:
        cwd_dir = "/apollo" if os.path.exists("/apollo") else str(tcmsf_offline_dir)
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout + 30,
                              cwd=cwd_dir, env=env)

        # 保存日志
        with open(os.path.join(slice_output_dir, "slice_output.log"), 'w') as f:
            f.write(proc.stdout + "\n" + proc.stderr)

        # 解析结果
        log_file = os.path.join(slice_output_dir, "init_eval_log.txt")
        csv_file = os.path.join(slice_output_dir, "init_eval_results.csv")

        if os.path.exists(log_file):
            with open(log_file, 'r') as f:
                log_content = f.read()

            for line in log_content.split('\n'):
                m = re.search(r'Dynamic ready.*?([\d.]+)', line)
                if m:
                    result["dynamic_ready"] = float(m.group(1))

                m = re.search(r'TCMSF Start Init.*?([\d.]+)', line)
                if m:
                    result["init_start"] = float(m.group(1))

                m = re.search(r'TCMSF Ready.*?([\d.]+)', line)
                if m:
                    result["init_complete"] = float(m.group(1))
                    result["is_success"] = True

                m = re.search(r'Result: (SUCCESS|FAILED), total_init_time=([\d.]+)', line)
                if m:
                    result["init_time"] = float(m.group(2))
                    if m.group(1) == "SUCCESS":
                        result["is_success"] = True

        if os.path.exists(csv_file):
            with open(csv_file, 'r') as f:
                for line in f:
                    if line.startswith('#') or not line.strip():
                        continue
                    parts = line.split(',')
                    if len(parts) >= 19:
                        try:
                            result["gnss_count"] = int(parts[17])
                            result["imu_count"] = int(parts[18])
                        except:
                            pass
                    break

        if result["init_time"] < 0 and result["init_complete"] > 0 and result["dynamic_ready"] > 0:
            result["init_time"] = result["init_complete"] - result["dynamic_ready"]

        return result

    except subprocess.TimeoutExpired:
        return result
    except Exception as e:
        return result


def write_summary(results, output_path, record_path, slice_interval, init_timeout, pvtlc_source="pvt"):
    """写入汇总报告"""
    summary_file = os.path.join(output_path, "init_eval_summary.txt")
    csv_file = os.path.join(output_path, "init_eval_results.csv")

    # 过滤掉跳过的切片
    valid_results = [r for r in results if not r.get("is_skipped", False)]
    skipped_count = len(results) - len(valid_results)

    total = len(valid_results)
    success = sum(1 for r in valid_results if r["is_success"])
    fail = total - success
    rate = success * 100.0 / total if total > 0 else 0.0

    init_times = [r["init_time"] for r in valid_results if r["init_time"] > 0]

    # 写入 CSV（包含跳过的切片）
    with open(csv_file, 'w') as f:
        f.write("# slice_id,start_ts,dynamic_ready,init_start,init_complete,init_time,is_success,is_skipped,gnss_count,imu_count\n")
        for r in results:
            f.write(f"{r['slice_id']},{r['slice_start']:.2f},{r['dynamic_ready']:.4f},"
                    f"{r['init_start']:.4f},{r['init_complete']:.4f},{r['init_time']:.4f},"
                    f"{r['is_success']},{r.get('is_skipped', False)},{r['gnss_count']},{r['imu_count']}\n")

    # 写入汇总
    gnss_topic = "/drivers/gnss/" + pvtlc_source
    with open(summary_file, 'w') as f:
        f.write("# TCMSF Slice Evaluation Summary\n")
        f.write(f"# Generated: {datetime.now().isoformat()}\n")
        f.write(f"# Record: {record_path}\n")
        f.write(f"# GNSS source: {pvtlc_source} ({gnss_topic})\n")
        f.write(f"# Slice interval: {slice_interval}s\n")
        f.write(f"# Timeout: {init_timeout}s\n")
        f.write("#------------------------------------------------------------\n\n")

        # 简洁表格
        f.write("Slice Results:\n")
        f.write("#------------------------------------------------------------\n")
        f.write(f"{'ID':<6} {'Start':<12} {'Status':<10} {'InitTime':<10}\n")
        f.write("------ ------------ ---------- ----------\n")

        for r in results:
            if r.get("is_skipped", False):
                status = "SKIPPED"
                init_t = "-"
            elif r["is_success"]:
                status = "SUCCESS"
                init_t = f"{r['init_time']:.2f}s" if r['init_time'] > 0 else "-"
            else:
                status = "FAILED"
                init_t = "-"
            f.write(f"{r['slice_id']:<6} {int(r['slice_start']):<12} {status:<10} {init_t:<10}\n")

        f.write("\n#------------------------------------------------------------\n")
        f.write("Statistics:\n")
        if skipped_count > 0:
            f.write(f"  Total slices: {len(results)} (valid: {total}, skipped: {skipped_count})\n")
        else:
            f.write(f"  Total slices: {total}\n")
        f.write(f"  Success: {success}\n")
        f.write(f"  Failed: {fail}\n")
        f.write(f"  Success rate: {rate:.1f}%\n\n")

        if init_times:
            avg = sum(init_times) / len(init_times)
            min_t = min(init_times)
            max_t = max(init_times)
            median = sorted(init_times)[len(init_times) // 2]
            f.write("Init Time Stats (successful):\n")
            f.write(f"  Average: {avg:.2f}s\n")
            f.write(f"  Min: {min_t:.2f}s\n")
            f.write(f"  Max: {max_t:.2f}s\n")
            f.write(f"  Median: {median:.2f}s\n")

    return summary_file


def main():
    if len(sys.argv) < 7:
        print("Usage: python3 tcmsf_init_evaluator_dispatcher_cloud.py "
              "record_path output_path start_timestamp slice_interval slice_count init_timeout [pvtlc_source]")
        print("\nArguments:")
        print("  pvtlc_source: GNSS data source ('pvt' or 'gnss', default: 'pvt')")
        sys.exit(1)

    record_path = sys.argv[1]
    output_path = sys.argv[2]
    start_timestamp_input = float(sys.argv[3])
    slice_interval = float(sys.argv[4])
    slice_count_input = int(sys.argv[5])
    init_timeout = float(sys.argv[6])
    pvtlc_source = sys.argv[7] if len(sys.argv) >= 8 else "pvt"

    if pvtlc_source not in ["pvt", "gnss"]:
        print(f"[WARN] Invalid pvtlc_source: {pvtlc_source}, using 'pvt'")
        pvtlc_source = "pvt"

    # 最小剩余时间阈值：至少需要这么多数据才能完成初始化
    MIN_DATA_TIME = 10.0  # 秒

    auto_mode = (slice_count_input == -1)
    os.makedirs(output_path, exist_ok=True)

    tcmsf_offline_dir = get_tcmsf_offline_dir()
    env = source_environment_variables(tcmsf_offline_dir)

    # 检测起始时间（不探测结束时间，依靠运行时的 consecutive_no_data 检测）
    if start_timestamp_input <= 0:
        print("Detecting record start time...")
        start_timestamp = get_record_start_time(record_path, tcmsf_offline_dir, env)
        if start_timestamp is None:
            print("Error: Cannot detect start time")
            sys.exit(1)
        print(f"Start timestamp: {start_timestamp:.2f}")
    else:
        start_timestamp = start_timestamp_input

    print(f"\n{'='*60}")
    print(f"Slice Evaluation: {record_path}")
    print(f"GNSS source: {pvtlc_source} ({'/drivers/gnss/' + pvtlc_source})")
    print(f"Start: {start_timestamp:.2f}, Interval: {slice_interval}s")
    print(f"Auto mode: {auto_mode}, will stop when no data for 3 consecutive slices")
    print(f"{'='*60}\n")

    results = []
    consecutive_no_data = 0
    i = 0

    while True:
        slice_start = start_timestamp + i * slice_interval

        if not auto_mode and i >= slice_count_input:
            break

        print(f"Slice {i} [{int(slice_start)}]...")

        result = run_single_slice(record_path, output_path, i, slice_start,
                                  tcmsf_offline_dir, env, pvtlc_source, init_timeout)

        if auto_mode:
            if result['gnss_count'] == 0 and result['imu_count'] == 0:
                consecutive_no_data += 1
                if consecutive_no_data >= 3:
                    print("No more data (3 consecutive slices without data).")
                    break
            else:
                consecutive_no_data = 0

        results.append(result)
        status = "OK" if result["is_success"] else "FAIL"
        init_t = f"{result['init_time']:.2f}s" if result['init_time'] > 0 else "-"
        print(f"  -> {status}, {init_t}")

        i += 1

    # 过滤无效结果（没有数据的切片）
    if auto_mode:
        results = [r for r in results if r['gnss_count'] > 0 or r['imu_count'] > 0]

    # 写入汇总（不传 end_timestamp）
    summary_file = write_summary(results, output_path, record_path, slice_interval, init_timeout, pvtlc_source)

    # 只统计有效切片（非跳过）
    valid_results = [r for r in results if not r.get('is_skipped', False)]
    success_count = sum(1 for r in valid_results if r['is_success'])
    rate = success_count * 100.0 / len(valid_results) if valid_results else 0
    skipped_count = len(results) - len(valid_results)

    print(f"\n{'='*60}")
    print(f"Done. Valid slices: {len(valid_results)}, Skipped: {skipped_count}")
    print(f"Success: {success_count}/{len(valid_results)} ({rate:.1f}%)")
    print(f"Summary: {summary_file}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()