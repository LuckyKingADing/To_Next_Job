#!/usr/bin/env python3
"""
TCMSF 初始化评测工具 - 通过调用 TCMSF 可执行程序

特点：
1. 直接调用 bin/TCMSF 可执行程序，无需编译动态库链接
2. 通过捕获 glog 日志提取初始化时间点
3. 不支持切片测试，运行完整数据
4. 支持指定 GNSS 数据源 (pvt 或 gnss)

用法：
python3 tcmsf_init_evaluator_via_exec.py record_path output_path [options]

Options:
  --pvtlc-source <pvt|gnss>  指定 GNSS 数据源
                             'pvt': 使用 /drivers/gnss/pvt topic
                             'gnss': 使用 /drivers/gnss/raw topic
  --config <path>            IMU 配置文件路径

输出文件：
- init_result.json: 初始化结果
- tcmsf_output.log: TCMSF完整运行日志
"""

import subprocess
import sys
import os
import re
import json
import time
from datetime import datetime
from pathlib import Path


def get_tcmsf_offline_dir():
    """
    自动检测 tcmsf_offline 目录路径
    """
    script_path = Path(__file__).resolve()
    current_dir = script_path.parent

    # 向上查找 source_env.sh
    while current_dir != current_dir.parent:
        source_env_candidate = current_dir / "source_env.sh"
        if source_env_candidate.exists():
            return current_dir
        current_dir = current_dir.parent

    # 默认推断
    return script_path.parent.parent.parent


def source_environment_variables(tcmsf_offline_dir):
    """
    加载环境变量
    """
    env = os.environ.copy()
    source_env_path = tcmsf_offline_dir / "source_env.sh"

    if source_env_path.exists():
        try:
            command = "source {} && env".format(source_env_path)
            result = subprocess.run(
                command,
                shell=True,
                executable='/bin/bash',
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            for line in result.stdout.split('\n'):
                if '=' in line:
                    key, value = line.split('=', 1)
                    env[key] = value
        except subprocess.CalledProcessError as e:
            print("[警告] Source 环境变量失败: {}".format(e.stderr))
    else:
        lib_dir = tcmsf_offline_dir / "lib"
        if lib_dir.exists():
            env["LD_LIBRARY_PATH"] = str(lib_dir) + ":" + env.get("LD_LIBRARY_PATH", "")

    return env


def run_tcmsf_and_detect_init(record_path, output_path, tcmsf_offline_dir, env,
                              imu_config_path=None, pvtlc_source="pvt", timeout=600):
    """
    调用 TCMSF 可执行程序运行数据，检测初始化时间点

    Args:
        record_path: record 文件路径
        output_path: 输出路径
        tcmsf_offline_dir: tcmsf_offline 目录
        env: 环境变量
        imu_config_path: IMU 配置文件路径（可选）
        pvtlc_source: GNSS 数据源类型，"pvt" 或 "gnss"
        timeout: 超时时间（秒）

    Returns:
        dict: {
            'dynamic_ready_timestamp': float,
            'init_start_timestamp': float,
            'init_complete_timestamp': float,
            'total_init_time': float,
            'is_success': bool,
            'reinit_count': int,
            'reinit_events': list
        }
    """
    # TCMSF 可执行程序路径
    tcmsf_exec_path = tcmsf_offline_dir / "bin" / "TCMSF"

    if not tcmsf_exec_path.exists():
        print("[错误] TCMSF 可执行程序不存在: {}".format(tcmsf_exec_path))
        return None

    # 创建输出目录
    os.makedirs(output_path, exist_ok=True)

    # 构建命令
    cmd = [
        str(tcmsf_exec_path),
        record_path,
        output_path,
    ]

    # 添加 pvtlc-source 参数（指定 GNSS 数据源）
    cmd.append("--pvtlc-source")
    cmd.append(pvtlc_source)

    # 添加可选参数
    if imu_config_path:
        cmd.append("--config")
        cmd.append(imu_config_path)

    # 设置 glog 输出到 stderr
    env["GLOG_logtostderr"] = "1"

    print("\n" + "="*60)
    print("TCMSF Initialization Detection (via Executable)")
    print("="*60)
    print("TCMSF executable: {}".format(tcmsf_exec_path))
    print("Record path: {}".format(record_path))
    print("Output path: {}".format(output_path))
    print("GNSS source: {} ({})".format(pvtlc_source,
          "/drivers/gnss/pvt" if pvtlc_source == "pvt" else "/drivers/gnss/raw"))
    print("Timeout: {}s".format(timeout))
    print("="*60 + "\n")

    print("Running TCMSF...")
    start_time = time.time()

    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            cwd=str(tcmsf_offline_dir),
            env=env
        )

        elapsed = time.time() - start_time
        print("Completed in {:.1f} seconds".format(elapsed))

        # 保存完整日志
        output = proc.stdout + "\n" + proc.stderr
        log_file = os.path.join(output_path, "tcmsf_output.log")
        with open(log_file, 'w') as f:
            f.write("# TCMSF Output Log\n")
            f.write("# Record: {}\n".format(record_path))
            f.write("# Timestamp: {}\n".format(datetime.now().isoformat()))
            f.write("# Elapsed: {:.1f}s\n".format(elapsed))
            f.write("#" + "="*50 + "\n\n")
            f.write(output)

        # 解析日志，提取初始化时间点
        result = parse_init_logs(output)

        return result

    except subprocess.TimeoutExpired:
        print("Timeout expired (> {}s)".format(timeout))
        return {
            'dynamic_ready_timestamp': -1.0,
            'init_start_timestamp': -1.0,
            'init_complete_timestamp': -1.0,
            'total_init_time': -1.0,
            'is_success': False,
            'reinit_count': 0,
            'reinit_events': [],
            'error': 'timeout'
        }
    except Exception as e:
        print("Error: {}".format(e))
        return None


def parse_init_logs(log_content):
    """
    从 TCMSF 日志中解析初始化时间点

    关键日志格式：
    - [startup] Dynamic ready (gnss)! imu time: XXX
    - [startup] TCMSF Start Init! imu time: XXX
    - [startup] TCMSF Ready! imu time: XXX
    - reinit due to large pos error
    - reinit for sensor delay issue
    """
    dynamic_ready_regex = re.compile(r'\[startup\] Dynamic ready \(gnss\)! imu time:\s*([\d.]+)')
    start_init_regex = re.compile(r'\[startup\] TCMSF Start Init! imu time:\s*([\d.]+)')
    ready_regex = re.compile(r'\[startup\] TCMSF Ready! imu time:\s*([\d.]+)')
    reinit_regex = re.compile(r'reinit (due to large pos error|for sensor delay issue)')

    # 用于提取当前 IMU 时间
    imu_time_regex = re.compile(r'imu time:\s*([\d.]+)')

    result = {
        'dynamic_ready_timestamp': -1.0,
        'init_start_timestamp': -1.0,
        'init_complete_timestamp': -1.0,
        'total_init_time': -1.0,
        'is_success': False,
        'reinit_count': 0,
        'reinit_events': []
    }

    current_imu_time = 0.0
    reinit_count = 0

    for line in log_content.split('\n'):
        # 更新当前 IMU 时间
        imu_match = imu_time_regex.search(line)
        if imu_match:
            current_imu_time = float(imu_match.group(1))

        # 检测 Dynamic Ready
        match = dynamic_ready_regex.search(line)
        if match and result['dynamic_ready_timestamp'] < 0:
            result['dynamic_ready_timestamp'] = float(match.group(1))
            print("  [Dynamic ready] imu time: {:.4f}".format(result['dynamic_ready_timestamp']))

        # 检测 Start Init
        match = start_init_regex.search(line)
        if match and result['init_start_timestamp'] < 0:
            result['init_start_timestamp'] = float(match.group(1))
            print("  [Start Init] imu time: {:.4f}".format(result['init_start_timestamp']))

        # 检测 TCMSF Ready
        match = ready_regex.search(line)
        if match and result['init_complete_timestamp'] < 0:
            result['init_complete_timestamp'] = float(match.group(1))
            result['is_success'] = True
            print("  [TCMSF Ready] imu time: {:.4f}".format(result['init_complete_timestamp']))

        # 检测重初始化
        if reinit_regex.search(line):
            reinit_count += 1
            result['reinit_events'].append({
                'timestamp': current_imu_time,
                'count': reinit_count,
                'log': line.strip()
            })
            print("  [Reinit #{}] imu time: {:.4f}".format(reinit_count, current_imu_time))

    result['reinit_count'] = reinit_count

    # 计算总初始化时间
    if result['dynamic_ready_timestamp'] > 0 and result['init_complete_timestamp'] > 0:
        result['total_init_time'] = result['init_complete_timestamp'] - result['dynamic_ready_timestamp']

    return result


def write_result(result, output_path, record_path):
    """
    写入结果文件
    """
    result_file = os.path.join(output_path, "init_result.json")

    output_data = {
        'record_path': record_path,
        'timestamp': datetime.now().isoformat(),
        'result': result
    }

    with open(result_file, 'w') as f:
        json.dump(output_data, f, indent=2)

    # 也写入一个简单的文本报告
    report_file = os.path.join(output_path, "init_report.txt")
    with open(report_file, 'w') as f:
        f.write("# TCMSF Initialization Report\n")
        f.write("#" + "="*60 + "\n")
        f.write("# Record: {}\n".format(record_path))
        f.write("# Generated: {}\n".format(datetime.now().isoformat()))
        f.write("#" + "="*60 + "\n\n")

        if result['is_success']:
            f.write("Initialization: SUCCESS\n")
            f.write("Dynamic ready timestamp: {:.4f}\n".format(result['dynamic_ready_timestamp']))
            f.write("Init start timestamp: {:.4f}\n".format(result['init_start_timestamp']))
            f.write("Init complete timestamp: {:.4f}\n".format(result['init_complete_timestamp']))
            f.write("Total init time: {:.4f} seconds\n".format(result['total_init_time']))
        else:
            f.write("Initialization: FAILED\n")
            if result['dynamic_ready_timestamp'] > 0:
                f.write("Dynamic ready timestamp: {:.4f}\n".format(result['dynamic_ready_timestamp']))
            else:
                f.write("Dynamic ready: NOT DETECTED\n")

        f.write("\nReinit count: {}\n".format(result['reinit_count']))

        if result['reinit_events']:
            f.write("\n# Reinit Events:\n")
            for event in result['reinit_events']:
                f.write("  Reinit #{} at {:.4f}: {}\n".format(
                    event['count'], event['timestamp'], event['log']))

    return result_file


def main():
    if len(sys.argv) < 3:
        print("Usage: python3 tcmsf_init_evaluator_via_exec.py record_path output_path [options]")
        print("\nArguments:")
        print("  record_path      : record文件路径")
        print("  output_path      : 输出结果路径")
        print("\nOptions:")
        print("  --pvtlc-source <pvt|gnss>  指定 GNSS 数据源")
        print("                             'pvt' (默认): 使用 /drivers/gnss/pvt topic")
        print("                             'gnss': 使用 /drivers/gnss/raw topic")
        print("  --config <path>            IMU配置文件路径")
        print("\nExample:")
        print("  python3 tcmsf_init_evaluator_via_exec.py /data/record /data/output")
        print("  python3 tcmsf_init_evaluator_via_exec.py /data/record /data/output --pvtlc-source gnss")
        print("  python3 tcmsf_init_evaluator_via_exec.py /data/record /data/output --config imu_config.toml")
        sys.exit(1)

    record_path = sys.argv[1]
    output_path = sys.argv[2]

    # 解析命令行参数
    imu_config_path = None
    pvtlc_source = "pvt"  # 默认使用 pvt

    for i in range(3, len(sys.argv)):
        arg = sys.argv[i]
        if arg == "--pvtlc-source" and i + 1 < len(sys.argv):
            pvtlc_source = sys.argv[i + 1]
            i += 1
            if pvtlc_source not in ["pvt", "gnss"]:
                print("[警告] 无效的 pvtlc-source 值: {}, 使用默认 'pvt'".format(pvtlc_source))
                pvtlc_source = "pvt"
        elif arg == "--config" and i + 1 < len(sys.argv):
            imu_config_path = sys.argv[i + 1]
            i += 1
        elif not arg.startswith("--"):
            # 兼容旧格式：第三个参数是 config 路径（不带 --config 前缀）
            imu_config_path = arg

    # 自动检测 tcmsf_offline 目录
    tcmsf_offline_dir = get_tcmsf_offline_dir()
    print("[信息] tcmsf_offline 目录: {}".format(tcmsf_offline_dir))
    print("[信息] GNSS 数据源: {} ({})".format(pvtlc_source,
          "/drivers/gnss/pvt" if pvtlc_source == "pvt" else "/drivers/gnss/raw"))

    # 加载环境变量
    env = source_environment_variables(tcmsf_offline_dir)

    # 运行 TCMSF 并检测初始化
    result = run_tcmsf_and_detect_init(
        record_path, output_path, tcmsf_offline_dir, env,
        imu_config_path, pvtlc_source
    )

    if result:
        # 写入结果
        result_file = write_result(result, output_path, record_path)

        print("\n" + "="*60)
        print("Initialization Detection Complete")
        print("="*60)
        if result['is_success']:
            print("Result: SUCCESS")
            print("Total init time: {:.4f} seconds".format(result['total_init_time']))
        else:
            print("Result: FAILED")
            if 'error' in result:
                print("Error: {}".format(result['error']))
        print("Reinit count: {}".format(result['reinit_count']))
        print("Result file: {}".format(result_file))
        print("Log file: {}".format(os.path.join(output_path, "tcmsf_output.log")))
        print("="*60)
    else:
        print("\n[错误] 运行失败")


if __name__ == "__main__":
    main()