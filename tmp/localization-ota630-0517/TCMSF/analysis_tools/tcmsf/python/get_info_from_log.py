import re
import csv
import os
import sys
from typing import List, Optional

import argparse


def extract_comma_separated_floats(text: str, target_string: str) -> List[float]:
    """
    专门提取目标字符串后面的逗号分隔浮点数

    参数:
        text: 输入字符串
        target_string: 目标字符串

    返回:
        List[float]: 提取的浮点数列表
    """
    floats = []

    # 找到目标字符串的位置
    target_index = text.find(target_string)
    if target_index != -1:
        # 提取目标字符串后面的内容
        after_target = text[target_index + len(target_string) :]

        # 方法1: 使用正则表达式匹配浮点数（包括科学计数法）
        float_pattern = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"
        float_matches = re.findall(float_pattern, after_target)

        # 将匹配的字符串转换为浮点数

        for match in float_matches:
            try:
                float_value = float(match)
                floats.append(float_value)
            except ValueError:
                continue  # 如果转换失败，跳过这个匹配

    return floats


def write_floats_to_csv(data: List[List[float]], csv_file_path: str):
    """
    将浮点数数据写入CSV文件

    参数:
        data: 二维浮点数列表
        csv_file_path: CSV文件路径
    """
    with open(csv_file_path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)

        # 写入数据
        for row in data:
            writer.writerow(row)


def process_multiple_log_files(
    log_directory: str,
    target_string: str,
    output_csv: str,
    file_extension: str = ".log",
):
    """
    处理目录下的多个日志文件

    参数:
        log_directory: 日志文件目录
        target_string: 目标字符串
        output_csv: 输出CSV文件路径
        file_extension: 日志文件扩展名
    """
    all_data = []

    # 遍历目录下的所有文件[4](@ref)
    for filename in os.listdir(log_directory):
        if filename.endswith(file_extension):
            log_file_path = os.path.join(log_directory, filename)
            print(f"\n处理文件: {filename}")

            try:
                with open(log_file_path, "r", encoding="utf-8") as log_file:
                    for line_num, line in enumerate(log_file, 1):
                        if target_string in line:
                            floats = extract_comma_separated_floats(line, target_string)
                            if floats:
                                all_data.append(floats)
                                print(
                                    f"  {filename} 第 {line_num} 行: 找到 {len(floats)} 个浮点数"
                                )

            except Exception as e:
                print(f"  处理文件 {filename} 时出错: {str(e)}")

    # 写入CSV文件
    if all_data:
        write_floats_to_csv(all_data, output_csv)
        print(f"\n所有数据已写入: {output_csv}")
        print(f"总共处理了 {len(all_data)} 行数据")
    else:
        print("未找到任何匹配的数据")


def main():
    """主函数：解析命令行参数并处理日志文件"""
    # 创建命令行参数解析器 [1,6](@ref)
    parser = argparse.ArgumentParser(
        description="从日志文件中提取包含特定字符串的行中的逗号分隔浮点数，并保存到CSV文件",
        epilog='示例: python log_parser.py --input app.log --output data.csv --target "Speed:" --verbose',
    )

    # 添加必需的参数 [3,8](@ref)
    parser.add_argument(
        "--input", "-i", type=str, required=True, help="输入日志文件的路径"
    )

    parser.add_argument(
        "--output", "-o", type=str, required=True, help="输出CSV文件的路径"
    )

    parser.add_argument(
        "--target", "-t", type=str, required=True, help="要匹配的目标字符串"
    )

    parser.add_argument(
        "--extension", "-e", type=str, required=True, help="要匹配的目标字符串"
    )

    # 解析命令行参数 [5](@ref)
    args = parser.parse_args()

    # 处理日志文件
    process_multiple_log_files(
        log_directory=args.input,
        output_csv=args.output,
        target_string=args.target,
        file_extension=args.extension,
    )


if __name__ == "__main__":
    main()
