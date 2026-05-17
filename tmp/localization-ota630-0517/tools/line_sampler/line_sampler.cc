/*
 * Copyright BYD ADAS App. All Rights Reserved.
 *
 * Line sampler tool: samples lines from a file at a specified interval.
 *
 * Usage:
 *   简略参数: line_sampler <input> <output> [sample_rate]
 *   完整参数: line_sampler --input=<path> --output=<path> [--sample_rate=<N>]
 *
 * Example:
 *   line_sampler data.txt sampled.txt 10
 *   line_sampler --input=data.txt --output=sampled.txt --sample_rate=10
 */

#include <fstream>
#include <iostream>
#include <string>

#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(input, "", "Input file path (简略参数: 第1个位置参数)");
DEFINE_string(output, "", "Output file path (简略参数: 第2个位置参数)");
DEFINE_int32(sample_rate, 10, "Sample rate: keep every Nth line (简略参数: 第3个位置参数, 默认: 10)");

void PrintUsage(const char *program) {
    std::cout << "用法:\n"
              << "  简略参数: " << program << " <input> <output> [sample_rate]\n"
              << "  完整参数: " << program << " --input=<path> --output=<path> [--sample_rate=<N>]\n"
              << "\n示例:\n"
              << "  " << program << " data.txt sampled.txt 10\n"
              << "  " << program << " --input=data.txt --output=sampled.txt --sample_rate=10\n"
              << "\n参数说明:\n"
              << "  input       输入文件路径\n"
              << "  output      输出文件路径\n"
              << "  sample_rate 采样频率, 每N行保留一行 (默认: 10)\n";
}

bool ParseArgs(int argc, char *argv[]) {
    // 检查是否有位置参数 (非 -- 开头的参数)
    int positional_count = 0;
    for (int i = 1; i < argc; ++i) {
        if (argv[i][0] != '-') {
            positional_count++;
        }
    }

    // 如果有位置参数，按顺序解析
    if (positional_count >= 2) {
        int pos_index = 0;
        for (int i = 1; i < argc; ++i) {
            if (argv[i][0] != '-') {
                pos_index++;
                if (pos_index == 1) {
                    FLAGS_input = argv[i];
                } else if (pos_index == 2) {
                    FLAGS_output = argv[i];
                } else if (pos_index == 3) {
                    FLAGS_sample_rate = std::stoi(argv[i]);
                }
            }
        }
    }

    // 验证必需参数
    if (FLAGS_input.empty()) {
        LOG(ERROR) << "缺少输入文件路径";
        PrintUsage(argv[0]);
        return false;
    }
    if (FLAGS_output.empty()) {
        LOG(ERROR) << "缺少输出文件路径";
        PrintUsage(argv[0]);
        return false;
    }
    if (FLAGS_sample_rate <= 0) {
        LOG(ERROR) << "采样频率必须为正整数";
        PrintUsage(argv[0]);
        return false;
    }
    return true;
}

int main(int argc, char *argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    if (!ParseArgs(argc, argv)) {
        return 1;
    }

    std::ifstream input_file(FLAGS_input);
    if (!input_file.is_open()) {
        LOG(ERROR) << "无法打开输入文件: " << FLAGS_input;
        return 1;
    }

    std::ofstream output_file(FLAGS_output);
    if (!output_file.is_open()) {
        LOG(ERROR) << "无法打开输出文件: " << FLAGS_output;
        input_file.close();
        return 1;
    }

    std::string line;
    int         line_count    = 0;
    int         sampled_count = 0;

    while (std::getline(input_file, line)) {
        line_count++;
        if (line_count % FLAGS_sample_rate == 0) {
            output_file << line << "\n";
            sampled_count++;
        }
    }

    input_file.close();
    output_file.close();

    LOG(INFO) << "采样完成. 总行数: " << line_count
              << ", 采样行数: " << sampled_count
              << ", 采样频率: " << FLAGS_sample_rate;

    return 0;
}