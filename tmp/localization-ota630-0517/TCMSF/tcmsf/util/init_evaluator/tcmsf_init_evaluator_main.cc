/**
 * TCMSF 初始化成功率/耗时测试工具
 *
 * 功能：
 * 1. 耗时测试：通过固定间隔切片方法，统计初始化耗时
 * 2. 成功率测试：用难例数据做切片测试，检查是否成功初始化
 * 3. 使用非静态带卫星定位的数据（通过GNSS速度判断）
 * 4. 从TCMSF内部日志中精确提取初始化时间点
 *
 * 关键日志（TCMSF内部）：
 * - [startup] Dynamic ready (gnss)! imu time: XXX
 * - [startup] TCMSF Start Init! imu time: XXX
 * - [startup] TCMSF Ready! imu time: XXX
 *
 * 用法：
 * tcmsf_init_evaluator record_path output_path start_timestamp slice_interval slice_count slice_duration [imu_config_path]
 */

#include "Coord.h"
#include "cyber/cyber.h"
#include "cyber/record/record_reader.h"
#include "fmt/format.h"
#include "glog/logging.h"
#include "modules/msg/localization_msgs/vf_result.pb.h"
#include "modules/msg/localization_msgs/sd_map_match.pb.h"
#include "tcmsf_interface.h"
#include "init_state_recorder.h"
#include "tcmsf_config.h"

#include <fstream>
#include <iostream>
#include <string>
#include <filesystem>
#include <algorithm>
#include <sstream>
#include <regex>
#include <memory>
#include <iomanip>

#define PVT_TPC "/drivers/gnss/pvt"
#define GPS_TPC "/drivers/gnss/raw"
#define IMU_TPC "/drivers/imu/raw"
#define VEH_TPC "/drivers/canbus/vehicle_info"
#define DR_TPC "/localization/dr"
#define VF_TPC "/localization/vf/vf_result"
#define SDMM_TPC "/localization/sd_mapmatch_result"

using byd::modules::loc_vf::VFResult;
using byd::modules::localization::SDMapMatchResult;
using byd::modules::tcmsf::Pose;
using byd::msg::drivers::Gps;
using byd::msg::drivers::Imu;
using byd::msg::drivers::VehInfo;
using byd::msg::localization::LocalizationEstimate;
using namespace byd::tcmsf::eval;

struct EvalConfig {
    std::string record_path;
    std::string output_path;
    std::string imu_config_path;
    double start_timestamp = 0.0;
    double slice_interval = 40.0;
    int slice_count = 20;
    double slice_duration = 60.0;
    double init_timeout = 300.0;  // 5分钟超时
    int slice_id = 0;  // 当前切片编号（由dispatcher传入）
    bool continue_after_init = false;  // 初始化完成后是否继续运行（用于重初始化检测）
    std::string pvtlc_source = "pvt";  // GNSS数据源: "pvt" 或 "gnss"
};

// 自定义日志 Sink，用于捕获 TCMSF 的关键日志
class InitLogSink : public google::LogSink {
public:
    InitLogSink(int slice_id, std::ofstream& log_file, InitStateRecorder& recorder, bool continue_mode = false)
        : slice_id_(slice_id), log_file_(log_file), recorder_(recorder), last_imu_time_(0.0), continue_mode_(continue_mode) {}

    void send(google::LogSeverity severity, const char* full_filename,
              const char* base_filename, int line,
              const struct ::tm* tm_time,
              const char* message, size_t message_len) override {

        std::string msg(message, message_len);

        // TCMSF 内部日志格式：
        // [startup] Dynamic ready (gnss)! imu time:    1775804340.6329
        // [startup] TCMSF Start Init! imu time:    1775804341.1316
        // [startup] TCMSF Ready! imu time:    1775804342.1407
        // 注意：fmt::format("{:14.4f}") 会产生14字符宽度，前面可能有空格

        std::regex dynamic_ready_regex("\\[startup\\] Dynamic ready \\(gnss\\)! imu time:\\s*([\\d.]+)");
        std::regex start_init_regex("\\[startup\\] TCMSF Start Init! imu time:\\s*([\\d.]+)");
        std::regex ready_regex("\\[startup\\] TCMSF Ready! imu time:\\s*([\\d.]+)");
        // 重初始化日志格式：
        // 1. "reinit due to large pos error" - gps_single_processor.cpp:71
        // 2. "reinit for sensor delay issue" - processor_interface_impl.cpp:373
        std::regex reinit_regex("reinit (due to large pos error|for sensor delay issue)");

        std::smatch match;

        if (std::regex_search(msg, match, dynamic_ready_regex) && !dynamic_ready_recorded_) {
            double imu_time = std::stod(match[1].str());
            recorder_.record_dynamic_ready(imu_time);
            dynamic_ready_recorded_ = true;
            last_imu_time_ = imu_time;
            if (!continue_mode_) {
                log_file_ << "[Slice " << slice_id_ << "][startup] Dynamic ready (gnss)! imu time: "
                          << std::fixed << std::setprecision(4) << imu_time << "\n";
                std::cout << "Slice " << slice_id_ << ": Dynamic ready at " << std::fixed << std::setprecision(4) << imu_time << std::endl;
            }
        }

        if (std::regex_search(msg, match, start_init_regex) && !start_init_recorded_) {
            double imu_time = std::stod(match[1].str());
            recorder_.record_init_start(imu_time);
            start_init_recorded_ = true;
            last_imu_time_ = imu_time;
            if (!continue_mode_) {
                log_file_ << "[Slice " << slice_id_ << "][startup] TCMSF Start Init! imu time: "
                          << std::fixed << std::setprecision(4) << imu_time << "\n";
                std::cout << "Slice " << slice_id_ << ": Start Init at " << std::fixed << std::setprecision(4) << imu_time << std::endl;
            }
        }

        if (std::regex_search(msg, match, ready_regex) && !ready_recorded_) {
            double imu_time = std::stod(match[1].str());
            recorder_.record_init_complete(imu_time);
            recorder_.record_aligned(imu_time);
            ready_recorded_ = true;
            init_completed_ = true;
            last_imu_time_ = imu_time;
            if (!continue_mode_) {
                log_file_ << "[Slice " << slice_id_ << "][startup] TCMSF Ready! imu time: "
                          << std::fixed << std::setprecision(4) << imu_time << "\n";
                std::cout << "Slice " << slice_id_ << ": Ready at " << std::fixed << std::setprecision(4) << imu_time << std::endl;
            }
        }

        // 检测重初始化
        if (std::regex_search(msg, reinit_regex)) {
            recorder_.record_reinit(last_imu_time_);
            if (!continue_mode_) {
                log_file_ << "[Slice " << slice_id_ << "][reinit] Detected at imu time: "
                          << std::fixed << std::setprecision(4) << last_imu_time_ << "\n";
                std::cout << "Slice " << slice_id_ << ": Reinit detected at " << std::fixed << std::setprecision(4) << last_imu_time_ << std::endl;
            }
        }
    }

    bool is_dynamic_ready() const { return dynamic_ready_recorded_; }
    bool is_init_completed() const { return init_completed_; }
    bool is_start_init_recorded() const { return start_init_recorded_; }
    int get_reinit_count() const { return recorder_.get_result().reinit_count; }

    // 更新 last_imu_time（从外部调用）
    void update_imu_time(double imu_time) { last_imu_time_ = imu_time; }

private:
    int slice_id_;
    std::ofstream& log_file_;
    InitStateRecorder& recorder_;
    bool dynamic_ready_recorded_ = false;
    bool start_init_recorded_ = false;
    bool ready_recorded_ = false;
    bool init_completed_ = false;
    double last_imu_time_ = 0.0;  // 用于记录重初始化发生时的IMU时间
    bool continue_mode_ = false;  // 重初始化检测模式，不输出Slice日志
};

class SliceTestRunner {
public:
    SliceTestRunner(const EvalConfig& config, int slice_id, double slice_start, double slice_end, std::ofstream& log_file)
        : config_(config), slice_id_(slice_id), slice_start_(slice_start), slice_end_(slice_end), log_file_(log_file) {
        recorder_.set_slice_info(slice_id, slice_start, slice_end);
    }

    InitTestResult run() {
        auto tcmsf = byd::tcmsf::TCMSF::create(config_.imu_config_path);

        auto gps_ = std::make_shared<Gps>();
        auto imu_ = std::make_shared<Imu>();
        auto veh_ = std::make_shared<VehInfo>();
        auto dr_ = std::make_shared<LocalizationEstimate>();
        auto vf_ = std::make_shared<VFResult>();
        auto sdmm_ = std::make_shared<SDMapMatchResult>();

        auto& parameters_sgt = byd::tcmsf::config::Parameters::getInstance(byd::tcmsf::config::TCMSF_CONFIG_FILE_DIR_);

        // 创建自定义日志 sink 来捕获 TCMSF 的关键日志
        InitLogSink log_sink(slice_id_, log_file_, recorder_, config_.continue_after_init);
        google::AddLogSink(&log_sink);

        // 写入切片开始的日志（重初始化检测模式不输出）
        if (!config_.continue_after_init) {
            log_file_ << "[Slice " << slice_id_ << "] Start at timestamp: " << std::fixed << std::setprecision(0) << slice_start_ << "\n";
            std::cout << "=== Slice " << slice_id_ << " [" << std::fixed << std::setprecision(0) << slice_start_ << "-" << slice_end_ << "] ===" << std::endl;
        }

        std::vector<std::string> records;
        if (std::filesystem::is_directory(config_.record_path)) {
            for (auto& iter : std::filesystem::directory_iterator(config_.record_path)) {
                std::string path_str = iter.path().string();
                if (path_str.find(".record") != std::string::npos &&
                    path_str.find(".log") == std::string::npos) {
                    records.push_back(path_str);
                }
            }
            std::sort(records.begin(), records.end());
        } else {
            records.push_back(config_.record_path);
        }

        bool slice_started = false;

        auto result_cb_ = [&]() {
            auto result_msg = std::make_shared<Pose>();
            tcmsf->output_msg(result_msg);
        };

        // 记录开始测试的wall clock时间，用于超时判断
        double test_start_time = 0;

        for (const auto& record_path : records) {
            apollo::cyber::record::RecordReader reader(record_path);
            apollo::cyber::record::RecordMessage msg_;
            while (reader.ReadMessage(&msg_)) {
                double msg_time = msg_.time / 1e9;

                // 只跳过slice_start之前的数据，不限制slice_end
                if (msg_time < slice_start_) continue;
                slice_started = true;

                if (msg_.channel_name == GPS_TPC) {
                    // 根据 pvtlc_source 参数决定是否使用 gnss/raw 数据
                    // "gnss" -> 强制使用 GPS_TPC
                    // "pvt"  -> 跳过 GPS_TPC，使用 PVT_TPC
                    bool use_gps_raw = (config_.pvtlc_source == "gnss") ||
                        (config_.pvtlc_source == "pvt" &&
                         parameters_sgt.get_gnss_fusion_mode() !=
                             byd::tcmsf::config::Parameters::GnssFusionMode::GNSS_LOOSE_COUPLE);

                    if (use_gps_raw) {
                        gps_->ParseFromString(msg_.content);

                        double speed = std::sqrt(
                            gps_->linear_velocity().x() * gps_->linear_velocity().x() +
                            gps_->linear_velocity().y() * gps_->linear_velocity().y() +
                            gps_->linear_velocity().z() * gps_->linear_velocity().z());

                        recorder_.set_start_position(gps_->position().lat(), gps_->position().lon(), speed);

                        recorder_.increment_gnss_count();
                        int pos_status = static_cast<int>(gps_->position_status());
                        if (pos_status == 6) recorder_.increment_gnss_fix_count();
                        else if (pos_status == 5) recorder_.increment_gnss_float_count();

                        double lat_mars = 0.0, lon_mars = 0.0;
                        wgtochina_lb(0, gps_->position().lon(), gps_->position().lat(),
                                    gps_->position().height(), 0, 0, &lon_mars, &lat_mars);
                        gps_->mutable_position()->set_lon(lon_mars);
                        gps_->mutable_position()->set_lat(lat_mars);
                        tcmsf->insert_msg(gps_);
                    }
                }

                if (msg_.channel_name == PVT_TPC) {
                    // 根据 pvtlc_source 参数决定是否使用 gnss/pvt 数据
                    // "pvt"  -> 强制使用 PVT_TPC
                    // "gnss" -> 跳过 PVT_TPC，使用 GPS_TPC
                    bool use_pvt = (config_.pvtlc_source == "pvt") ||
                        (config_.pvtlc_source == "gnss" &&
                         parameters_sgt.get_gnss_fusion_mode() ==
                             byd::tcmsf::config::Parameters::GnssFusionMode::GNSS_LOOSE_COUPLE);

                    if (use_pvt) {
                        gps_->ParseFromString(msg_.content);

                        double speed = std::sqrt(
                            gps_->linear_velocity().x() * gps_->linear_velocity().x() +
                            gps_->linear_velocity().y() * gps_->linear_velocity().y() +
                            gps_->linear_velocity().z() * gps_->linear_velocity().z());

                        recorder_.set_start_position(gps_->position().lat(), gps_->position().lon(), speed);

                        recorder_.increment_gnss_count();
                        int pos_status = static_cast<int>(gps_->position_status());
                        if (pos_status == 6) recorder_.increment_gnss_fix_count();
                        else if (pos_status == 5) recorder_.increment_gnss_float_count();

                        tcmsf->insert_msg(gps_);
                    }
                }

                if (msg_.channel_name == VEH_TPC) {
                    veh_->ParseFromString(msg_.content);
                    tcmsf->insert_msg(veh_);
                }
                if (msg_.channel_name == DR_TPC) {
                    dr_->ParseFromString(msg_.content);
                    tcmsf->insert_msg(dr_);
                }
                if (msg_.channel_name == SDMM_TPC) {
                    sdmm_->ParseFromString(msg_.content);
                    tcmsf->insert_msg(sdmm_);
                }
                if (msg_.channel_name == VF_TPC) {
                    vf_->ParseFromString(msg_.content);
                    tcmsf->insert_msg(vf_);
                }

                if (msg_.channel_name == IMU_TPC) {
                    imu_->ParseFromString(msg_.content);
                    recorder_.increment_imu_count();
                    log_sink.update_imu_time(msg_time);  // 更新当前 IMU 时间用于重初始化检测
                    tcmsf->insert_msg(imu_);
                    tcmsf->offline_mode_step(result_cb_);

                    double current_time = msg_time;
                    double elapsed_seconds = current_time - test_start_time;
                    if (elapsed_seconds > config_.init_timeout && test_start_time > 1) {
                        if (!config_.continue_after_init) {
                            std::cout << "Slice " << slice_id_ << ": Timeout (elapsed " << elapsed_seconds << "s)" << std::endl;
                                //<< std::fixed << std::setprecision(9) << current_time << "," << test_start_time << std::endl;
                        }
                        recorder_.mark_failed(msg_time);
                        break;
                    }

                    if (log_sink.is_dynamic_ready() && !config_.continue_after_init && test_start_time < 1){
                        test_start_time = msg_time;
                    }

                    // 检查是否完成初始化
                    if (log_sink.is_init_completed() && !config_.continue_after_init) {
                        if (!config_.continue_after_init) {
                            std::cout << "Slice " << slice_id_ << ": Init completed!" << std::endl;
                        }
                        break;
                    }
                }
            }
            // 只有在非continue模式下才在初始化完成或失败时停止
            if (!config_.continue_after_init) {
                if (recorder_.get_result().final_state == InitState::INIT_FAILED || log_sink.is_init_completed()) break;
            }
        }

        // 移除日志 sink
        google::RemoveLogSink(&log_sink);

        if (!slice_started) {
            recorder_.mark_failed(slice_start_);
        }

        auto result = recorder_.get_result();
        if (!config_.continue_after_init) {
            log_file_ << "[Slice " << slice_id_ << "] Result: " << (result.is_success ? "SUCCESS" : "FAILED")
                      << ", total_init_time=" << result.total_init_time << "s\n\n";
        }

        return result;
    }

private:
    const EvalConfig& config_;
    int slice_id_;
    double slice_start_;
    double slice_end_;
    InitStateRecorder recorder_;
    std::ofstream& log_file_;
};

int main(int argc, char** argv) {
    if (argc < 7) {
        std::cout << "Usage: tcmsf_init_evaluator record_path output_path start_timestamp "
                  << "slice_interval slice_count slice_duration [options]\n";
        std::cout << "\nOptions:\n";
        std::cout << "  --pvtlc-source <pvt|gnss>  GNSS data source (default: pvt)\n";
        std::cout << "                             'pvt': use /drivers/gnss/pvt topic\n";
        std::cout << "                             'gnss': use /drivers/gnss/raw topic\n";
        std::cout << "  --config <path>            IMU config file path\n";
        std::cout << "  --slice-id <id>            Slice ID (for dispatcher)\n";
        std::cout << "\nExample:\n";
        std::cout << "  tcmsf_init_evaluator /data/record /data/output 1774324300 40 20 60\n";
        std::cout << "  tcmsf_init_evaluator /data/record /data/output 1774324300 40 20 60 --pvtlc-source gnss\n";
        return -1;
    }

    // 初始化 glog - 必须在使用前初始化
    google::InitGoogleLogging(argv[0]);
    // 设置 glog 输出到 stderr，便于捕获完整的 TCMSF 内部日志
    FLAGS_logtostderr = true;

    EvalConfig config;
    config.record_path = argv[1];
    config.output_path = argv[2];
    config.start_timestamp = std::stod(argv[3]);
    config.slice_interval = std::stod(argv[4]);
    config.slice_count = std::stoi(argv[5]);
    config.slice_duration = std::stod(argv[6]);

    // 解析可选参数
    for (int i = 7; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--pvtlc-source" && i + 1 < argc) {
            config.pvtlc_source = argv[i + 1];
            i++;
            if (config.pvtlc_source != "pvt" && config.pvtlc_source != "gnss") {
                std::cout << "[WARN] Invalid pvtlc-source: " << config.pvtlc_source << ", using 'pvt'\n";
                config.pvtlc_source = "pvt";
            }
            std::cout << "[INFO] pvtlc-source: " << config.pvtlc_source << "\n";
        } else if (arg == "--config" && i + 1 < argc) {
            config.imu_config_path = argv[i + 1];
            i++;
        } else if (arg == "--slice-id" && i + 1 < argc) {
            config.slice_id = std::stoi(argv[i + 1]);
            i++;
        } else if (arg == "--continue-after-init" && i + 1 < argc) {
            config.continue_after_init = (std::stoi(argv[i + 1]) == 1);
            i++;
        } else if (arg.find("--") != 0) {
            // 兼容旧格式：第7个参数是 imu_config_path（不带--config前缀）
            if (config.imu_config_path.empty()) {
                config.imu_config_path = arg;
            } else if (i + 1 < argc && config.slice_id == 0) {
                // 第8个参数是 slice_id
                config.slice_id = std::stoi(arg);
            }
        }
    }

    if (!std::filesystem::exists(config.output_path)) {
        std::filesystem::create_directories(config.output_path);
    }

    std::string results_file = config.output_path + "/init_eval_results.csv";
    std::string log_file_path = config.output_path + "/init_eval_log.txt";

    std::ofstream log_file(log_file_path, std::ios::out);
    if (!log_file.is_open()) {
        std::cerr << "Failed to open log file: " << log_file_path << std::endl;
        google::ShutdownGoogleLogging();
        return -1;
    }

    // 只在单切片模式下写入简化日志（dispatcher调用时slice_count=1）
    bool single_slice_mode = (config.slice_count == 1);

    if (!single_slice_mode) {
        // 多切片模式：写入完整表头
        log_file << "# TCMSF Initialization Evaluation Log\n";
        log_file << "# Record path: " << config.record_path << "\n";
        log_file << "# Start timestamp: " << std::fixed << std::setprecision(0) << config.start_timestamp << "\n";
        log_file << "# Slice interval: " << config.slice_interval << "s\n";
        log_file << "# Slice count: " << config.slice_count << "\n";
        log_file << "# Slice duration: " << config.slice_duration << "s\n";
        log_file << "# Init timeout: " << config.init_timeout << "s\n";
        log_file << "# =====================================================\n\n";
    }

    InitTestResultWriter writer(results_file);
    std::vector<InitTestResult> all_results;

    for (int i = 0; i < config.slice_count; i++) {
        int current_slice_id = single_slice_mode ? config.slice_id : i;
        double slice_start = config.start_timestamp + i * config.slice_interval;
        double slice_end = slice_start + config.slice_duration;

        SliceTestRunner runner(config, current_slice_id, slice_start, slice_end, log_file);
        auto result = runner.run();

        writer.write_result(result);
        all_results.push_back(result);

        if (!config.continue_after_init) {
            std::cout << "Slice " << current_slice_id << " Result: " << (result.is_success ? "SUCCESS" : "FAILED")
                      << ", init_time=" << std::fixed << std::setprecision(4) << result.total_init_time << "s\n";
        }
    }

    log_file.close();

    // 只在多切片模式下生成summary文件
    if (!single_slice_mode) {
        std::string summary_file = config.output_path + "/init_eval_summary.txt";
        writer.write_summary(all_results, summary_file);

        std::cout << "\n========================================\n";
        std::cout << "Evaluation Complete\n";
        std::cout << "Results CSV: " << results_file << "\n";
        std::cout << "Summary: " << summary_file << "\n";
        std::cout << "Log file: " << log_file_path << "\n";

        int success_count = 0;
        for (const auto& r : all_results) if (r.is_success) success_count++;
        std::cout << "Success rate: " << std::fixed << std::setprecision(1)
                  << (success_count * 100.0 / config.slice_count) << "%\n";
        std::cout << "========================================\n";
    }

    google::ShutdownGoogleLogging();

    return 0;
}