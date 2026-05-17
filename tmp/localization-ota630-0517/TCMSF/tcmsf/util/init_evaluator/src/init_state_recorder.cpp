/**
 * 初始化状态记录器实现
 */

#include "init_state_recorder.h"

namespace byd {
namespace tcmsf {
namespace eval {

// InitTestResult 方法实现

void InitTestResult::calculate_timing() {
    if (dynamic_ready_timestamp > 0 && slice_start_timestamp > 0) {
        time_to_dynamic_ready = dynamic_ready_timestamp - slice_start_timestamp;
    }
    if (init_start_timestamp > 0 && dynamic_ready_timestamp > 0) {
        time_to_init_start = init_start_timestamp - dynamic_ready_timestamp;
    }
    // 总初始化耗时 = TCMSF Ready! time - Dynamic Ready! time
    // 包含：等待运动条件满足 + TCMSF内部初始化
    if (init_complete_timestamp > 0 && dynamic_ready_timestamp > 0) {
        total_init_time = init_complete_timestamp - dynamic_ready_timestamp;
    }
    is_success = (final_state == InitState::INIT_COMPLETED);
}

// InitStateRecorder 方法实现

InitStateRecorder::InitStateRecorder() {
    reset();
}

InitStateRecorder::~InitStateRecorder() = default;

void InitStateRecorder::reset() {
    current_result_ = InitTestResult();
    dynamic_ready_recorded_ = false;
    init_start_recorded_ = false;
    init_complete_recorded_ = false;
    aligned_reached_ = false;
    first_gnss_recorded_ = false;
}

void InitStateRecorder::set_slice_info(int slice_id, double start_ts, double end_ts) {
    current_result_.slice_id = slice_id;
    current_result_.slice_start_timestamp = start_ts;
    current_result_.slice_end_timestamp = end_ts;
}

void InitStateRecorder::record_dynamic_ready(double timestamp) {
    if (!dynamic_ready_recorded_) {
        current_result_.dynamic_ready_timestamp = timestamp;
        current_result_.final_state = InitState::DYNAMIC_READY;
        dynamic_ready_recorded_ = true;
    }
}

void InitStateRecorder::record_init_start(double timestamp) {
    if (!init_start_recorded_ && dynamic_ready_recorded_) {
        current_result_.init_start_timestamp = timestamp;
        current_result_.final_state = InitState::INIT_STARTED;
        init_start_recorded_ = true;
    }
}

void InitStateRecorder::record_init_complete(double timestamp) {
    if (!init_complete_recorded_ && init_start_recorded_) {
        current_result_.init_complete_timestamp = timestamp;
        current_result_.final_state = InitState::INIT_COMPLETED;
        init_complete_recorded_ = true;
        current_result_.calculate_timing();
    }
}

void InitStateRecorder::record_reinit(double timestamp) {
    current_result_.has_reinit = true;
    current_result_.reinit_count++;
    init_complete_recorded_ = false;
    current_result_.init_complete_timestamp = -1.0;
    current_result_.total_init_time = -1.0;
    current_result_.final_state = InitState::REINIT_OCCURRED;
}

void InitStateRecorder::record_aligned(double timestamp) {
    if (!aligned_reached_) {
        current_result_.first_aligned_timestamp = timestamp;
        aligned_reached_ = true;
    }
}

void InitStateRecorder::update_state(InitState state) {
    if (state > current_result_.final_state ||
        current_result_.final_state == InitState::REINIT_OCCURRED) {
        current_result_.final_state = state;
    }
}

void InitStateRecorder::mark_failed(double timestamp) {
    current_result_.final_state = InitState::INIT_FAILED;
    current_result_.calculate_timing();
}

void InitStateRecorder::increment_gnss_count() {
    current_result_.gnss_count++;
}

void InitStateRecorder::increment_imu_count() {
    current_result_.imu_count++;
}

void InitStateRecorder::increment_gnss_fix_count() {
    current_result_.gnss_fix_count++;
}

void InitStateRecorder::increment_gnss_float_count() {
    current_result_.gnss_float_count++;
}

void InitStateRecorder::set_start_position(double lat, double lon, double speed) {
    if (!first_gnss_recorded_) {
        current_result_.start_lat = lat;
        current_result_.start_lon = lon;
        current_result_.start_speed = speed;
        first_gnss_recorded_ = true;
    }
}

InitTestResult InitStateRecorder::get_result() const {
    InitTestResult result = current_result_;
    result.calculate_timing();
    return result;
}

bool InitStateRecorder::check_timeout(double current_timestamp, double timeout_threshold) {
    if (current_result_.final_state == InitState::NOT_STARTED ||
        current_result_.final_state == InitState::DYNAMIC_READY ||
        current_result_.final_state == InitState::INIT_STARTED ||
        current_result_.final_state == InitState::REINIT_OCCURRED) {
        double elapsed = current_timestamp - current_result_.slice_start_timestamp;
        if (elapsed > timeout_threshold && !init_complete_recorded_) {
            mark_failed(current_timestamp);
            return true;
        }
    }
    return false;
}

bool InitStateRecorder::is_init_completed() const {
    return init_complete_recorded_;
}

// InitTestResultWriter 方法实现

InitTestResultWriter::InitTestResultWriter(const std::string& output_path)
    : output_path_(output_path) {
    file_.open(output_path, std::ios::out);
    if (file_.is_open()) {
        write_header();
    }
}

InitTestResultWriter::~InitTestResultWriter() {
    if (file_.is_open()) {
        file_.close();
    }
}

bool InitTestResultWriter::is_open() const {
    return file_.is_open();
}

void InitTestResultWriter::write_header() {
    file_ << "# TCMSF Initialization Evaluation Results\n";
    file_ << "# slice_id,slice_start_ts,slice_end_ts,"
          << "dynamic_ready_ts,init_start_ts,init_complete_ts,"
          << "time_to_dynamic_ready,time_to_init_start,total_init_time,"
          << "final_state,is_success,has_reinit,reinit_count,"
          << "first_aligned_ts,start_lat,start_lon,start_speed,"
          << "gnss_count,imu_count,gnss_fix_count,gnss_float_count\n";
}

void InitTestResultWriter::write_result(const InitTestResult& result) {
    file_ << std::fixed << std::setprecision(6);
    file_ << result.slice_id << ","
          << result.slice_start_timestamp << ","
          << result.slice_end_timestamp << ","
          << result.dynamic_ready_timestamp << ","
          << result.init_start_timestamp << ","
          << result.init_complete_timestamp << ","
          << result.time_to_dynamic_ready << ","
          << result.time_to_init_start << ","
          << result.total_init_time << ","
          << static_cast<int>(result.final_state) << ","
          << (result.is_success ? 1 : 0) << ","
          << (result.has_reinit ? 1 : 0) << ","
          << result.reinit_count << ","
          << result.first_aligned_timestamp << ","
          << result.start_lat << ","
          << result.start_lon << ","
          << result.start_speed << ","
          << result.gnss_count << ","
          << result.imu_count << ","
          << result.gnss_fix_count << ","
          << result.gnss_float_count << "\n";
}

void InitTestResultWriter::write_summary(const std::vector<InitTestResult>& results,
                                         const std::string& summary_path) {
    std::ofstream summary_file(summary_path, std::ios::out);
    if (!summary_file.is_open()) {
        return;
    }

    int total = results.size();
    int success_count = 0;
    int fail_count = 0;
    int reinit_count = 0;

    std::vector<double> init_times;
    std::vector<double> dynamic_ready_times;

    for (const auto& r : results) {
        if (r.is_success) {
            success_count++;
            if (r.total_init_time > 0) {
                init_times.push_back(r.total_init_time);
            }
            if (r.time_to_dynamic_ready > 0) {
                dynamic_ready_times.push_back(r.time_to_dynamic_ready);
            }
        } else {
            fail_count++;
        }
        if (r.has_reinit) {
            reinit_count++;
        }
    }

    double success_rate = (total > 0) ? (success_count * 100.0 / total) : 0.0;

    double avg_init_time = 0.0;
    double min_init_time = 0.0;
    double max_init_time = 0.0;
    double median_init_time = 0.0;

    if (!init_times.empty()) {
        for (double t : init_times) {
            avg_init_time += t;
        }
        avg_init_time /= init_times.size();

        min_init_time = *std::min_element(init_times.begin(), init_times.end());
        max_init_time = *std::max_element(init_times.begin(), init_times.end());

        std::sort(init_times.begin(), init_times.end());
        median_init_time = init_times[init_times.size() / 2];
    }

    double avg_dynamic_ready_time = 0.0;
    if (!dynamic_ready_times.empty()) {
        for (double t : dynamic_ready_times) {
            avg_dynamic_ready_time += t;
        }
        avg_dynamic_ready_time /= dynamic_ready_times.size();
    }

    summary_file << "# TCMSF Initialization Evaluation Summary\n";
    summary_file << "# =====================================================\n";
    summary_file << "Total slices tested: " << total << "\n";
    summary_file << "Successful initializations: " << success_count << "\n";
    summary_file << "Failed initializations: " << fail_count << "\n";
    summary_file << "Success rate: " << std::fixed << std::setprecision(2)
                 << success_rate << "%\n";
    summary_file << "\n";
    summary_file << "# Timing Statistics (successful cases)\n";
    summary_file << "# Note: total_init_time = TCMSF Ready! time - slice start time\n";
    summary_file << "#       (includes data reading, waiting for Dynamic Ready, and TCMSF internal init)\n";
    summary_file << "Average time to dynamic ready: " << std::fixed
                 << std::setprecision(2) << avg_dynamic_ready_time << " seconds\n";
    summary_file << "Average total init time: " << std::fixed
                 << std::setprecision(2) << avg_init_time << " seconds\n";
    summary_file << "Min init time: " << std::fixed
                 << std::setprecision(2) << min_init_time << " seconds\n";
    summary_file << "Max init time: " << std::fixed
                 << std::setprecision(2) << max_init_time << " seconds\n";
    summary_file << "Median init time: " << std::fixed
                 << std::setprecision(2) << median_init_time << " seconds\n";
    summary_file << "\n";
    summary_file << "# Re-initialization Statistics\n";
    summary_file << "Slices with reinit: " << reinit_count << "\n";
    summary_file << "Reinit ratio: " << std::fixed << std::setprecision(2)
                 << (total > 0 ? (reinit_count * 100.0 / total) : 0.0) << "%\n";
    summary_file << "\n";
    summary_file << "# =====================================================\n";
    summary_file << "# Recommendations:\n";

    if (success_rate >= 95.0) {
        summary_file << "# 1. 初始化成功率较高，满足预期\n";
    } else if (success_rate >= 80.0) {
        summary_file << "# 1. 初始化成功率一般，建议优化参数\n";
    } else {
        summary_file << "# 1. 初始化成功率偏低，需要重点优化\n";
    }

    if (avg_init_time <= 10.0) {
        summary_file << "# 2. 初始化耗时较短，性能良好\n";
    } else if (avg_init_time <= 20.0) {
        summary_file << "# 2. 初始化耗时适中，可以考虑优化\n";
    } else {
        summary_file << "# 2. 初始化耗时较长，建议优化参数以缩短时间\n";
    }

    if (reinit_count > 0) {
        summary_file << "# 3. 存在重初始化情况，建议检查数据质量\n";
    }

    summary_file.close();
}

} // namespace eval
} // namespace tcmsf
} // namespace byd