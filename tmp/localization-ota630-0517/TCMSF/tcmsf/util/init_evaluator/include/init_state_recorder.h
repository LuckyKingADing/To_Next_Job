#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <memory>
#include <algorithm>
#include <cmath>
#include <iomanip>

namespace byd {
namespace tcmsf {
namespace eval {

// 初始化状态枚举
enum class InitState {
    NOT_STARTED = 0,      // 未开始
    DYNAMIC_READY = 1,    // 运动状态满足
    INIT_STARTED = 2,     // 开始初始化
    INIT_COMPLETED = 3,   // 初始化完成
    INIT_FAILED = 4,      // 初始化失败（超时或重置）
    REINIT_OCCURRED = 5   // 发生重新初始化
};

// 单次初始化测试结果结构体
struct InitTestResult {
    int slice_id = -1;                    // 切片编号
    double slice_start_timestamp = 0.0;   // 切片起始时间戳
    double slice_end_timestamp = 0.0;     // 切片结束时间戳

    // 时间点记录
    double dynamic_ready_timestamp = -1.0;  // 运动状态满足时间
    double init_start_timestamp = -1.0;     // 开始初始化时间
    double init_complete_timestamp = -1.0;  // 初始化完成时间

    // 耗时统计
    double time_to_dynamic_ready = -1.0;    // 从切片起点到运动状态满足
    double time_to_init_start = -1.0;       // 从运动满足到开始初始化
    double total_init_time = -1.0;          // 总初始化耗时

    // 状态结果
    InitState final_state = InitState::NOT_STARTED;  // 最终状态
    bool is_success = false;                // 是否成功初始化
    bool has_reinit = false;                 // 是否发生重初始化
    int reinit_count = 0;                    // 重初始化次数
    double first_aligned_timestamp = -1.0;  // 首次达到ALIGNED状态的时间

    // 位置信息
    double start_lat = 0.0;
    double start_lon = 0.0;
    double start_speed = 0.0;              // 初始速度

    // 统计信息
    int gnss_count = 0;                    // GNSS消息数量
    int imu_count = 0;                     // IMU消息数量
    int gnss_fix_count = 0;                // FIX解数量
    int gnss_float_count = 0;              // FLOAT解数量

    // 计算耗时
    void calculate_timing();
};

// 初始化状态记录器类声明
class InitStateRecorder {
public:
    InitStateRecorder();
    ~InitStateRecorder();

    // 重置状态
    void reset();

    // 设置切片信息
    void set_slice_info(int slice_id, double start_ts, double end_ts);

    // 记录各状态节点
    void record_dynamic_ready(double timestamp);
    void record_init_start(double timestamp);
    void record_init_complete(double timestamp);
    void record_reinit(double timestamp);
    void record_aligned(double timestamp);

    // 更新状态
    void update_state(InitState state);

    // 标记失败
    void mark_failed(double timestamp);

    // 统计信息更新
    void increment_gnss_count();
    void increment_imu_count();
    void increment_gnss_fix_count();
    void increment_gnss_float_count();

    void set_start_position(double lat, double lon, double speed);

    // 获取当前结果
    InitTestResult get_result() const;

    // 检查超时
    bool check_timeout(double current_timestamp, double timeout_threshold);

    // 检查是否完成初始化
    bool is_init_completed() const;

private:
    InitTestResult current_result_;
    bool dynamic_ready_recorded_ = false;
    bool init_start_recorded_ = false;
    bool init_complete_recorded_ = false;
    bool aligned_reached_ = false;
    bool first_gnss_recorded_ = false;
};

// 初始化测试结果输出器类声明
class InitTestResultWriter {
public:
    InitTestResultWriter(const std::string& output_path);
    ~InitTestResultWriter();

    bool is_open() const;

    void write_header();
    void write_result(const InitTestResult& result);
    void write_summary(const std::vector<InitTestResult>& results,
                       const std::string& summary_path);

private:
    std::ofstream file_;
    std::string output_path_;
};

} // namespace eval
} // namespace tcmsf
} // namespace byd