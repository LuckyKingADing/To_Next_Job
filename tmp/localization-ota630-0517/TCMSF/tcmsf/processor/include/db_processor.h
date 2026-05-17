#pragma once

#include <algorithm>

#include "Eigen/Eigen"
#include "base_type.h"
#include "kalman_filter.h"

namespace MSF {
class DbProcessor {

private:
    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

private:
    // 滤波器状态
    double distance_filtered_  = 0.0;   // 滤波后的横向距离
    double distance_variance_  = 1.0;   // 滤波器方差
    double last_mileage_       = 0.0;   // 上次里程（用于fading更新）
    bool   filter_initialized_ = false; // 滤波器是否已初始化

    // 调试状态（用于CSV输出）
    double last_measurement_   = 0.0; // 最新量测值
    int    csv_sample_counter_ = 0;   // CSV采样计数器（100Hz -> 10Hz）

    // 滤波器参数（二次方形态，提高可读性）
    // Q：过程噪声，表征模型不确定性随位移增长的速度
    // 设计考量：量测中断时方差需能快速累积，使下次量测能有效更新
    constexpr static double Q_STD_      = 0.1;             // 过程噪声标准差：每米0.1m
    constexpr static double Q_DISTANCE_ = Q_STD_ * Q_STD_; // Q = 0.01

    // R：测量噪声，表征单次测量的不确定性
    // 设计考量：平滑性要求高，R应较大；但需防止K过小导致滤波器迟钝
    constexpr static double R_STD_      = 0.3;             // 测量噪声标准差：0.3m
    constexpr static double R_DISTANCE_ = R_STD_ * R_STD_; // R = 0.09

    // P上下限：方差保护范围
    constexpr static double P_MAX_ = 1.0;   // 方差上限：方差过大时clamp
    constexpr static double P_MIN_ = 0.001; // 方差下限

    // Fading率：距离向零衰减的速度
    constexpr static double FADING_RATE_ = 0.02; // 衰减率：每前进1m，distance向零衰减0.02m

    // 距离上限：防止滤波值异常过大
    constexpr static double DISTANCE_MAX_ = 5.0; // 滤波距离绝对值上限：5m

public:
    DbProcessor();

public:
    // 处理DB数据：量测更新，使用最新的db消息作为观测量
    bool ProcessDbData(const DbDataPtr db_data_ptr, StatePtr state_ptr);

    // Fading更新：投影distance随前进距离向零线性衰减
    void FadingUpdate(StatePtr state_ptr);
};
} // namespace MSF
