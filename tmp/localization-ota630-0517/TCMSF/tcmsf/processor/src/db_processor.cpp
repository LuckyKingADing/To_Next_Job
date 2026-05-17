#include "db_processor.h"
#include "processor_debug.h"

namespace MSF {

DbProcessor::DbProcessor() {}

// ============================================================================
// Kalman滤波量测更新（Measurement Update）
//
// 状态模型：
// - 状态 x：滤波后的横向距离（车辆相对于中心线的横向偏差估计）
// - 方差 P：状态估计的不确定性
// - 量测 z：当前投影距离（DbData.projection.distance）
//
// 量测方程：
// z = x + v，其中 v ~ N(0, R) 为量测噪声
//
// Kalman滤波推导：
// 1. 新息（innovation）：y = z - x_pred
//    表示量测与预测的差异
//
// 2. 新息方差：S = P_pred + R
//    表示新息的不确定性（预测方差 + 量测噪声）
//
// 3. Kalman增益：
//    K = P_pred / S = P_pred / (P_pred + R)
//    解释：K 决定了新息对状态更新的贡献权重
//    - K → 1：预测方差大，信任量测，大幅更新
//    - K → 0：预测方差小，信任预测，小幅更新
//
// 4. 状态更新：
//    x_new = x_pred + K * y = x_pred + K * (z - x_pred)
//
// 5. 方差更新：
//    P_new = (1 - K) * P_pred
//    推导：
//    P_new = (1 - K)^2 * P_pred + K^2 * R （完整形式，假设预测与量测独立）
//    简化：当量测方程为 z = x + v 时，最优估计方差为 P_new = (1-K) * P_pred
//    代入 K = P/(P+R)：P_new = P * R / (P+R) < P，方差减小
//
// 参数说明：
// - R = 0.09 m²：量测噪声方差，反映单次投影距离测量的不确定性
// - P_MIN = 0.001 m²：方差下限，防止方差过小导致滤波器僵化
// - P_MAX = 1.0 m²：方差上限，防止方差过大导致滤波器不稳定
// ============================================================================
bool DbProcessor::ProcessDbData(const DbDataPtr db_data_ptr, StatePtr state_ptr) {
    // 只有有效投影才进行滤波
    if (!db_data_ptr->projection.is_valid) {
        return false;
    }

    // 量测值：当前投影距离
    double z = db_data_ptr->projection.distance;

    if (!filter_initialized_) {
        // 首次初始化：状态取量测值，方差取量测噪声方差
        // 原因：无先验信息时，初始状态的不确定性等于单次量测的不确定性
        distance_filtered_  = z;
        distance_variance_  = R_DISTANCE_;
        last_mileage_       = state_ptr->mileage;
        filter_initialized_ = true;
    } else {
        // Kalman滤波量测更新
        // 预测值：distance_filtered_（在 FadingUpdate 中已更新）
        // 预测方差：distance_variance_（在 FadingUpdate 中已累积）

        // Step 1: 新息 y = z - x_pred
        double innovation = z - distance_filtered_;

        // Step 2: 新息方差 S = P_pred + R
        double innovation_var = distance_variance_ + R_DISTANCE_;

        // Step 3: Kalman增益 K = P_pred / S
        double K = distance_variance_ / innovation_var;

        // Step 4: 状态更新 x_new = x_pred + K * y
        distance_filtered_ = distance_filtered_ + K * innovation;

        // Step 5: 方差更新 P_new = (1 - K) * P_pred
        distance_variance_ = (1.0 - K) * distance_variance_;

        // 方差保护：clamp 到 [P_MIN, P_MAX]
        distance_variance_ = std::clamp(distance_variance_, P_MIN_, P_MAX_);

        // 距离保护：clamp 到 [-DISTANCE_MAX, DISTANCE_MAX]
        distance_filtered_ = std::clamp(distance_filtered_, -DISTANCE_MAX_, DISTANCE_MAX_);
    }

    // 更新state中的投影信息
    state_ptr->db_projection                   = db_data_ptr->projection;
    state_ptr->db_projection.distance_smoothed = distance_filtered_;

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
    // DbData csv 序列化输出（使用滤波后的 distance_smoothed）
    std::string db_csv = db_data_ptr->to_csv(distance_filtered_) + "\n";
    debug::debug_sgt.db_state.line(db_csv);
#endif

    // 记录最新量测值，供 FadingUpdate 调试输出使用
    last_measurement_ = z;

    return true;
}

// ============================================================================
// 衰减更新（Fading Update）：预测步骤
//
// 功能1：距离衰减
// - 横向距离随前进里程向零衰减
// - 物理意义：车辆前进时，历史横向偏差的影响逐渐减弱
// - 衰减模型：distance_new = distance_old - fading_rate * delta_mileage * sign(distance)
//   - 若 |distance| > fading_amount：distance 向零线性衰减
//   - 若 |distance| <= fading_amount：distance = 0（避免过零振荡）
//
// 功能2：方差累积
// - 方差随前进里程线性增加
// - 物理意义：无量测更新时，状态不确定性随时间（里程）增长
// - 模型：P_new = P_old + Q * delta_mileage
//   - Q 为过程噪声方差率（单位：m²/m），表示每米位移带来的方差增量
//   - 线性增长而非二次增长，因为横向偏差主要由航向误差引起，与位移近似线性关系
//
// 参数说明：
// - Q = 0.01 m²/m：过程噪声方差率，每米位移方差增加 0.01 m²
// - FADING_RATE = 0.02 m/m：衰减率，每米位移距离衰减 0.02 m
// ============================================================================
void DbProcessor::FadingUpdate(StatePtr state_ptr) {
    if (!filter_initialized_) {
        return;
    }

    // 计算位移增量（绝对值）
    double delta_mileage = std::abs(state_ptr->mileage - last_mileage_);

    // -------------------------------------------------
    // 功能1：距离衰减
    // -------------------------------------------------
    // 衰减量 = 衰减率 × 位移
    double fading_amount = FADING_RATE_ * delta_mileage;

    // 衰减逻辑：向零方向衰减，避免过零振荡
    if (std::abs(distance_filtered_) > fading_amount) {
        // 衰减后仍有残余距离
        distance_filtered_ -= fading_amount * (distance_filtered_ > 0.0 ? 1.0 : -1.0);
    } else {
        // 衰减后距离过零，直接置零
        distance_filtered_ = 0.0;
    }

    // -------------------------------------------------
    // 功能2：方差累积（过程噪声注入）
    // -------------------------------------------------
    // P_new = P_old + Q * delta_mileage
    // 解释：无量测时，不确定性随里程线性增长
    distance_variance_ += Q_DISTANCE_ * delta_mileage;

    // 方差保护：clamp 到 [P_MIN, P_MAX]
    distance_variance_ = std::clamp(distance_variance_, P_MIN_, P_MAX_);

    // 距离保护：clamp 到 [-DISTANCE_MAX, DISTANCE_MAX]
    distance_filtered_ = std::clamp(distance_filtered_, -DISTANCE_MAX_, DISTANCE_MAX_);

    // 更新里程记录
    last_mileage_ = state_ptr->mileage;

    // 更新state中的平滑距离
    state_ptr->db_projection.distance_smoothed = distance_filtered_;
}

} // namespace MSF
