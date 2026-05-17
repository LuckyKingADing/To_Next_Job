#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>

#include "cyber/common/log.h"

namespace MSF::TRAJ {

// ==================== FrechetDistanceAligner ====================
/**
 * @brief 计算两条轨迹之间对齐后的 Fréchet 距离及相关工具函数。
 *
 * 支持将弧度经纬度转换为局部米制坐标，并进行航向对齐。
 */
class FrechetDistanceAligner {
public:
    /// 地球平均半径（IUGG 推荐值，单位：米）
    static constexpr double EARTH_RADIUS    = 6371008.8;
    static constexpr double MIN_TRAJ_LENGTH = 100.0;

    FrechetDistanceAligner() = default;

    /**
     * @brief 计算米制轨迹的总长度（累积欧氏距离）。
     * @param meterTraj 米制坐标轨迹，单位：米。
     * @return 轨迹总长度（米）。
     */
    static double trajectoryLength(const std::vector<Eigen::Vector2d> &meterTraj) {
        if (meterTraj.size() < 2) {
            return 0.0;
        }
        double length = 0.0;
        for (size_t i = 1; i < meterTraj.size(); ++i) {
            length += (meterTraj[i] - meterTraj[i - 1]).norm();
        }
        return length;
    }

    /**
     * @brief 计算经纬度轨迹与米制轨迹对齐后的离散 Fréchet 距离。
     * @param llhTraj   经纬度轨迹，每个元素为 (经度, 纬度)，单位：弧度。
     * @param meterTraj 米制坐标轨迹，单位：米。
     * @return 对齐后的 Fréchet 距离（米）。若任一轨迹点数 < 2，返回 0.0。
     */
    double compute(const std::vector<Eigen::Vector2d> &llhTraj, const std::vector<Eigen::Vector2d> &meterTraj) {
        if ((llhTraj.size() < 2) || (meterTraj.size() < 2)) {
            return 0.0;
        }
        if (trajectoryLength(meterTraj) < MIN_TRAJ_LENGTH) {
            return 0.0;
        }

        // 1. 经纬度 → 局部米制坐标（存入 buf1_）
        llhToLocalMeters(llhTraj, buf1_);

        // 2. 航向对齐：旋转 + 平移（存入 buf2_）
        alignHeading(buf1_, meterTraj, buf2_);

        // 3. 计算离散 Fréchet 距离
        return discreteFrechet(buf2_, meterTraj);
    }

    /**
     * @brief 将弧度经纬度差转换为局部米制位移矢量（球面近似）。
     * @param from_lonlat 起点 (经度, 纬度)，单位：弧度。
     * @param to_lonlat   终点 (经度, 纬度)，单位：弧度。
     * @return 从起点指向终点的二维矢量 (东向位移, 北向位移)，单位：米。
     * @note  采用等距圆柱投影，假设地球为半径 R 的球体，适用于小范围（<100km）。
     */
    static Eigen::Vector2d lonlatToMeterOffsetApprox(const Eigen::Vector2d &from_lonlat, const Eigen::Vector2d &to_lonlat) {

        const double dLon   = to_lonlat.x() - from_lonlat.x();
        const double dLat   = to_lonlat.y() - from_lonlat.y();
        const double cosLat = std::cos(from_lonlat.y()); // 使用起点纬度近似

        const double dx = dLon * EARTH_RADIUS * cosLat; // 东向（经度方向）
        const double dy = dLat * EARTH_RADIUS;          // 北向（纬度方向）

        return Eigen::Vector2d(dx, dy);
    }

    /**
     * @brief 将弧度经纬度序列转换为以第一个点为原点的局部米制坐标。
     * @param llh       输入经纬度（弧度）。
     * @param outMeters 输出米制坐标，大小将被调整为 llh.size()。
     */
    static void llhToLocalMeters(const std::vector<Eigen::Vector2d> &llh, std::vector<Eigen::Vector2d> &outMeters) {
        const size_t n = llh.size();
        outMeters.resize(n);
        if (n == 0) {
            return;
        }

        const double          lat0    = llh[0].y();
        const double          cosLat0 = std::cos(lat0);
        const Eigen::Vector2d origin  = llh[0];

        for (size_t i = 0; i < n; ++i) {
            const double dLon = llh[i].x() - origin.x();
            const double dLat = llh[i].y() - origin.y();
            outMeters[i]      = Eigen::Vector2d(dLon * EARTH_RADIUS * cosLat0, dLat * EARTH_RADIUS);
        }
    }

    /**
     * @brief 计算轨迹起点到终点的向量角度（弧度）。
     * @param traj 点序列（至少2个点）。
     * @return 向量角度，若首尾点重合或点数不足，返回 0.0。
     */
    static double headingAngle(const std::vector<Eigen::Vector2d> &traj) {
        if (traj.size() < 2) {
            return 0.0;
        }
        const Eigen::Vector2d vec = traj.back() - traj.front();
        if (vec.squaredNorm() <= std::numeric_limits<double>::epsilon()) {
            return 0.0;
        }
        return std::atan2(vec.y(), vec.x());
    }

    /**
     * @brief 将点集绕原点旋转 angle 弧度（逆时针为正）。
     * @param points     输入点集。
     * @param angle      旋转角度（弧度）。
     * @param outRotated 输出旋转后的点集，大小将被调整为 points.size()。
     */
    static void rotate(const std::vector<Eigen::Vector2d> &points, double angle, std::vector<Eigen::Vector2d> &outRotated) {
        const size_t n = points.size();
        outRotated.resize(n);
        const double cosA = std::cos(angle);
        const double sinA = std::sin(angle);
        for (size_t i = 0; i < n; ++i) {
            const double x = points[i].x();
            const double y = points[i].y();
            outRotated[i]  = Eigen::Vector2d(x * cosA - y * sinA, x * sinA + y * cosA);
        }
    }

    /**
     * @brief 航向对齐：旋转局部米制轨迹，使其首尾向量方向与参考轨迹一致，
     *        并平移使起点重合。同时保证首尾尺度重合。
     * @param localMeters 待对齐的局部米制轨迹（点数 ≥ 2）。
     * @param meterTraj   参考米制轨迹（点数 ≥ 2）。
     * @param aligned     输出对齐后的轨迹。
     */
    static void alignHeading(const std::vector<Eigen::Vector2d> &localMeters, const std::vector<Eigen::Vector2d> &meterTraj, std::vector<Eigen::Vector2d> &aligned) {
        assert(localMeters.size() >= 2 && meterTraj.size() >= 2);

        const double angle1   = headingAngle(localMeters);
        const double angle2   = headingAngle(meterTraj);
        const double rotAngle = angle2 - angle1;

        const double scale_ = (meterTraj.back() - meterTraj.front()).norm() / ((localMeters.back() - localMeters.front()).norm() + 1e-10);

        rotate(localMeters, rotAngle, aligned);

        // 这里通过一个尺度，使得两个序列能够首尾重合
        const auto &pt0 = aligned.front();
        for (auto &pt : aligned) {
            pt = pt0 + (pt - pt0) * scale_;
        }

        const Eigen::Vector2d offset = meterTraj.front() - aligned.front();
        for (auto &pt : aligned) {
            pt += offset;
        }
    }

    /**
     * @brief 计算两个米制轨迹之间的离散 Fréchet 距离。
     * @param P 轨迹 P（米制坐标）。
     * @param Q 轨迹 Q（米制坐标）。
     * @return 离散 Fréchet 距离（米）。
     */
    static double discreteFrechet(const std::vector<Eigen::Vector2d> &P, const std::vector<Eigen::Vector2d> &Q) {
        const size_t n = P.size();
        const size_t m = Q.size();
        if ((n == 0) || (m == 0)) {
            return 0.0;
        }

        // 确保 P 是较短的序列，以减少内存使用
        if (n > m) {
            return discreteFrechet(Q, P);
        }

        std::vector<double> prevRow(m);
        std::vector<double> currRow(m);

        // 初始化第一行
        for (size_t j = 0; j < m; ++j) {
            prevRow[j] = (P[0] - Q[j]).norm();
        }
        for (size_t j = 1; j < m; ++j) {
            prevRow[j] = std::max(prevRow[j - 1], prevRow[j]);
        }

        // 若 P 只有一个点，直接返回
        if (n == 1) {
            return prevRow[m - 1];
        }

        // 动态规划填充剩余行
        for (size_t i = 1; i < n; ++i) {
            currRow[0] = std::max(prevRow[0], (P[i] - Q[0]).norm());
            for (size_t j = 1; j < m; ++j) {
                const double dist_ij = (P[i] - Q[j]).norm();
                const double minPrev = std::min({prevRow[j], currRow[j - 1], prevRow[j - 1]});
                currRow[j]           = std::max(minPrev, dist_ij);
            }
            prevRow.swap(currRow);
        }

        return prevRow[m - 1];
    }

    /**
     * @brief 预分配内部缓冲区容量，避免多次动态分配。
     * @param cap 预计最大点数。
     */
    void reserve(size_t cap) {
        buf1_.reserve(cap);
        buf2_.reserve(cap);
    }

private:
    std::vector<Eigen::Vector2d> buf1_; // 缓冲区1：存放局部米制坐标
    std::vector<Eigen::Vector2d> buf2_; // 缓冲区2：存放对齐后的坐标
};

// ==================== TrajectoryCollector (Sliding Window) ====================
class TrajectoryCollector {
public:
    /**
     * @brief 构造函数。
     * @param window_size       滑动窗口大小（保留最近的点数）。
     * @param distance_interval 里程触发间隔（米）。
     * @param max_time_gap      最大允许时间间隔（秒），超出则清空窗口。
     */
    TrajectoryCollector(size_t window_size, double distance_interval, double max_time_gap) :
        window_size_(window_size),
        distance_interval_(distance_interval),
        max_time_gap_(max_time_gap),
        accumulated_distance_(0.0),
        last_timestamp_(-1.0),
        last_odom_(0.0),
        has_last_odom_(false) {}

    /**
     * @brief 输入一帧数据，按里程触发采集并维护滑动窗口。
     * @param timestamp  当前时间戳（秒）。
     * @param odom       累积里程（米）。
     * @param gnss_rad   卫星定位经纬度（弧度），格式 (lon, lat)。
     * @param fused_rad  融合定位经纬度（弧度）。
     * @param dr_xy      二维 DR 递推坐标（米）。
     * @return 若本次采集导致窗口内容发生变化，返回 true；否则 false。
     */
    bool update(double timestamp, double odom, const Eigen::Vector2d &gnss_rad, const Eigen::Vector2d &fused_rad, const Eigen::Vector2d &dr_xy) {
        // 首次调用初始化
        if (!has_last_odom_) {
            last_timestamp_ = timestamp;
            last_odom_      = odom;
            has_last_odom_  = true;
            return false;
        }

        // 时间不连续检查：清空整个窗口
        if (std::abs(timestamp - last_timestamp_) > max_time_gap_) {
            reset();
            last_timestamp_       = timestamp;
            last_odom_            = odom;
            accumulated_distance_ = 0.0;
            return true; // 窗口被清空，视为变化
        }

        // 累计里程变化（绝对值）
        const double delta_odom = std::abs(odom - last_odom_);
        accumulated_distance_ += delta_odom;

        bool changed = false;

        // 可能触发多次采集（如果单次里程跨越多个间隔）
        while (accumulated_distance_ >= distance_interval_) {
            // 采集当前点
            gnss_window_.push_back(gnss_rad);
            fused_window_.push_back(fused_rad);
            dr_window_.push_back(dr_xy);

            // 保持窗口大小
            if (gnss_window_.size() > window_size_) {
                gnss_window_.pop_front();
                fused_window_.pop_front();
                dr_window_.pop_front();
            }

            accumulated_distance_ -= distance_interval_;
            changed = true;
        }

        // 更新状态
        last_timestamp_ = timestamp;
        last_odom_      = odom;

        return changed;
    }

    /**
     * @brief 检查滑动窗口是否已满（即点数达到预设大小）。
     */
    bool isWindowFull() const {
        return gnss_window_.size() == window_size_;
    }

    /**
     * @brief 清空窗口并重置里程累积状态。
     */
    void reset() {
        gnss_window_.clear();
        fused_window_.clear();
        dr_window_.clear();
        accumulated_distance_ = 0.0;
        has_last_odom_        = false;
    }

    // ---------- 轨迹质量评估 ----------
    /**
     * @brief 计算当前窗口内轨迹的质量指标。
     * @param[out] out_frechet_gnss_dr  GNSS 与 DR 轨迹的 Fréchet 距离（米）。
     * @param[out] out_rmse_gnss_fused  GNSS 与融合定位的均方根误差（米）。
     * @return 若当前窗口至少有 2 个点且计算成功，返回 true；否则返回 false。
     */
    bool computeMetrics(double &out_frechet_gnss_dr, double &out_rmse_gnss_fused) {
        if (!isWindowFull() || gnss_window_.size() < 2) {
            // 如果窗口未满或者窗口小于2，返回false
            return false;
        }

        // 将 deque 转换为 vector 以便计算（因为 Frechet 类使用 vector）
        std::vector<Eigen::Vector2d> gnss_vec(gnss_window_.begin(), gnss_window_.end());
        std::vector<Eigen::Vector2d> fused_vec(fused_window_.begin(), fused_window_.end());
        std::vector<Eigen::Vector2d> dr_vec(dr_window_.begin(), dr_window_.end());

        // 1. 将 GNSS 经纬度转换为局部米制坐标
        std::vector<Eigen::Vector2d> gnss_meters;
        FrechetDistanceAligner::llhToLocalMeters(gnss_vec, gnss_meters);

        // 2. 计算 GNSS 与 DR 的 Fréchet 距离（内部进行航向对齐）
        FrechetDistanceAligner aligner;
        out_frechet_gnss_dr = aligner.compute(gnss_vec, dr_vec);

        // 3. 将融合定位经纬度转换为米制坐标
        std::vector<Eigen::Vector2d> fused_meters;
        FrechetDistanceAligner::llhToLocalMeters(fused_vec, fused_meters);

        // 4. 计算 GNSS 与融合定位的均方根误差（逐点对应）
        auto dpos_gnss2msf_ = FrechetDistanceAligner::lonlatToMeterOffsetApprox(gnss_vec[0], fused_vec[0]);
        if (gnss_meters.size() == fused_meters.size()) {
            double sum_sq = 0.0;
            for (size_t i = 0; i < gnss_meters.size(); ++i) {
                sum_sq += (-dpos_gnss2msf_ + gnss_meters[i] - fused_meters[i]).squaredNorm();
            }
            out_rmse_gnss_fused = std::sqrt(sum_sq / gnss_meters.size());
        } else {
            // 点数不一致时无法计算 RMSE，返回 double 最大值作为无效标志
            // 这里不使用NaN，避免下游调用的时候，未能对NaN值做很好的处理。
            out_rmse_gnss_fused = std::numeric_limits<double>::max();
        }
        return true;
    }

private:
    size_t window_size_;       ///< 滑动窗口容量
    double distance_interval_; ///< 里程触发间隔（米）
    double max_time_gap_;      ///< 最大允许时间间隔（秒）

    std::deque<Eigen::Vector2d> gnss_window_;  ///< GNSS 轨迹窗口
    std::deque<Eigen::Vector2d> fused_window_; ///< 融合定位轨迹窗口
    std::deque<Eigen::Vector2d> dr_window_;    ///< DR 递推轨迹窗口

    double accumulated_distance_; ///< 当前累积的里程变化（未触发部分）
    double last_timestamp_;       ///< 上一次时间戳
    double last_odom_;            ///< 上一次里程值
    bool   has_last_odom_;        ///< 是否已初始化
};

} // namespace MSF::TRAJ
