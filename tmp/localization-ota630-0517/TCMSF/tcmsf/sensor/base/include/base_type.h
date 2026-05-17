#pragma once

#include "state_type.h"
#include <iostream>

namespace MSF {

// 基本数据结构，主体是传感器数据

// IMU
struct ImuData {
    double          measurement_timestamp; // In second.
    double          publish_timestamp;
    uint64_t        sequence_num;
    Eigen::Vector3d acc;  // Acceleration in m/s^2
    Eigen::Vector3d gyro; // Angular velocity in radian/s.
    ImuData() {
        measurement_timestamp = 0.0;
        publish_timestamp     = 0.0;
        sequence_num          = 0;
        acc                   = Eigen::Vector3d::Zero();
        gyro                  = Eigen::Vector3d::Zero();
    }
    bool isNaN() {
        // 输入字段NaN值检测
        return std::isnan(measurement_timestamp) ||
               std::isnan(publish_timestamp) ||
               acc.array().isNaN().any() ||
               gyro.array().isNaN().any();
    }
};
using ImuDataPtr = std::shared_ptr<ImuData>;

// DR
struct KinematicData {
    double             measurement_timestamp; // In second.
    Eigen::Vector3d    vel;
    Eigen::Vector3d    pos;
    Eigen::Quaterniond att;
    double             ego_longitude_vel;
    KinematicData() {
        measurement_timestamp = 0.0;
        vel                   = Eigen::Vector3d::Zero();
        pos                   = Eigen::Vector3d::Zero();
        att                   = Eigen::Quaterniond::Identity();
        ego_longitude_vel     = 0.0;
    }
    KinematicData operator-(const KinematicData &motion_) {
        // 约定减法表示增量
        // B-A => A到B的增量
        // 姿态上：B-A => C_A2B = B*A^-1
        KinematicData rslt;
        rslt.measurement_timestamp = this->measurement_timestamp - motion_.measurement_timestamp;
        rslt.vel                   = this->vel - motion_.vel;
        rslt.pos                   = this->pos - motion_.pos;
        rslt.ego_longitude_vel     = this->ego_longitude_vel - motion_.ego_longitude_vel;
        rslt.att                   = this->att * motion_.att.conjugate();
        return rslt;
    }
    bool isNaN() {
        return std::isnan(measurement_timestamp) ||
               vel.array().isNaN().any() ||
               pos.array().isNaN().any() ||
               std::isnan(att.x()) ||
               std::isnan(att.y()) ||
               std::isnan(att.z()) ||
               std::isnan(att.w()) ||
               std::isnan(ego_longitude_vel);
    }
};
std::ostream &operator<<(std::ostream &out, const KinematicData &motion);
using KinematicDataPtr = std::shared_ptr<KinematicData>;

// GNSS
struct GnssData {
    double          measurement_timestamp; // In second.
    double          publish_timestamp;
    uint64_t        sequence_num;
    Eigen::Vector3d lla;     // Latitude in rad, longitude in rad, and altitude in meter.
    Eigen::Vector3d lla_cov; // Covariance in m^2.
    Eigen::Vector3d vel;     // velocity
    Eigen::Vector3d vel_cov;
    double          hdg; // heading
    double          hdg_cov;
    uint64_t        num_sats;   // number of satellites
    uint64_t        status;     // position status
    uint64_t        raw_status; // 原始RTK状态
    double          rtk_age;
    GnssData() {
        measurement_timestamp = 0.0;
        publish_timestamp     = 0.0;
        sequence_num          = 0;
        lla                   = Eigen::Vector3d::Zero();
        lla_cov               = Eigen::Vector3d::Zero();
        vel                   = Eigen::Vector3d::Zero();
        vel_cov               = Eigen::Vector3d::Zero();
        hdg                   = 0.0;
        hdg_cov               = 0.0;
        num_sats              = 0;
        status                = 0;
        raw_status            = 0;
        rtk_age               = 0.0;
    }
    bool isNaN() {
        return std::isnan(measurement_timestamp) ||
               std::isnan(publish_timestamp) ||
               lla.array().isNaN().any() ||
               lla_cov.array().isNaN().any() ||
               vel.array().isNaN().any() ||
               vel_cov.array().isNaN().any() ||
               std::isnan(hdg) ||
               std::isnan(hdg_cov);
    }
};
using GnssDataPtr = std::shared_ptr<GnssData>;

// 底盘轮速
struct VehicleData {
    double   measurement_timestamp; // In second.
    double   publish_timestamp;
    uint64_t sequence_num;
    double   speed_rl;    // rear left wheel speed with direction
    double   speed_rr;    // rear right wheel speed with direction
    double   speed_rl_lp; // 低通过滤后的速度
    double   speed_rr_lp; // 低通过滤后的速度
    double   lpf_delay;   // 低通相位延迟
    double   yaw_rate;
    double   steer;
    double   acc_lat; // 横向加速度
    double   acc_lgt; // 纵向加速度
    uint16_t fl_pls_cnt;
    uint16_t fr_pls_cnt;
    uint16_t rl_pls_cnt;
    uint16_t rr_pls_cnt;
    bool     tire_slip;
    double   steer_rate;

    VehicleData() {
        measurement_timestamp = 0.0;
        publish_timestamp     = 0.0;
        sequence_num          = 0;
        speed_rl              = 0.0;
        speed_rr              = 0.0;
        speed_rl_lp           = 0.0;
        speed_rr_lp           = 0.0;
        lpf_delay             = 0.0;
        yaw_rate              = 0.0;
        steer                 = 0.0;
        acc_lat               = 0.0;
        acc_lgt               = 0.0;
        fl_pls_cnt            = 0;
        fr_pls_cnt            = 0;
        rl_pls_cnt            = 0;
        rr_pls_cnt            = 0;
        tire_slip             = false;
        steer_rate            = 0.0;
    }
    bool isNaN() {
        return std::isnan(measurement_timestamp) ||
               std::isnan(publish_timestamp) ||
               std::isnan(speed_rl) ||
               std::isnan(speed_rr) ||
               std::isnan(speed_rl_lp) ||
               std::isnan(speed_rr_lp) ||
               std::isnan(lpf_delay) ||
               std::isnan(yaw_rate) ||
               std::isnan(steer) ||
               std::isnan(acc_lat) ||
               std::isnan(acc_lgt) ||
               std::isnan(steer_rate);
    }
};
using VehicleDataPtr = std::shared_ptr<VehicleData>;

// 地图信息
struct MapPosData {
    double          measurement_timestamp; // In second.
    double          publish_timestamp;
    uint64_t        sequence_num;
    Eigen::Vector3d lla;
    Eigen::Vector3d lla_cov;
    double          lat_offset; // lateral offset
    double          hdg_offset; // heading offset
    MapPosData() {
        measurement_timestamp = 0.0;
        publish_timestamp     = 0.0;
        sequence_num          = 0;
        lla                   = Eigen::Vector3d::Zero();
        lla_cov               = Eigen::Vector3d::Zero();
        lat_offset            = 0.0;
        hdg_offset            = 0.0;
    }
    bool isNaN() {
        return std::isnan(measurement_timestamp) ||
               std::isnan(publish_timestamp) ||
               lla.array().isNaN().any() ||
               lla_cov.array().isNaN().any() ||
               std::isnan(lat_offset) ||
               std::isnan(hdg_offset);
    }
};
using MapPosDataPtr = std::shared_ptr<MapPosData>;

// Drive Boundary 数据
struct DbData {
    double          measurement_timestamp; // In second.
    Eigen::Matrix4d dr2veh_matrix;         // DR到车辆坐标系的变换矩阵 (4x4)
    Eigen::Vector3d veh2dr_translation;    // dr2veh_matrix逆的位移部分

    // 边界数据，各5个形点（序号区分：boundary_0、boundary_1）
    static constexpr int BOUNDARY_POINT_NUM = 5;
    Eigen::Vector2d      boundary_0[BOUNDARY_POINT_NUM]; // 边界0
    Eigen::Vector2d      boundary_1[BOUNDARY_POINT_NUM]; // 边界1

    // 中心线：左右边界对应点的中点
    Eigen::Vector2d center_line[BOUNDARY_POINT_NUM];

    // 到中心线的最近投影点结果（使用 DbProjection，定义在 state_type.h）
    DbProjection projection;

    DbData() {
        measurement_timestamp = 0.0;
        dr2veh_matrix         = Eigen::Matrix4d::Identity();
        veh2dr_translation    = Eigen::Vector3d::Zero();
        for (int i = 0; i < BOUNDARY_POINT_NUM; ++i) {
            boundary_0[i]  = Eigen::Vector2d::Zero();
            boundary_1[i]  = Eigen::Vector2d::Zero();
            center_line[i] = Eigen::Vector2d::Zero();
        }
    }

    // 构建中心线并计算投影点（在 insert_msg 时调用）
    void compute_projection() {
        // 构建中心线：两边界对应点的中点
        for (int i = 0; i < BOUNDARY_POINT_NUM; ++i) {
            center_line[i] = (boundary_0[i] + boundary_1[i]) / 2.0;
        }

        // 保存量测时间戳
        projection.measurement_timestamp = measurement_timestamp;

        // 计算到中心线的投影点
        Eigen::Vector2d query_pt(veh2dr_translation.x(), veh2dr_translation.y());
        compute_projection_to_line(query_pt, center_line, projection);
    }

private:
    // 计算查询点到中心线的最近投影
    void compute_projection_to_line(const Eigen::Vector2d &query_pt, const Eigen::Vector2d *line, DbProjection &result) const;

public:
    bool isNaN() {
        if (std::isnan(measurement_timestamp))
            return true;
        for (int i = 0; i < BOUNDARY_POINT_NUM; ++i) {
            if (boundary_0[i].array().isNaN().any())
                return true;
            if (boundary_1[i].array().isNaN().any())
                return true;
            if (center_line[i].array().isNaN().any())
                return true;
        }
        if (dr2veh_matrix.array().isNaN().any())
            return true;
        if (veh2dr_translation.array().isNaN().any())
            return true;
        if (std::isnan(projection.measurement_timestamp))
            return true;
        if (projection.proj_point.array().isNaN().any())
            return true;
        if (std::isnan(projection.distance))
            return true;
        return false;
    }

    // 调试打印：将边界形点转换到车辆坐标系并打印
    void debug_print_boundary_in_veh() const;

    // 序列化到 CSV 格式字符串
    // distance_smoothed_override: 可选参数，用于替代 projection.distance_smoothed（NaN表示使用原值）
    std::string to_csv(double distance_smoothed_override = std::numeric_limits<double>::quiet_NaN()) const;
};
using DbDataPtr = std::shared_ptr<DbData>;

// 视觉、地图匹配信息
struct VisionFusionData {
    double          measurement_timestamp; // In second.
    double          publish_timestamp;
    uint64_t        sequence_num;
    Eigen::Vector3d pos_offset;     // position offset, z=0 by default
    double          hdg_offset;     // heading offset
    Eigen::Vector3d pos_offset_std; // 位置误差参考标准差
    double          hdg_offset_std; // 航向误差参考标准差
    uint64_t        status;
    VisionFusionData() {
        measurement_timestamp = 0.0;
        publish_timestamp     = 0.0;
        sequence_num          = 0;
        pos_offset            = Eigen::Vector3d::Zero();
        hdg_offset            = 0.0;
        pos_offset_std        = Eigen::Vector3d::Zero();
        hdg_offset_std        = 0.0;
        status                = 0;
    }
    bool isNaN() {
        return std::isnan(measurement_timestamp) ||
               std::isnan(publish_timestamp) ||
               pos_offset.array().isNaN().any() ||
               pos_offset_std.array().isNaN().any() ||
               std::isnan(hdg_offset) ||
               std::isnan(hdg_offset_std);
    }
};
using VisionFusionDataPtr = std::shared_ptr<VisionFusionData>;

// 全局性的状态信息，主要用来记录一些全局状态以控制滤波过程
// 以单例的方式实现
class ProcessControl {
public:
    static ProcessControl &getInstance() {
        static ProcessControl inst;
        return inst;
    }

    ProcessControl(const ProcessControl &)            = delete;
    ProcessControl &operator=(const ProcessControl &) = delete;

private:
    ProcessControl() {}

    // private:
    // std::mutex mutex_;
private:
    ~ProcessControl() {
        LOG(INFO) << "exit ProcessControl";
    }

public:
    // 这里与state_type.h里的AlignType有一些重叠。。
    enum AlignmentType { INITIALIZATION = 0,
                         COARSE_ALIGN   = 1,
                         FINE_ALIGN     = 2,
                         ALIGNED        = 3,
    };

    // 载体运动状态
    enum VehicleMotionType { UNKNOWN       = 0,
                             MOVING        = 1,
                             ZERO_VELOCITY = 2,
    };

    // 通过IMU估计的载体机动状态
    enum ManeuverStatusByImu {
        IMU_STEADY       = 0,
        IMU_DYNAMIC_LOW  = 1,
        IMU_DYNAMIC_HIGH = 2,
    };

    // 重新初始化时候，需要重置的状态
    struct ReinitializationState {
        bool position;
        bool velocity;
        bool heading;
        ReinitializationState() :
            position(true),
            velocity(true),
            heading(true) {}
    };

    // 通过较长周期的统计，粗略估计当前RTK的状态
    enum RtkOverallStatus {
        MAJORITY_FIX   = 0, // 正常状态，预期60%以上的FIX率。默认这个状态
        MEDIUM_FIX     = 1, // 部分正常，预期40%以上的FIX率，FIX和FLOAT占比超过60%，看起来RTK本身是OK的，只是遮挡比较多
        MAJORITY_FLOAT = 2, // 亚稳状态，FIX和FLOAT占比超过80%，但是60%以上FLOAT解
        UNSTABLE       = 3, // 非稳状态，FIX和FLOAT占比超过20%，但是达不到上述其他状态
        BAD            = 4, // 异常状态，RTK难以得到FIX或者FLOAT解，FIX和FLOAT占比低于20%
    };

public:
    // 量测更新，融合的状态
    struct FusionStates {
        uint64_t imu_update_count_after_gnss_update;
        // 连续GNSS更新次数，GNSS失锁后重置
        uint64_t continous_gnss_fusion_count;
        uint64_t continous_gnss_fix_count;
        uint64_t continous_zupt_imu_bias_estimate_count;
        uint64_t wheel_speed_invalid_count;
        uint64_t imu_bias_estimating;
        uint64_t imu_update_count_after_rtk_fix;
        uint64_t imu_update_count_after_vision_fusion_update;
        uint64_t continous_gnss_float_fix_count;
        uint64_t rtk_status_maintain_count;       // fix float ppp dgps single，仅考虑这几个状态之间的切换
        uint64_t imu_update_count_aftr_pos_reset; // 位置重置后，置零。如果系统重置了航向，则这个标志位不清零。
    };
    AlignmentType     msf_align_type{INITIALIZATION};
    FusionStates      fusion_status{0};
    VehicleMotionType vehicle_motion_type{UNKNOWN};

    ReinitializationState reinitialization_state;

    ManeuverStatusByImu maneuver_status_by_imu{IMU_DYNAMIC_HIGH};
    double              maneuver_status_by_imu_scale_ = 1.0;

public:
    RtkOverallStatus rtk_overall_status = RtkOverallStatus::MAJORITY_FIX;

public:
    Eigen::Quaterniond att0_zupt = Eigen::Quaterniond::Identity();
    Eigen::Vector3d    lla0_zupt = Eigen::Vector3d::Zero();

public:
    uint64_t initialization_count = 0;

public:
    struct SensorDtState {
        bool imu_ok   = true;
        bool wheel_ok = true;
    };
    SensorDtState sensor_dt_state;
    // 设置重置的时候，这个标志位清零
    bool sensor_issue_detected = false;

private:
    struct LatestZuptInfo {
        constexpr static double NEARBY_RANGE = 100.0; // 100米里程内认为是附近

        double timestamp_out_zupt = 0.0; // 最后一次退出零速更新的时间
        double mileage_out_zupt   = 0.0; // 最后一次退出零速更新的里程
        double timestamp_nearby   = 0.0; // 持续时间累积开始的时间
        double mileage_nearby     = 0.0; // 持续时间累积开始的里程
        double duration_nearby    = 0.0; // 零速更新持续时间（统计一段距离内）
    };

    struct HeadingInnoInfo {
        constexpr static double TIME_RANGE                  = 10.0;       // 超出10秒则重置
        constexpr static double MEAN_WEIGHT_PER_MEASUREMENT = 1.0 / 20.0; // 每个量测的权重

        double   timestamp        = 0.0; // 最后更新时间
        double   mean_compensated = 0.0; // VEH（运动）补偿后的航向新息伪均值
        double   mean_raw_imu     = 0.0; // IMU航向的新息伪均值
        double   mean_raw_veh     = 0.0; // VEH航向的新息伪均值
        uint64_t hdg_count        = 0;   // 重置之后参与统计的数量
    };

    struct PoseInnoInfo {
        constexpr static double TIME_RANGE                  = 10.0;       // 超出10秒则重置
        constexpr static double MEAN_WEIGHT_PER_MEASUREMENT = 1.0 / 20.0; // 每个量测的权重

        double          pos_timestamp = 0.0;                     // 位置新息最后更新时间
        Eigen::Vector3d pos_mean_rfu  = Eigen::Vector3d::Zero(); // 右前上位置新息均值
        uint64_t        pos_count     = 0;                       // 重置之后参与统计的数量
    };

    struct VisionInnoInfo {
        constexpr static double TIME_RANGE                  = 5.0;        // 超出5秒则重置
        constexpr static double MEAN_WEIGHT_PER_MEASUREMENT = 1.0 / 10.0; // 每个量测的权重

        double   vis_timestamp = 0.0; // 最后更新时间
        double   pos_mean_r    = 0.0; // 横向偏差伪均值
        double   hdg_mean      = 0.0; // 航向偏差伪均值
        uint64_t vis_count     = 0;   // 重置之后参与统计的数量
    };

public:
    struct StateNearbyInfo {
        // 对一段时间或者里程内的状态进行一定的分析，能对融合定位的状态有一个预期
        LatestZuptInfo  latest_zupt_info;  // 零速更新过程如果周期较长的话，会导致航向误差变大
        HeadingInnoInfo heading_inno_info; // 航向新息
        PoseInnoInfo    pose_inno_info;    // 位置速度新息
        VisionInnoInfo  vision_inno_info;  // 视觉新息
    } state_nearby_info;

public:
    std::pair<double, bool> good_state_for_vehicle_bias_estimation{0.0, false}; // timestamp & state
    double                  last_valid_dt = 0;
};

static MSF::ProcessControl &process_control_sgt = MSF::ProcessControl::getInstance();

using byd::tcmsf::config::Parameters;
using byd::tcmsf::config::TCMSF_CONFIG_FILE_DIR_;

/* 21 dimension state
 * error states
 * attitude
 * velocity
 * lat-lon-height
 * gyro_bias
 * acc_bias
 * vehicle_bias <pitch_bias wheel_speed_bias yaw_bias>
 * gps_bias
 */
using Vector21d = Eigen::Matrix<double, 21, 1>;
using Matrix21d = Eigen::Matrix<double, 21, 21>;

} // namespace MSF