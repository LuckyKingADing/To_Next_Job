#pragma once
#include "Eigen/Dense"
#include "geo_trans.h"
#include "local_trans.h"
#include "tcmsf_config.h"
#include <limits>
#include <memory>

namespace MSF {

// DriveBoundary投影点结果（独立结构体，避免头文件循环依赖）
struct DbProjection {
    double          measurement_timestamp; // In second.
    Eigen::Vector2d proj_point;            // 最近投影点（DR系）
    double          distance;              // 最近距离（带符号：右侧为正，左侧为负）
    double          distance_smoothed;     // 平滑后的距离（滤波器输出）
    int             segment_idx;           // 对应的线段索引（起点索引）
    bool            is_valid;              // 投影是否在线段内部
    DbProjection() {
        measurement_timestamp = 0.0;
        proj_point            = Eigen::Vector2d::Zero();
        distance              = std::numeric_limits<double>::max();
        distance_smoothed     = 0.0;
        segment_idx           = -1;
        is_valid              = false;
    }
};

// State类是融合定位的核心状态类，包含了绝大多数在更新过程中需要传递的状态
class State {

private:
    // 取一下配置信息
    byd::tcmsf::config::Parameters &parameters_sgt = byd::tcmsf::config::Parameters::getInstance(byd::tcmsf::config::TCMSF_CONFIG_FILE_DIR_);

public:
    // 融合状态，主要表征当前参与融合的量测信息
    enum FusionStatus {
        UNINIT    = 0, // Un-initialized
        INIT      = 1, // Initialization
        GPSONLY   = 2, // GPS available, no VF
        FULLSTATE = 3, // Full state output
        VFMODE    = 4, // Only VF is alive, no GPS
        DRMODE    = 5, // Dead reckoning mode
        STATUSCNT = 6,
    };
    // 对准状态，主要表征当前融合定位的状态，使用RTK量测信息确定，不能保证所有时刻的准确性
    enum AlignType {
        // 系统会根据误差状态，在以下几个状态切换。
        // 即，即使系统完成对准了，但是因为种种因素导致系统误差变大后，系统也会切换到其他状态。
        UNALIGNED    = 0, // 未对准
        COARSE_ALIGN = 1, // 粗对准状态，可能会有大的误差和跳变
        FINE_ALIGN   = 2, // 精对准状态，系统处于缓慢收敛状态
        ALIGNED      = 3, // 完成对准
    };

public:
    State() {
        // 融合定位核心状态信息
        {
            measurement_timestamp = 0.0;
            publish_timestamp     = 0.0;
            sequence_num          = 0;
            att                   = Eigen::Quaterniond::Identity();
            vel                   = Eigen::Vector3d::Zero();
            lla                   = Eigen::Vector3d::Zero();
            acc_bias              = Eigen::Vector3d::Zero();
            gyro_bias             = Eigen::Vector3d::Zero();
            acc                   = Eigen::Vector3d::Zero();
            gyro                  = Eigen::Vector3d::Zero();
            web                   = Eigen::Vector3d::Zero();
            Mpv                   = Eigen::Matrix3d::Identity();
            MpvCnb                = Eigen::Matrix3d::Identity();
            C_b2n                 = Eigen::Matrix3d::Identity();
            vehicle_bias          = Eigen::Vector3d::Zero();
            map_bias              = Eigen::Vector3d::Zero();
            sdmap_bias_enu        = Eigen::Vector3d::Zero();
            sdmap_proj            = Eigen::Vector3d::Zero();
            sdmap_proj_mid_db     = Eigen::Vector3d::Zero();
            gravity << 0.0, 0.0, -9.8;
        }

        // 融合定位状态的标准差
        {
            error_state_std = Eigen::Matrix<double, 21, 1>::Zero();
        }

        imu_dt = 0.0;

        wheel_spd_scale_bias_ = 0.0;

        eulr_         = Eigen::Vector3d::Zero();
        q_imu2vehicle = Eigen::Quaterniond::Identity();
        C_imu2vehicle = Eigen::Matrix3d::Identity();

        rtk_status = 0;

        ctl       = Eigen::Matrix<double, 6, 1>::Zero();
        ctl_count = 0;

        phi_b_gravity    = Eigen::Vector3d::Zero();
        dr_phi_b_gravity = Eigen::Vector3d::Zero();

        lever_imu2vehicle = Eigen::Vector3d::Zero();
        lever_imu2gnss    = Eigen::Vector3d::Zero();

        inno_slip_angle_factor_s = 0.0;
        slip_index               = 0.0;

        fusion_status = FusionStatus::UNINIT;
        align_type    = AlignType::UNALIGNED;

        vel_ego = Eigen::Vector3d::Zero();

        mileage = 0.0;

        acc_lp_  = Eigen::Vector3d::Zero();
        gyro_lp_ = Eigen::Vector3d::Zero();

        gnss_inno_ = Eigen::Matrix<double, 8, 1>::Zero();

        gnss_pos_inno_statistics_ = Eigen::Matrix<double, 9, 1>::Zero();

        pos_rfu_inno_            = Eigen::Vector3d::Zero();
        pos_rfu_inno_range_norm_ = 0.0;

        pos_reset_timestamp = 0.0;

        zupt_imu_bias_estimate_ok_count = 0;
        tcmsf_aligned_count             = 0;

        vis_inno_            = Eigen::Vector3d::Zero();
        vis_t_inno_mean_std_ = Eigen::Matrix<double, 7, 1>::Zero();

        violent_bump_detected = false;

        tire_slip = false;

        state_stability_index_ratio_ = 0.0;

        gnss_consistency_score_ = -1.0;

        tcmsf_1st_time_initialized_ = false;

        rtk_fix_sat_num_mean_  = 0.0;
        spp_good_sat_num_mean_ = 0.0;
    }

public:
    // 将状态转换到载体坐标系下
    // 考虑IMU的安装角误差和安装杆臂
    State ToVehicle();
    // 从车体系转换到IMU壳体系
    State FromVehicleToImu();

    // 在lla上附加sdmap偏置量，返回新状态
    State WithSdmapBias();
    // 在lla上去掉sdmap偏置量，返回新状态（逆过程）
    State WithoutSdmapBias();

public:
    // 计算自车坐标系下自车的速度信息
    Eigen::Vector3d GetEgoVel();
    // 计算融合映射到天线的位置
    Eigen::Vector3d GetAntennaBLH();

public:
    double             measurement_timestamp; // In second.
    double             publish_timestamp;
    uint64_t           sequence_num;
    Eigen::Quaterniond att;
    Eigen::Vector3d    vel;
    Eigen::Vector3d    lla;
    Eigen::Vector3d    acc_bias;  // The bias of the acceleration sensor.
    Eigen::Vector3d    gyro_bias; // The bias of the gyroscope sensor.

    Eigen::Vector3d vehicle_bias;
    double          wheel_spd_scale_bias_;

    Eigen::Vector3d map_bias;          // LD 制图误差（右前上）
    Eigen::Vector3d sdmap_bias_enu;    // SD 地图偏置量（东北天）
    Eigen::Vector3d sdmap_proj;        // SD投影点
    Eigen::Vector3d sdmap_proj_mid_db; // SD投影点，补偿到DB中心

    int rtk_status;

    Eigen::Vector3d acc;  // 加速度测量量，去除重力
    Eigen::Vector3d gyro; // 角速度测量量

    Eigen::Vector3d web;
    Eigen::Matrix3d Mpv;
    Eigen::Matrix3d MpvCnb;
    Eigen::Matrix3d C_b2n;

    Eigen::Vector3d gravity; // IMU系下的重力矢量

public:
    Eigen::Vector3d vel_ego;

public:
    double inno_slip_angle_factor_s;
    double slip_index;

public:
    Eigen::Vector3d eulr_;

public:
    int                         ctl_count;
    Eigen::Matrix<double, 6, 1> ctl;

public:
    Eigen::Matrix<double, 21, 1> error_state_std;

public:
    Eigen::Vector3d phi_b_gravity;
    Eigen::Vector3d dr_phi_b_gravity;

public:
    Eigen::Vector3d lever_imu2vehicle;
    Eigen::Vector3d lever_imu2gnss;

public:
    Eigen::Quaterniond q_imu2vehicle;
    Eigen::Matrix3d    C_imu2vehicle;

public:
    FusionStatus fusion_status;
    AlignType    align_type;

public:
    double imu_dt;

public:
    double mileage;

public:
    Eigen::Vector3d acc_lp_;
    Eigen::Vector3d gyro_lp_;

public:
    Eigen::Matrix<double, 8, 1> gnss_inno_;
    Eigen::Matrix<double, 9, 1> gnss_pos_inno_statistics_;

    Eigen::Vector3d pos_rfu_inno_;
    double          pos_rfu_inno_range_norm_;

public:
    double pos_reset_timestamp;

public:
    uint32_t zupt_imu_bias_estimate_ok_count;

public:
    uint64_t tcmsf_aligned_count;

public:
    Eigen::Vector3d             vis_inno_;
    Eigen::Matrix<double, 7, 1> vis_t_inno_mean_std_;

public:
    bool violent_bump_detected;

public:
    bool tire_slip;

public:
    double state_stability_index_ratio_;

public:
    // 卫星消息自洽性系数，范围[0,1]，1表示完全自洽
    // 默认值为-1，表示不可用（未满足验证条件）
    double gnss_consistency_score_;

public:
    bool tcmsf_1st_time_initialized_;

public:
    double rtk_fix_sat_num_mean_;
    double spp_good_sat_num_mean_;

public:
    DbProjection db_projection; // DriveBoundary中心线投影点

public:
    bool isNaN() {
        return std::isnan(measurement_timestamp) ||
               std::isnan(publish_timestamp) ||
               std::isnan(att.x()) ||
               std::isnan(att.y()) ||
               std::isnan(att.z()) ||
               std::isnan(att.w()) ||
               vel.array().isNaN().any() ||
               lla.array().isNaN().any() ||
               acc_bias.array().isNaN().any() ||
               gyro_bias.array().isNaN().any() ||
               acc.array().isNaN().any() ||
               gyro.array().isNaN().any() ||
               web.array().isNaN().any() ||
               Mpv.array().isNaN().any() ||
               MpvCnb.array().isNaN().any() ||
               C_b2n.array().isNaN().any() ||
               vehicle_bias.array().isNaN().any() ||
               map_bias.array().isNaN().any() ||
               sdmap_bias_enu.array().isNaN().any() ||
               sdmap_proj.array().isNaN().any() ||
               sdmap_proj_mid_db.array().isNaN().any() ||
               gravity.array().isNaN().any() ||
               error_state_std.array().isNaN().any() ||
               std::isnan(state_stability_index_ratio_) ||
               std::isnan(gnss_consistency_score_);
    }

private:
    byd::geo::LocalTrans local_trans;

public:
    void mapBiasAblationWhenReset(const Eigen::Vector3d &new_lla_);
};

using StatePtr = std::shared_ptr<MSF::State>;

class StateSwitchInfo {
private:
    double switch_timestamp{0};
    double into_timestamp{0};
    double out_of_timestamp{0};
    bool   pre_state{false};
    bool   cur_state{false};

public:
    void insert_state(bool state, double timestamp) {
        if (pre_state != state) {
            switch_timestamp = timestamp;
            // AINFO << "state switch T: " << fmt::format("{:>14.5f}", switch_timestamp);
            if (state == true) {
                into_timestamp = timestamp;
                // AINFO << "state into T: " << fmt::format("{:>14.5f}", into_timestamp);
            }
            if (state == false) {
                out_of_timestamp = timestamp;
                // AINFO << "state out of T: " << fmt::format("{:>14.5f}", out_of_timestamp);
            }
        }
        cur_state = state;
        pre_state = state;
    }
    double get_switch_timestamp() {
        return switch_timestamp;
    }
    double get_into_timestamp() {
        return into_timestamp;
    }
    double get_out_of_timestamp() {
        return out_of_timestamp;
    }
    bool get_current_state() {
        return cur_state;
    }
};

} // namespace MSF