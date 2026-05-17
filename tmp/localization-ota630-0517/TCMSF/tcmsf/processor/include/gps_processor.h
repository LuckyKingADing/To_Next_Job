#pragma once

#include "Eigen/Eigen"

#include "base_type.h"
#include "earth.h"
#include "igg3.h"
#include "kalman_filter.h"
#include "local_trans.h"
// #include "motion_status.h"
#include "state_analysis.h"
#include <iomanip>

#include <array>
#include <deque>

#include "vehicle_processor.h"

namespace MSF {

// DR、MSF、RTK三者做一个交叉验证
// 通过DR+RTK判断RTK是否正常，如果RTK正常，再通过RTK+MSF判断MSF是否正常
class RtkCrossValidation {
private:
    struct NaviInfo {
        double timestamp;
        // pos
        Eigen::Vector3d DrPos;
        Eigen::Vector3d MsfPos;
        Eigen::Vector3d GnssPos;
        // rfu
        Eigen::Vector3d DrRfu;
        Eigen::Vector3d GnssRfu;
        // delta rfu
        Eigen::Vector3d DrDeltaRfu;
        Eigen::Vector3d GnssDeltaRfu;
        // diff rfu
        Eigen::Vector3d GnssMsfDiffRfu;
        // quat
        Eigen::Quaterniond DrQuat;
        Eigen::Quaterniond MsfQuat;
        // vel
        Eigen::Vector3d DrVel;

        NaviInfo() {
            DrPos          = Eigen::Vector3d::Zero();
            MsfPos         = Eigen::Vector3d::Zero();
            GnssPos        = Eigen::Vector3d::Zero();
            DrRfu          = Eigen::Vector3d::Zero();
            GnssRfu        = Eigen::Vector3d::Zero();
            DrDeltaRfu     = Eigen::Vector3d::Zero();
            GnssDeltaRfu   = Eigen::Vector3d::Zero();
            GnssMsfDiffRfu = Eigen::Vector3d::Zero();
            DrQuat         = Eigen::Quaterniond::Identity();
            MsfQuat        = Eigen::Quaterniond::Identity();
            DrVel          = Eigen::Vector3d::Zero();
        }
    };

private:
    static constexpr uint64_t BUFFER_SIZE   = 4;
    static constexpr uint64_t GNSS_MSG_SKIP = 3;
    uint64_t                  msg_count     = 0;

private:
    std::deque<NaviInfo> navi_info_buffer;

private:
    byd::geo::LocalTrans local_trans;

public:
    constexpr static double CROSS_VALID_DT_ = BUFFER_SIZE * GNSS_MSG_SKIP * 0.1 * 2.0;

public:
    std::pair<bool, NaviInfo> CurNaviInfo = std::make_pair(false, NaviInfo());

public:
    void insert(const GnssDataPtr gps_data_ptr_, const StatePtr state_ptr_gpst_, const KinematicDataPtr dr_ptr_gpst_);

public:
    struct DiffInfo {
        double timestamp           = 0.0;
        double gnss_dr_diff_index  = 0.0;
        double msf_gnss_diff_index = 0.0;
        double cross_diff_index    = 0.0;
    } diffinfo;
};

class SinsStableIndex {
private:
    double                  ssi_pre_mileage_       = 0.0;
    double                  ssi_pre_timestamp_     = 0.0;
    double                  state_stability_index_ = 0.0;
    static constexpr double SSI_MAXIMUM_           = 6000.0;
    static constexpr double SSI_MINIMUM_           = 0.0;

private:
    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

public:
    void   state_stability_index_update(const GnssDataPtr gps_data_ptr, const StatePtr state_ptr, const Eigen::Vector3d pos_enu_inno_, const RtkCrossValidation::DiffInfo &diffinfo, const std::pair<double, Eigen::Vector2d> &pos_inno_info_);
    double get_state_stability_index_() {
        return state_stability_index_;
    }
    double get_state_stability_index_ratio_() {
        return state_stability_index_ / SSI_MAXIMUM_;
    }
    void set_state_stability_index_(double new_ssi_) {
        state_stability_index_ = new_ssi_;
        if (state_stability_index_ > SSI_MAXIMUM_) {
            state_stability_index_ = SSI_MAXIMUM_;
        }

        if (state_stability_index_ < SSI_MINIMUM_) {
            state_stability_index_ = SSI_MINIMUM_;
        }
    }
};

class RtkStatusAnalysis {
public:
    RtkStatusAnalysis() {
        rtk_state_statistic.set_state_max_num(STATE_SEQ_MAX_SIZE);
    }

private:
    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

    /*
        RTK假固定检测
    */
public:
    bool RTK_false_fix_detect(const GnssDataPtr gps_data_ptr, const StatePtr state_ptr, const Eigen::Vector3d pos_inno_, const Eigen::Vector3d pos_inno_rfu_, double ssi_ratio);
    bool is_rtk_false_fix_ = false;

private:
    double                  rffd_pre_mileage_                  = 0.0;
    double                  rffd_pre_timestamp_                = 0.0;
    double                  pre_gnss_timestamp_                = 0.0;
    double                  rffd_persistence_mileage_          = 0.0;
    double                  rffd_persistence_duration_         = 0.0;
    double                  rffd_dynamic_persistence_duration_ = 0.0;
    static constexpr double RFFD_MAXIMUM_MILEAGE_              = 50.0;
    static constexpr double RFFD_MAXIMUM_DYNAMIC_DURATION_     = 20.0;
    static constexpr double RFFD_MAXIMUM_DURATION_             = 300.0;

    double                  rffd_mileage_when_stop_          = 0.0; // or very low speed
    static constexpr double RFFD_MAXIMUM_MILEAGE_AFTER_STOP_ = 0.5;

    /*
        RTK估计解多帧判断
    */
public:
    bool rtk_fix_valid_judgement(GnssDataPtr gps_data_ptr, uint64_t bound_);
    bool is_rtk_fix_valid_ = false;

private:
    uint64_t continous_rtk_fix_count_ = 0;

    /*
        RTK整体性状态评估
    */
public:
    void rtk_overall_state_analysis(const GnssDataPtr gps_data_ptr, const StatePtr state_ptr);

private:
    constexpr static uint64_t   STATE_SEQ_MAX_SIZE = 100;
    statistics::StateStatistics rtk_state_statistic;
};

class PositionCompensation {
private:
    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

public:
    PositionCompensation() {
        ego_pos_inno_statistic.set_max_size(15);
        ref_pos_inno_statistic.set_max_size(20);
    }

public:
    statistics::StateMeanStd ego_pos_inno_statistic;
    statistics::StateMeanStd ref_pos_inno_statistic;

private:
    byd::geo::LocalTrans local_trans;

public:
    // 自车位置补偿
    bool     ego_position_compensation(const StatePtr state_ptr, const Eigen::Vector3d &pos_inno);
    uint64_t should_compensation_count_        = 0;
    uint64_t continous_dynamic_gnss_good_inno_ = 0;

private:
    static constexpr uint64_t long_distance_outage_longi_compe_max_count_ = 30;

public:
    uint64_t longi_compe_count_         = 0;
    double   last_mileage_with_gnss_fix = 0.0;
    double   delta_mileage              = 0.0;

public:
    bool mapping_bias_ablation_and_compensation(const Eigen::Vector3d &lla0_, Eigen::Vector3d &lla_, const INS::EARTH &earth_, StatePtr state_ptr);
};

class GnssFusionControl {
private:
    byd::geo::LocalTrans local_trans;

private:
    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

private:
    uint64_t                                inno_count       = 0;
    static constexpr int                    INNO_BUFFER_SKIP = 2;
    static constexpr int                    INNO_BUFFER_SIZE = 6;
    std::deque<Eigen::Matrix<double, 8, 1>> inno_buffer; // timestamp pos vel heading

public:
    void reset_state_control(const GnssDataPtr gps_data_ptr, const StatePtr state_ptr, bool is_rtk_fix_valid);

public:
    void msf_align_type_control(const GnssDataPtr gps_data_ptr, const StatePtr state_ptr, const Eigen::Matrix<double, 8, 1> &inno_);

public:
    // 整个队列，专门负责对位置进行判断
    // 有些场景，载体的速度会非常慢地蠕行。比如刚出隧道但是极度堵车的场景。这个时候虽然航向和速度均不可靠，但是位置信息大概率是可靠的。
    uint64_t                                pos_inno_count       = 0;
    static constexpr int                    POS_INNO_BUFFER_SKIP = 2;
    static constexpr int                    POS_INNO_BUFFER_SIZE = 6;
    std::deque<Eigen::Matrix<double, 4, 1>> pos_inno_buffer; // timestamp pos
};

class GpsProcessor {

public:
    GpsProcessor() {
        lever_imu2gnss       = parameters_sgt.get_lever_imu2gnss();
        gnss_data_refresh_dt = parameters_sgt.get_gnss_data_refresh_dt();
        for (int i = 0; i <= 6; i++) {
            gnss_position_additional_std_scale[i] = parameters_sgt.get_gnss_position_additional_std_scale(Parameters::GNSS_STATUS(i));
            gnss_velocity_additional_std_scale[i] = parameters_sgt.get_gnss_velocity_additional_std_scale(Parameters::GNSS_STATUS(i));
            gnss_heading_additional_std_scale[i]  = parameters_sgt.get_gnss_heading_additional_std_scale(Parameters::GNSS_STATUS(i));
        }
        constrain_gyro_bias = parameters_sgt.get_constrain_gyro_bias();
        constrain_acc_bias  = parameters_sgt.get_constrain_acc_bias();

        igg3_pos = IGG3(parameters_sgt.get_gnss_pos_igg3_k0(), parameters_sgt.get_gnss_pos_igg3_k1());
        igg3_vel = IGG3(parameters_sgt.get_gnss_vel_igg3_k0(), parameters_sgt.get_gnss_vel_igg3_k1());
        igg3_hdg = IGG3(parameters_sgt.get_gnss_hdg_igg3_k0(), parameters_sgt.get_gnss_hdg_igg3_k1());

        igg3_pos_norm = IGG3(parameters_sgt.get_gnss_pos_igg3_k0(), parameters_sgt.get_gnss_pos_igg3_k1());
        igg3_vel_norm = IGG3(parameters_sgt.get_gnss_vel_igg3_k0(), parameters_sgt.get_gnss_vel_igg3_k1());

        enable_igg = false;

        count = 0;

        tcmsf_not_init_                = false;
        pos_inno_statistic_info_.first = 0.0;
        pos_inno_statistic_info_.second << 1e10, 1e10;
    }

public:
    bool UpdateStateByGps(GnssDataPtr gps_data_ptr, StatePtr state_ptr, KfPtr<21, 3> kf_ptr, const KinematicDataPtr dr_ptr);

private:
    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

private:
    Eigen::Vector3d lever_imu2gnss; // IMU to GNSS antenna
    double          gnss_data_refresh_dt;
    double          gnss_position_additional_std_scale[7] = {0};
    double          gnss_velocity_additional_std_scale[7] = {0};
    double          gnss_heading_additional_std_scale[7]  = {0};
    Eigen::Vector3d constrain_gyro_bias;
    Eigen::Vector3d constrain_acc_bias;

private:
    byd::geo::LocalTrans local_trans;

private:
    uint64_t gnss_rtk_fix_count = 0;

private:
    IGG3 igg3_pos;
    IGG3 igg3_vel;
    IGG3 igg3_hdg;
    IGG3 igg3_pos_norm;
    IGG3 igg3_vel_norm;

    int count;

private:
    double fix_sat_num_mean = 0.0;

private:
    bool enable_igg;

private:
    void gnss_data_pre_process(GnssDataPtr gps_data_ptr, StatePtr state_ptr, const RtkCrossValidation::DiffInfo &diffinfo);

private:
    double pre_gnss_measurement_timestamp = 0.0;

private:
    SinsStableIndex      ssi_;
    RtkStatusAnalysis    rsa_;
    PositionCompensation pos_cmp_;
    GnssFusionControl    gfc_;

private:
    Eigen::Vector3d positionUpdateLinearBalance(const Eigen::Vector3d &pos_inno_, const Eigen::Vector3d &pos_update_, const INS::EARTH &earth_);

private:
    RtkCrossValidation rtk_cross_validation;

private:
    bool tcmsf_not_init_;

private:
    WheelSpeedScaleAdapter wheel_spd_scale_adapter;

private:
    std::pair<double, Eigen::Vector2d> pos_inno_statistic_info_; // timestamp, l1 mean, l1 std

private:
    // 该标志位主要用来抵抗精度非常差的单点解或者差分解状态的卫星定位信息
    // 主要出现在有卫星转发器的隧道
    bool bad_gnss_detected_ = false;

public:
    void set_ssi_(double new_ssi_) {
        ssi_.set_state_stability_index_(new_ssi_);
    }
};

} // namespace MSF