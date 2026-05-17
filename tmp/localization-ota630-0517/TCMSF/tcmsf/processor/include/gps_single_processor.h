#pragma once

#include "Eigen/Eigen"

#include "base_type.h"
#include "earth.h"
#include "igg3.h"
#include "kalman_filter.h"
#include "local_trans.h"
#include "state_analysis.h"
#include "traj_analysis.h"
#include <iomanip>

#include <array>
#include <deque>

namespace MSF {

// 通过DR、MSF、GNSS三者做一个交叉验证，判断MSF状态是否OK
// 即通过DR+GNSS判断GNSS是否正常，如果GNSS正常，则通过GNSS+MSF判断MSF是否正常
// 如果判断出MSF不正常，那么后续做MSF的重置
class CrossValidation {
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
    double gnss_dr_diff_index = 0.0;

public:
    double get_gnss_dr_diff_index() {
        return gnss_dr_diff_index;
    }

private:
    static constexpr uint64_t BUFFER_SIZE   = 7;
    static constexpr uint64_t GNSS_MSG_SKIP = 3;
    uint64_t                  msg_count     = 0;

private:
    std::deque<NaviInfo> navi_info_buffer;

private:
    byd::geo::LocalTrans local_trans;
    bool                 msf_invalid = false;

private:
    double fusion_valid_index = 0.0;

public:
    void   insert(const GnssDataPtr gps_data_ptr_, const StatePtr state_ptr_gpst_, const KinematicDataPtr dr_ptr_gpst_);
    bool   is_msf_invalid() { return msf_invalid; }
    double get_fusion_valid_index() { return fusion_valid_index; }

private:
    double                  ssi_pre_mileage_       = 0.0;
    double                  ssi_pre_timestamp_     = 0.0;
    double                  state_stability_index_ = 0.0;
    static constexpr double SSI_MAXIMUM_           = 6000.0;
    static constexpr double SSI_MINIMUM_           = 0.0;

private:
    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

private:
    void state_stability_index_update(const GnssDataPtr gps_data_ptr, const StatePtr state_ptr, double gnss_diff_, double msf_diff_);

public:
    double get_state_stability_index_() {
        return state_stability_index_;
    }
    double get_state_stability_index_ratio_() {
        return state_stability_index_ / SSI_MAXIMUM_;
    }
};

class GpsSingleProcessor {
public:
    GpsSingleProcessor() {
        lever_imu2gnss       = parameters_sgt.get_lever_imu2gnss();
        gnss_data_refresh_dt = parameters_sgt.get_gnss_data_refresh_dt();

        constrain_gyro_bias = parameters_sgt.get_constrain_gyro_bias();
        constrain_acc_bias  = parameters_sgt.get_constrain_acc_bias();

        igg3_pos = IGG3(parameters_sgt.get_gnss_pos_igg3_k0(), parameters_sgt.get_gnss_pos_igg3_k1());
        igg3_vel = IGG3(parameters_sgt.get_gnss_vel_igg3_k0(), parameters_sgt.get_gnss_vel_igg3_k1());
        igg3_hdg = IGG3(parameters_sgt.get_gnss_hdg_igg3_k0(), parameters_sgt.get_gnss_hdg_igg3_k1());

        igg3_pos_norm = IGG3(parameters_sgt.get_gnss_pos_igg3_k0(), parameters_sgt.get_gnss_pos_igg3_k1());
        igg3_vel_norm = IGG3(parameters_sgt.get_gnss_vel_igg3_k0(), parameters_sgt.get_gnss_vel_igg3_k1());
    }

public:
    bool UpdateStateByGpsSingle(GnssDataPtr gps_data_ptr, StatePtr state_ptr, KfPtr<21, 3> kf_ptr, const KinematicDataPtr dr_ptr);

private:
    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

private:
    double               gnss_data_refresh_dt;
    double               pre_gnss_measurement_timestamp = 0.0;
    Eigen::Vector3d      lever_imu2gnss; // IMU to GNSS antenna
    Eigen::Vector3d      constrain_gyro_bias;
    Eigen::Vector3d      constrain_acc_bias;
    byd::geo::LocalTrans local_trans;

private:
    IGG3 igg3_pos;
    IGG3 igg3_vel;
    IGG3 igg3_hdg;
    IGG3 igg3_pos_norm;
    IGG3 igg3_vel_norm;

private:
    void            MeasurementDataPreProcess(GnssDataPtr gps_data_ptr, StatePtr state_ptr);
    Eigen::Vector3d EnuUpdateLinearBalance(const Eigen::Vector3d &pos_inno_, const Eigen::Vector3d &pos_update_, const INS::EARTH &earth_);

private:
    CrossValidation cross_validate;

private:
    // 卫星定位好情况下，卫星数量均值
    double good_state_sat_num_mean_ = 0.0;

    // 该标志位主要用来抵抗精度非常差的单点解或者差分解状态的卫星定位信息
    // 主要出现在有卫星转发器的隧道
    bool bad_gnss_detected_ = false;

private:
    TRAJ::TrajectoryCollector traj_info{15, 14.0, 2.0};
    uint64_t                  gnss_msg_count_         = 0;
    static constexpr double   kFrechetThresholdMeters = 2.0;
    static constexpr double   kRmseThresholdMeters    = 40.0;
};
} // namespace MSF