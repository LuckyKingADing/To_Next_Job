#pragma once

#include "analysis.h"
#include "base_type.h"
#include "kalman_filter.h"
#include "sins_interface.h"

#include "calc.h"
#include "state_analysis.h"

#include "local_trans.h"
#include "signal_filter.h"

#include <deque>
namespace MSF {

constexpr int    kImuDataBufferLength = 100;
constexpr double kAccStdLimit         = 1.0;

class Initializer {

private:
    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

public:
    Initializer(KfPtr<21, 3> kf_ptr);

public:
    void AddImuData(const ImuDataPtr imu_data_ptr, double dt_);
    bool AddGnssData(const GnssDataPtr gps_data_ptr, StatePtr state_ptr, KfPtr<21, 3> kf_ptr, const KinematicDataPtr delta_ptr);

private:
    GnssData                pre_gnss_data_;
    bool                    pre_gnss_data_ready_  = false;
    uint64_t                gnss_hdg_ready_count_ = 0;
    uint64_t                gnss_pos_ready_count_ = 0;
    byd::geo::LocalTrans    local_trans;
    static constexpr double kDmileageOffsetRatioThreshold = 0.3;
    static constexpr double kDmileageThresholdMeters      = 0.2;
    static constexpr double kDmileageMaxThresholdMeters   = 0.6;
    static constexpr double kHeadingDiffThresholdRad      = 3.0 / 180.0 * M_PI;
    static constexpr int    kGnssReadyCountThreshold      = 6;

private:
    bool                   EstimateAttitudeFromImuData(Eigen::Quaterniond &attitude_);
    std::deque<ImuDataPtr> imu_buffer_;

private:
    statistics::StateMeanStd gyro_statistics;
    Eigen::Vector3d          gyro_bias_init_estimate_;

private:
    Eigen::Vector3d constrain_gyro_bias;

private:
    byd::geo::Trans geotrans;
};

class AttReference {
private:
    VDSA      vdsa;
    AS        as_lat, as_lon;
    VLPF<3>   vlpf;
    const int VDSA_SKIP  = 4;
    uint64_t  step_count = 0;

public:
    // 这里需要计算两个参考状态
    // 一个是tcmsf的，一个是内部dr的
    void AttRef(const ImuData &cur_imu, const VehicleData &cur_veh, double dt_, StatePtr state_ptr, const KinematicDataPtr inner_motion_state_ptr);
};

class ImuProcessor {
public:
    ImuProcessor() {
        acc_rw_  = parameters_sgt.get_acc_RW();
        acc_ND_  = parameters_sgt.get_acc_ND();
        gyro_rw_ = parameters_sgt.get_gyro_RW();
        gyro_ND_ = parameters_sgt.get_gyro_ND();

        wheel_yaw_rw       = parameters_sgt.get_wheel_yaw_rw();
        wheel_spd_scale_rw = parameters_sgt.get_wheel_spd_scale_rw();
        wheel_pitch_rw     = parameters_sgt.get_wheel_pitch_rw();

        map_pos_east_bias_rw  = parameters_sgt.get_map_pos_east_bias_rw();
        map_pos_north_bias_rw = parameters_sgt.get_map_pos_north_bias_rw();
        map_heading_bias_rw   = parameters_sgt.get_map_heading_bias_rw();

        sins_ptr            = INS::Sins::create();
        lever_imu2vehicle   = parameters_sgt.get_lever_imu2vehicle();
        imu_data_refresh_dt = parameters_sgt.get_imu_data_refresh_dt();
        vdsa_ptr            = std::make_shared<VDSA>();

        constrain_P_std_min = parameters_sgt.get_constrain_P_std_min();
        constrain_P_std_max = parameters_sgt.get_constrain_P_std_max();
        pre_att             = Eigen::Quaterniond::Identity();

        imu_msg_count_ = 0;

        init_P_     = parameters_sgt.get_init_P();
        init_P_cov_ = init_P_.cwiseAbs2();
    }

private:
    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

public:
    std::unique_ptr<INS::Sins> sins_ptr = nullptr;
    void                       Predict(const ImuDataPtr cur_imu, double dt_, StatePtr state_ptr, KfPtr<21, 3> kf_ptr);
    void                       Feedback(const StatePtr state_ptr, bool init_set);

private:
    double          acc_rw_;
    double          acc_ND_;
    double          gyro_rw_;
    double          gyro_ND_;
    Eigen::Vector3d lever_imu2vehicle;
    double          imu_data_refresh_dt;

    double wheel_pitch_rw;
    double wheel_spd_scale_rw;
    double wheel_yaw_rw;

    double map_pos_east_bias_rw;
    double map_pos_north_bias_rw;
    double map_heading_bias_rw;

private:
    std::shared_ptr<VDSA> vdsa_ptr = nullptr;
    AS                    as_lat, as_lon;

private:
    Eigen::Matrix<double, 21, 1> constrain_P_std_min;
    Eigen::Matrix<double, 21, 1> constrain_P_std_max;

private:
    VLPF<6>            vlpf_imu_;
    Eigen::Quaterniond pre_att;

public:
    uint64_t imu_msg_count_;

    Eigen::Matrix<double, 21, 1> init_P_     = Eigen::Matrix<double, 21, 1>::Zero();
    Eigen::Matrix<double, 21, 1> init_P_cov_ = Eigen::Matrix<double, 21, 1>::Zero();
};

} // namespace MSF