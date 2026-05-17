#pragma once

#include "Eigen/Eigen"

#include "base_type.h"

#include "kalman_filter.h"

#include "igg3.h"

namespace MSF {

class WheelSpeedScaleAdapter {

    // 使用分段的线性函数拟合

public:
    WheelSpeedScaleAdapter() {
        wheel_scale_adapter_00mps = parameters_sgt.get_wheel_scale_adapter_00mps();
        wheel_scale_adapter_10mps = parameters_sgt.get_wheel_scale_adapter_10mps();
        wheel_scale_adapter_20mps = parameters_sgt.get_wheel_scale_adapter_20mps();
        wheel_scale_adapter_30mps = parameters_sgt.get_wheel_scale_adapter_30mps();
        wheel_scale_adapter_40mps = parameters_sgt.get_wheel_scale_adapter_40mps();
    }

private:
    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

private:
    double wheel_scale_adapter_00mps;
    double wheel_scale_adapter_10mps;
    double wheel_scale_adapter_20mps;
    double wheel_scale_adapter_30mps;
    double wheel_scale_adapter_40mps;

private:
    double spd2delta_scale(double x) {

        constexpr double px0 = 0.0;
        constexpr double px1 = 10.0;
        constexpr double px2 = 20.0;
        constexpr double px3 = 30.0;
        constexpr double px4 = 40.0;

        double py0 = wheel_scale_adapter_00mps;
        double py1 = wheel_scale_adapter_10mps;
        double py2 = wheel_scale_adapter_20mps;
        double py3 = wheel_scale_adapter_30mps;
        double py4 = wheel_scale_adapter_40mps;

        if (x <= px0)
            return py0;

        if (px1 >= x && x > px0)
            return py0 + (py1 - py0) / (px1 - px0) * (x - px0);

        if (px2 >= x && x > px1)
            return py1 + (py2 - py1) / (px2 - px1) * (x - px1);

        if (px3 >= x && x > px2)
            return py2 + (py3 - py2) / (px3 - px2) * (x - px2);

        if (px4 >= x && x > px3)
            return py3 + (py4 - py3) / (px4 - px3) * (x - px3);

        if (x > px4)
            return wheel_scale_adapter_40mps;

        return 0.0;
    }

public:
    double operator()(double k_, double spd_) {
        double dk = 0.0;
        if (true) {
            dk = spd2delta_scale(spd_);
            if (std::abs(k_) < 0.01) {
                dk = (k_ / 0.01) * dk;
            }
        }
        return k_ + dk;
    }
};

// 悬挂补偿。
class VehicleSuspensionCompensation {
    // 这里选择m/s^2和度的关系

private:
    double MaxPitch;
    double MinPitch;
    double LonAccScale_Positive;
    double LonAccScale_Negative;

private:
    double MaxRoll;
    double MinRoll;
    double LatAccScale_Positive;
    double LatAccScale_Negative;

public:
    VehicleSuspensionCompensation() {
        MaxPitch             = 2.0;  // 单位是度
        MinPitch             = -2.0; // 单位是度
        LonAccScale_Positive = 0.24; // 前向加速与抬头的关系（正值）
        LonAccScale_Negative = 0.13; // 前向减速与前倾的关系（正值）
        MaxRoll              = 3.0;  // 单位是度
        MinRoll              = -3.0; // 单位是度
        LatAccScale_Positive = 0.3;  // 左向加速与右倾的关系（正值）
        LatAccScale_Negative = 0.3;  // 右向加速与左倾的关系（正值）
    }

    VehicleSuspensionCompensation(
        double MaxPitch_,
        double MinPitch_,
        double LonAccScale_Positive_,
        double LonAccScale_Negative_,
        double MaxRoll_,
        double MinRoll_,
        double LatAccScale_Positive_,
        double LatAccScale_Negative_) {
        MaxPitch             = MaxPitch_;
        MinPitch             = MinPitch_;
        LonAccScale_Positive = LonAccScale_Positive_;
        LonAccScale_Negative = LonAccScale_Negative_;
        MaxRoll              = MaxRoll_;
        MinRoll              = MinRoll_;
        LatAccScale_Positive = LatAccScale_Positive_;
        LatAccScale_Negative = LatAccScale_Negative_;
    }

public:
    //输入单位是m/s^2
    //输出单位是度
    double PitchCompensate(double lon_acc_) {
        double pitch_ = 0.0;
        if (lon_acc_ > 0.0) {
            pitch_ = LonAccScale_Positive * lon_acc_;
            pitch_ = pitch_ > MaxPitch ? MaxPitch : pitch_;
        } else {
            pitch_ = LonAccScale_Negative * lon_acc_;
            pitch_ = pitch_ < MinPitch ? MinPitch : pitch_;
        }
        return pitch_;
    }

    //输入单位是m/s^2
    //输出单位是度
    double RollCompensate(double lat_acc_) {
        double roll_ = 0.0;
        if (lat_acc_ > 0.0) {
            roll_ = LatAccScale_Positive * lat_acc_;
            roll_ = roll_ > MaxRoll ? MaxRoll : roll_;
        } else {
            roll_ = LatAccScale_Negative * lat_acc_;
            roll_ = roll_ < MinRoll ? MinRoll : roll_;
        }
        return roll_;
    }

    void SetPitchPara(
        double MaxPitch_,
        double MinPitch_,
        double LonAccScale_Positive_,
        double LonAccScale_Negative_) {
        MaxPitch             = MaxPitch_;
        MinPitch             = MinPitch_;
        LonAccScale_Positive = LonAccScale_Positive_;
        LonAccScale_Negative = LonAccScale_Negative_;
    }

    void SetRollPara(
        double MaxRoll_,
        double MinRoll_,
        double LatAccScale_Positive_,
        double LatAccScale_Negative_) {
        MaxRoll              = MaxRoll_;
        MinRoll              = MinRoll_;
        LatAccScale_Positive = LatAccScale_Positive_;
        LatAccScale_Negative = LatAccScale_Negative_;
    }
};

class VehicleSideSlipCompensate {

    // 这里考虑侧滑角补偿
    // 对于高机动状态的场景，比如高速、大角度的转弯，轮胎需要提供向心力
    // 这个时候轮胎会产生形变，导致车辆方向与行使方向不一致
private:
    constexpr static double MAX_K = 1.0 / 2.5 / 180.0 * M_PI;
    constexpr static double MIN_K = 1.0 / 6.2 / 180.0 * M_PI;

    double K = 1.0 / 3.2 / 180.0 * M_PI;

public:
    VehicleSideSlipCompensate() {
        slip_R      = Eigen::Matrix3d::Identity();
        slip_angle_ = 0.0;
    }
    void   update_slip_R(Eigen::Vector3d &vel_, Eigen::Vector3d &gyro_, double timestamp_);
    void   update_K(double gyro_z, Eigen::Vector3d vel_ego);
    double get_K() {
        return K;
    }
    Eigen::Matrix3d get_slip_R() {
        return slip_R;
    }
    double get_slip_angle() {
        return slip_angle_;
    }

private:
    Eigen::Matrix3d slip_R;
    double          slip_angle_;
};

class VehicleProcessor {

private:
    const double dt_bound = 1e-10;

public:
    VehicleProcessor() {

        wheel_velocity_additional_std_scale = parameters_sgt.get_wheel_velocity_additional_std_scale();
        wheel_data_refresh_dt               = parameters_sgt.get_wheel_data_refresh_dt();
        imu_data_refresh_dt                 = parameters_sgt.get_imu_data_refresh_dt();
        constrain_euler_angle_imu2vehicle   = parameters_sgt.get_constrain_euler_angle_imu2vehicle();
        constrain_wheel_speed_bias          = parameters_sgt.get_constrain_wheel_speed_bias();

        vehinfo_msg_skip = parameters_sgt.get_wheel_msg_skip();
        if (std::abs(wheel_data_refresh_dt) < dt_bound) {
            wheel_data_refresh_dt = dt_bound;
        }

        lever_vehicle2imu = -parameters_sgt.get_lever_imu2vehicle();

        igg3 = IGG3(parameters_sgt.get_wheel_spd_igg3_k0(), parameters_sgt.get_wheel_spd_igg3_k1());

        pre_timestamp          = 0.0;
        wheel_spd_mean_        = 0.0;
        tcmsf_spd_mean_        = 0.0;
        high_dynamic_timestamp = 0.0;
    }

private:
    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

public:
    bool UpdateStateByVehicle(const VehicleDataPtr vehicle_data_ptr, StatePtr state_ptr, KfPtr<21, 3> kf_ptr, const KinematicDataPtr delta_ptr);

private:
    Eigen::Matrix3d MvkD = Eigen::Matrix3d::Zero();

    Eigen::Vector3d velocity_cov_std;
    double          wheel_velocity_additional_std_scale;
    double          wheel_data_refresh_dt;
    double          imu_data_refresh_dt;
    Eigen::Vector3d constrain_euler_angle_imu2vehicle;
    double          constrain_wheel_speed_bias;

    Eigen::Vector3d lever_vehicle2imu;

    IGG3 igg3;

private:
    uint64_t vehinfo_msg_skip  = 10;
    uint64_t vehinfo_msg_count = 0;

private:
    WheelSpeedScaleAdapter wheel_spd_scale_adapter;

private:
    double pre_timestamp;

private:
    double high_dynamic_timestamp;

private:
    double   wheel_spd_mean_;
    double   tcmsf_spd_mean_;
    uint64_t high_spd_msg_count = 0;

private:
    VehicleSuspensionCompensation vehicle_suspension_compensation;
    VehicleSideSlipCompensate     vehicle_side_slip_compensation;
};

} // namespace MSF