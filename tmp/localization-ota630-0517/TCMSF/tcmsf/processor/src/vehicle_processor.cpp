#include "vehicle_processor.h"

#include "calc.h"

#include "cyber/common/log.h"
#include "fmt/format.h"
#include "gps_processor.h"
#include "rigid_transform.h"

#include "processor_debug.h"
#include <iomanip>

namespace MSF {

// 需要估计车体到IMU的俯仰角偏差、航向角偏差、轮速误差系数，三个状态

bool VehicleProcessor::UpdateStateByVehicle(const VehicleDataPtr vehicle_data_ptr, StatePtr state_ptr, KfPtr<21, 3> kf_ptr, const KinematicDataPtr delta_ptr) {

    // 每2000个打印一个心跳信息
    AINFO_EVERY(2000) << "[HeartBeat] Vehicle Fusion, TS: " << fmt::format("{:14.4f}", vehicle_data_ptr->measurement_timestamp);

    Eigen::Matrix<double, 3, 21> H;
    Eigen::Vector3d              innovation;

    vehinfo_msg_count++;

    if (vehinfo_msg_count >= vehinfo_msg_skip) { // 对轮速量测进行采样抽稀
        vehinfo_msg_count = 0;

        double dt_cmp_ = 0.0;

        Eigen::Vector3d velocity = Eigen::Vector3d{0.0, (vehicle_data_ptr->speed_rl + vehicle_data_ptr->speed_rr) / 2.0, 0.0};

        if (delta_ptr) {
            // 补偿量映射到轮速上
            velocity.y() = velocity.y() + delta_ptr->ego_longitude_vel / (1.0 + state_ptr->wheel_spd_scale_bias_);
            dt_cmp_      = delta_ptr->measurement_timestamp;
        }

        Eigen::Vector3d gyro_ = state_ptr->gyro - state_ptr->gyro_bias;

        {
            // 此处做一下侧滑角补偿
            vehicle_side_slip_compensation.update_slip_R(velocity, gyro_, vehicle_data_ptr->measurement_timestamp);
            velocity = vehicle_side_slip_compensation.get_slip_R() * velocity;
        }

        double dt_    = (vehicle_data_ptr->measurement_timestamp + dt_cmp_) - pre_timestamp;
        pre_timestamp = (vehicle_data_ptr->measurement_timestamp + dt_cmp_);

        if (dt_ < 0.0 || dt_ > 1.0) {
            // 轮速量测更新的时间差
            double dt = parameters_sgt.get_wheel_data_refresh_dt() * parameters_sgt.get_wheel_msg_skip();
            dt_       = dt;
        }

        // 这里标志位的初值，默认是RTK LC模式

        // 设置车辆相关状态估计的准入条件
        // ALIGNED状态
        bool veh_state_estimate_fix_align_strict_ =
            process_control_sgt.fusion_status.continous_gnss_fix_count > 2 &&      //
            (process_control_sgt.msf_align_type == process_control_sgt.ALIGNED) && //
            state_ptr->rtk_status == Parameters::GNSS_STATUS::FIX;                 //
        // FINE_ALIGN状态，FINE_ALIGN状态可能并不稳定，但是考虑到安装角误差比较大的场景，在没有准确估计出安装角之前，滤波器状态长期保持FINE_ALIGN状态
        // 所以FINE_ALIGN状态也准入进行车辆相关状态的估计，对动态性做个额外的限制
        bool veh_state_estimate_fix_slip_strict_ =
            process_control_sgt.fusion_status.continous_gnss_fix_count > 2 &&                        //
            (process_control_sgt.msf_align_type == process_control_sgt.FINE_ALIGN) &&                //
            state_ptr->rtk_status == Parameters::GNSS_STATUS::FIX &&                                 //
            std::abs(state_ptr->slip_index) < parameters_sgt.get_slip_index_rejection_bound() / 2.0; //
        // 长期FLOAT场景
        bool veh_state_estimate_float_ =
            process_control_sgt.rtk_overall_status == process_control_sgt.MAJORITY_FLOAT &&                                       //
            process_control_sgt.fusion_status.continous_gnss_float_fix_count > 5 &&                                               //
            (state_ptr->rtk_status == Parameters::GNSS_STATUS::FIX || state_ptr->rtk_status == Parameters::GNSS_STATUS::FLOAT) && //
            std::abs(state_ptr->slip_index) < parameters_sgt.get_slip_index_rejection_bound() / 2.0;                              //

        bool veh_state_estimate_ =
            (veh_state_estimate_fix_align_strict_ || veh_state_estimate_fix_slip_strict_ || veh_state_estimate_float_) &&
            state_ptr->vel.norm() > parameters_sgt.get_slow_speed_bound();

        bool veh_low_dynamic_ =
            process_control_sgt.maneuver_status_by_imu == process_control_sgt.IMU_STEADY &&
            std::abs(state_ptr->slip_index) < parameters_sgt.get_slip_index_rejection_bound();

        // fuse velocity of vehicle

        bool gnss_good_status_ =
            std::abs(state_ptr->gnss_inno_(0) - vehicle_data_ptr->measurement_timestamp) < 2.0 &&
            state_ptr->gnss_inno_.block<3, 1>(1, 0).norm() < 0.5;

        bool wheel_spd_scale_coarse_estimate_ =                                                                                   //
            veh_low_dynamic_ &&                                                                                                   //
            (state_ptr->rtk_status == Parameters::GNSS_STATUS::FIX || state_ptr->rtk_status == Parameters::GNSS_STATUS::FLOAT) && //
            std::fabs(state_ptr->mileage) < 10e3;                                                                                 //

        bool state_stability_index_ratio_good_ =
            state_ptr->state_stability_index_ratio_ > 0.7;
        bool state_stability_index_ratio_good_for_wheel_spd_ =
            state_ptr->state_stability_index_ratio_ > 0.04;
        bool gnss_valid_ =
            process_control_sgt.fusion_status.continous_gnss_fix_count > 10 && //
            state_ptr->state_stability_index_ratio_ > 0.03;                    //

        switch (parameters_sgt.get_gnss_fusion_mode()) {
            case parameters_sgt.GNSS_LOOSE_COUPLE: {
                wheel_spd_scale_coarse_estimate_ =
                    veh_low_dynamic_ &&                             //
                    gnss_good_status_ &&                            //
                    std::fabs(state_ptr->mileage) < 10e3 &&         //
                    state_ptr->state_stability_index_ratio_ > 0.02; //
                veh_state_estimate_ =
                    state_ptr->vel.norm() > parameters_sgt.get_slow_speed_bound(); //
                state_stability_index_ratio_good_ =
                    state_ptr->state_stability_index_ratio_ > 0.3;
                state_stability_index_ratio_good_for_wheel_spd_ =
                    state_ptr->state_stability_index_ratio_ > 0.02;
                gnss_valid_ =
                    gnss_good_status_ &&
                    state_ptr->state_stability_index_ratio_ > 0.02;
            } break;

            case parameters_sgt.GNSS_TIGHT_COUPLE: {
                // 暂时没这个模式
            } break;

            case parameters_sgt.RTK_LOOSE_COUPLE: {
                wheel_spd_scale_coarse_estimate_ =                                                                                        //
                    veh_low_dynamic_ &&                                                                                                   //
                    (state_ptr->rtk_status == Parameters::GNSS_STATUS::FIX || state_ptr->rtk_status == Parameters::GNSS_STATUS::FLOAT) && //
                    std::fabs(state_ptr->mileage) < 10e3;                                                                                 //
                veh_state_estimate_ =
                    (veh_state_estimate_fix_align_strict_ || veh_state_estimate_fix_slip_strict_ || veh_state_estimate_float_) && //
                    state_ptr->vel.norm() > parameters_sgt.get_slow_speed_bound();                                                //
                state_stability_index_ratio_good_ =
                    state_ptr->state_stability_index_ratio_ > 0.7;
                state_stability_index_ratio_good_for_wheel_spd_ =
                    state_ptr->state_stability_index_ratio_ > 0.04;
                gnss_valid_ =
                    process_control_sgt.fusion_status.continous_gnss_fix_count > 10 && //
                    state_ptr->state_stability_index_ratio_ > 0.03;                    //
            } break;

            case parameters_sgt.RTK_TIGHT_COUPLE: {
                // 紧组合补充
            } break;

            default: {
            } break;
        }

        { // 计算行使里程
            // 为了方便计算前进距离，认为倒车是负里程
            if (std::abs(dt_) < 1.0) {
                state_ptr->mileage = state_ptr->mileage + (1.0 + state_ptr->wheel_spd_scale_bias_) * velocity.y() * dt_;
            }
        }

        // 低动态场景
        // RTK状态良好
        // 行使里程小于10km时
        // 直接使用速度对比，粗略的估计轮速误差系数

        if (wheel_spd_scale_coarse_estimate_) {
            double veh_spd_   = (1.0 + state_ptr->wheel_spd_scale_bias_) * velocity.norm();
            double tcsmf_spd_ = state_ptr->vel.norm();

            constexpr uint64_t MEAN_COUNT_ = 13;
            constexpr double   MIN_SPD_    = 5.0;

            if (tcsmf_spd_ > MIN_SPD_ && veh_spd_ > MIN_SPD_) {
                if (high_spd_msg_count == 0) {
                    wheel_spd_mean_ = veh_spd_;
                    tcmsf_spd_mean_ = tcsmf_spd_;
                } else {
                    constexpr double alpha = 1.0 / MEAN_COUNT_;

                    wheel_spd_mean_ = wheel_spd_mean_ * (1.0 - alpha) + veh_spd_ * alpha;
                    tcmsf_spd_mean_ = tcmsf_spd_mean_ * (1.0 - alpha) + tcsmf_spd_ * alpha;
                }
                high_spd_msg_count++;
                if (
                    high_spd_msg_count >= MEAN_COUNT_ && //
                    wheel_spd_mean_ > MIN_SPD_ &&        //
                    tcmsf_spd_mean_ > MIN_SPD_           //
                ) {
                    double scale_ = (tcmsf_spd_mean_ - wheel_spd_mean_) / (std::abs(wheel_spd_mean_) + 1e-5);
                    if (std::abs(scale_) < 0.05 && std::abs(scale_) > 0.003) {
                        AINFO_EVERY(100) << "estimate vehicle bias by speed compare";
                        if (std::abs(state_ptr->vehicle_bias.y()) < 1e-5 && std::abs(scale_) < 0.02) {
                            state_ptr->vehicle_bias.y() = scale_ * 0.6;
                        } else {
                            switch (parameters_sgt.get_gnss_fusion_mode()) {
                                case parameters_sgt.GNSS_LOOSE_COUPLE: {
                                    state_ptr->vehicle_bias.y() = state_ptr->vehicle_bias.y() + scale_ * 0.01;
                                } break;

                                default: {
                                    if (gnss_good_status_) {
                                        state_ptr->vehicle_bias.y() = state_ptr->vehicle_bias.y() + scale_ * 0.05;
                                    } else {
                                        state_ptr->vehicle_bias.y() = state_ptr->vehicle_bias.y() + scale_ * 0.02;
                                    }
                                } break;
                            }
                        }
                        StateConstrain(state_ptr->vehicle_bias, {constrain_euler_angle_imu2vehicle.x(), constrain_wheel_speed_bias, constrain_euler_angle_imu2vehicle.z()});
                    }
                    high_spd_msg_count = 0;
                }
            } else {
                high_spd_msg_count = 0;
            }
        }

        // 速度误差来源于车轮转动半径误差，速度越快认为轮速误差越大
        // 同时，轮速由轮脉冲计数计算，在低速时，同样不准。
        // 角速度大的时候，因为侧滑角的影响，轮速方向不准
        double velocity_cov_std_ = parameters_sgt.get_wheel_speed_bias_std();

        if (velocity.norm() < 0.3) {
            // 极低速场景，轮速不可靠
            velocity_cov_std_ = 0.3;
        }
        double velocity_cov_std_scale = (0.1 + velocity.norm());

        double hori_acc_ = (state_ptr->acc - state_ptr->acc_bias).block<2, 1>(0, 0).norm();

        // 此处做个轮速标准差的自适应
        velocity_cov_std << //
            0.01 * hori_acc_ + velocity_cov_std_scale * velocity_cov_std_ * 0.03 + 0.01 + std::abs(state_ptr->slip_index) * 0.02,
            0.01 * hori_acc_ + velocity_cov_std_scale * velocity_cov_std_ + std::abs(state_ptr->slip_index) * 0.1,
            0.01 * hori_acc_ + velocity_cov_std_scale * velocity_cov_std_ * 0.1 + 0.03;

        Eigen::Matrix3d velocity_cov_   = velocity_cov_std.cwiseAbs2().asDiagonal();
        Eigen::Matrix3d velocity_cov_n_ = state_ptr->C_b2n * velocity_cov_ * state_ptr->C_b2n.transpose();

        H.setZero();

        Eigen::Matrix3d T_imu2ref    = state_ptr->att.toRotationMatrix();
        Eigen::Vector3d vehicle_bias = state_ptr->vehicle_bias;
        Eigen::Matrix3d T_veh2imu    = state_ptr->C_imu2vehicle;

        Eigen::Matrix3d T_veh2ref = T_imu2ref * T_veh2imu;

        vehicle_bias.y() = wheel_spd_scale_adapter(vehicle_bias.y(), state_ptr->vel.norm());

        Eigen::Vector3d v_d_n = T_veh2ref * ((1.0 + vehicle_bias.y()) * velocity - INS::Skew(gyro_) * T_veh2imu * parameters_sgt.get_lever_imu2vehicle());

        double vD = (1.0 + vehicle_bias.y()) * velocity.y();

        // 三个状态量 k_D
        // 俯仰角误差、轮速系数误差、方位角误差
        // v^n_ins - v_d^n_measurement = dv^n_ins - v_d^n ✖️ phi_d - MvkD ✖️ k_d

        {
            // double vD  = (T_veh2ref.transpose() * v_d_n).y();
            MvkD(0, 0) = -T_imu2ref(0, 2);
            MvkD(1, 0) = -T_imu2ref(1, 2);
            MvkD(2, 0) = -T_imu2ref(2, 2);

            MvkD(0, 1) = T_imu2ref(0, 1);
            MvkD(1, 1) = T_imu2ref(1, 1);
            MvkD(2, 1) = T_imu2ref(2, 1);

            MvkD(0, 2) = T_imu2ref(0, 0);
            MvkD(1, 2) = T_imu2ref(1, 0);
            MvkD(2, 2) = T_imu2ref(2, 0);

            MvkD = vD * MvkD;
        }

        // // Compute innovation.
        innovation = state_ptr->vel - v_d_n;

        // // Compute jacobian.
        // H.block<3, 3>(0, 0)  = -INS::Skew(v_d_n) * dt_;
        H.block<3, 3>(0, 0)  = -INS::Skew(v_d_n + (state_ptr->acc - state_ptr->acc_bias) * dt_ / 2.0) * dt_;
        H.block<3, 3>(0, 3)  = Eigen::Matrix3d::Identity() * dt_;
        H.block<3, 3>(0, 15) = -MvkD * dt_;

        Eigen::Matrix3d V = ((innovation.cwiseAbs() * wheel_velocity_additional_std_scale).cwiseAbs2()).asDiagonal();
        V                 = V + velocity_cov_n_;

        kf_ptr->kf_update(H, V, innovation);
        // kf_ptr->measurement_update_debug_info();

        { // 更新状态
            state_ptr->vel          = state_ptr->vel - kf_ptr->dx.block<3, 1>(3, 0);
            Eigen::Vector3d    datt = constrain(kf_ptr->dx.block<3, 1>(0, 0), MaxEulrAnglePerMeasurementUpdate);
            Eigen::Quaterniond dq   = INS::rv2q(datt);
            state_ptr->att          = dq * state_ptr->att;
            state_ptr->att.normalize();

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
            Eigen::Vector3d Pveh = kf_ptr->P.block<3, 3>(15, 15).diagonal().cwiseSqrt();
            debug::debug_sgt.veh_state.line( //
                fmt::format(                 //
                    "{:>14.4f},{: >5.5f},{: >5.5f},{: >5.5f},{: >5.5f},{: >5.5f},{: >5.5f},{: >5.5f},{: >5.5f},{: >5.5f},{: >5.5f},"
                    "{: >5.5f},{: >5.5f},{: >5.5f},{: >5.5f},{:d},{: >5.5f},{: >5.5f},{: >5.5f},{: >5.5f},{: >5.5f},"
                    "{:d},{:d},{:d},{: >5.5f},{:d},{: >5.5f},{:d},{:d},{: >6.6f}\n", //

                    vehicle_data_ptr->measurement_timestamp + dt_cmp_,                   // 1
                    state_ptr->vehicle_bias.x() * 180 / M_PI,                            // 2
                    state_ptr->vehicle_bias.y(),                                         // 3
                    state_ptr->vehicle_bias.z() * 180 / M_PI,                            // 4
                    Pveh.x() * 180 / M_PI,                                               // 5
                    Pveh.y(),                                                            // 6
                    Pveh.z() * 180 / M_PI,                                               // 7
                    state_ptr->gyro.z() * 180 / M_PI,                                    // 8
                    state_ptr->vel.norm(),                                               // 9
                    state_ptr->gyro.z() * state_ptr->vel.norm(),                         // 10
                    std::abs((1.0 + state_ptr->wheel_spd_scale_bias_) * velocity.y()),   // 11
                    innovation.x(),                                                      // 12
                    innovation.y(),                                                      // 13
                    innovation.z(),                                                      // 14
                    velocity.y(),                                                        // 15
                    veh_state_estimate_,                                                 // 16
                    vehicle_bias.y(),                                                    // 17
                    wheel_spd_mean_,                                                     // 18
                    tcmsf_spd_mean_,                                                     // 19
                    vehicle_data_ptr->acc_lat,                                           // 20
                    vehicle_data_ptr->acc_lgt,                                           // 21
                    (int64_t)process_control_sgt.fusion_status.continous_gnss_fix_count, // 22
                    (int64_t)process_control_sgt.msf_align_type,                         // 23
                    (int64_t)state_ptr->rtk_status,                                      // 24
                    std::abs(state_ptr->slip_index),                                     // 25
                    (int64_t)process_control_sgt.maneuver_status_by_imu,                 // 26
                    state_ptr->gnss_inno_.block<3, 1>(1, 0).norm(),                      // 27
                    veh_state_estimate_,                                                 // 28
                    veh_low_dynamic_,                                                    // 29
                    vehicle_side_slip_compensation.get_slip_angle() * 180.0 / M_PI       // 30
                    )                                                                    //
            );
#endif

            Eigen::Matrix<bool, 21, 1> idx = Eigen::Matrix<bool, 21, 1>::Ones();
            idx.block<9, 1>(0, 0) << 0, 0, 0, 0, 0, 0, 0, 0, 0;

            if (gnss_valid_) {
                // 如果没有完成零速更新，则对IMU的零偏进行估计
                state_ptr->gyro_bias += kf_ptr->dx.block<3, 1>(9, 0);
                Eigen::Vector3d constrain_gyro_bias = parameters_sgt.get_constrain_gyro_bias();
                StateConstrain(state_ptr->gyro_bias, constrain_gyro_bias);
                idx.block<3, 1>(9, 0) << 0, 0, 0;

                state_ptr->acc_bias += kf_ptr->dx.block<3, 1>(12, 0);
                Eigen::Vector3d constrain_acc_bias = parameters_sgt.get_constrain_acc_bias();
                StateConstrain(state_ptr->acc_bias, constrain_acc_bias);
                idx.block<3, 1>(12, 0) << 0, 0, 0;
            }

            if (veh_state_estimate_) {
                // 限制角度估计的修正速度
                // 避免出现显著的跳变
                double pitch_bias_     = constrain(kf_ptr->dx(15), 0.002 / 180.0 * M_PI);
                double spd_scale_bias_ = 0.0;
                double yaw_bias_       = 0.0;

                yaw_bias_       = constrain(kf_ptr->dx(17), 0.002 / 180.0 * M_PI);
                spd_scale_bias_ = constrain(kf_ptr->dx(16), 0.0003);

                if (!gnss_good_status_) {
                    yaw_bias_       = constrain(kf_ptr->dx(17), 0.001 / 180.0 * M_PI);
                    spd_scale_bias_ = constrain(kf_ptr->dx(16), 0.00005);
                }

                if (veh_low_dynamic_) {
                    process_control_sgt.good_state_for_vehicle_bias_estimation = std::make_pair(vehicle_data_ptr->measurement_timestamp, true);
                } else {
                    process_control_sgt.good_state_for_vehicle_bias_estimation = std::make_pair(vehicle_data_ptr->measurement_timestamp, false);
                }

                if (veh_low_dynamic_ && state_stability_index_ratio_good_) {

                    state_ptr->vehicle_bias(0) -= pitch_bias_;
                    state_ptr->vehicle_bias(1) -= spd_scale_bias_;
                    state_ptr->vehicle_bias(2) -= yaw_bias_;

                    state_ptr->wheel_spd_scale_bias_ = wheel_spd_scale_adapter(state_ptr->vehicle_bias(1), state_ptr->vel.norm());

                    state_ptr->q_imu2vehicle = INS::euler2quaternion({state_ptr->vehicle_bias.x(), 0.0, state_ptr->vehicle_bias.z()});
                    state_ptr->C_imu2vehicle = state_ptr->q_imu2vehicle.toRotationMatrix().transpose();

                    idx.block<3, 1>(15, 0) << 0, 0, 0;
                } else {

                    if (veh_low_dynamic_ && state_stability_index_ratio_good_for_wheel_spd_) {
                        state_ptr->vehicle_bias(1) -= spd_scale_bias_;
                        state_ptr->wheel_spd_scale_bias_ = wheel_spd_scale_adapter(state_ptr->vehicle_bias(1), state_ptr->vel.norm());
                        idx.block<3, 1>(15, 0) << 1, 0, 1;
                    }
                }
            }
            // 更新协方差
            CovarianceUpdateByIdx(kf_ptr->P, kf_ptr->cov, idx, true);
            // kf_ptr->P = kf_ptr->cov;
            StateConstrain(state_ptr->vehicle_bias, {constrain_euler_angle_imu2vehicle.x(), constrain_wheel_speed_bias, constrain_euler_angle_imu2vehicle.z()});
        }
    }
    return true;
}

void VehicleSideSlipCompensate::update_slip_R(Eigen::Vector3d &vel_, Eigen::Vector3d &gyro_, double timestamp_) {
    // 使用经验公式，计算下侧滑角
    double vel_norm_ = vel_.norm();
    if (vel_.y() < 0.0) {
        vel_norm_ = -vel_norm_;
    }
    double slip_angle_ = gyro_.z() * vel_norm_ * K;
    {
        // 做个限制，侧滑角补偿最大量控制在4度以内，以免出现数值异常情况
        slip_angle_ = slip_angle_ > 4.0 / 180.0 * M_PI ? 4.0 / 180.0 * M_PI : slip_angle_;
        slip_angle_ = slip_angle_ < -4.0 / 180.0 * M_PI ? -4.0 / 180.0 * M_PI : slip_angle_;
    }
    if (vel_norm_ > 2.0 && std::fabs(slip_angle_) > 0.1 / 180.0 * M_PI) {
        // 速度为正，且大于一定值的时候，启用侧滑角补偿
        slip_R = Eigen::AngleAxisd(-slip_angle_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    } else {
        slip_R = Eigen::Matrix3d::Identity();
    }
}

void VehicleSideSlipCompensate::update_K(double gyro_z, Eigen::Vector3d vel_ego) {
    // todo : 当前算法还不太行，在线估计K的方法
    double vel_ = vel_ego.norm();
    if (vel_ego.y() < 0.0) {
        vel_ = -vel_;
    }
    double slip_  = gyro_z * vel_;
    double theta_ = std::atan2(vel_ego.x(), vel_ego.y());
    if (vel_ > 5.0 && vel_ego.y() > 3.0 && std::fabs(slip_) > 0.2) {
        double k_ = theta_ / slip_;

        K = 0.999 * K + 0.001 * k_;

        // 控制K的范围
        K = K > MAX_K ? MAX_K : K;
        K = K < MIN_K ? MIN_K : K;

        AINFO_EVERY(20) << "slip K: " << K;
    }
}

} // namespace MSF