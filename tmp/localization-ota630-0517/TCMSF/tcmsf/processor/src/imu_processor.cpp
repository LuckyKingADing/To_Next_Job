#include "imu_processor.h"

#include "Eigen/Cholesky"
#include "Eigen/Dense"
#include "cyber/common/log.h"
#include "local_trans.h"
#include "unsupported/Eigen/MatrixFunctions"

#include "processor_debug.h"

#include "calc.h"

#include <iomanip>

#include "rts_smoother_interface.h"

namespace MSF {

Initializer::Initializer(KfPtr<21, 3> kf_ptr) {
    // 设置滤波器粗对准各个状态
    auto            init_P_ = parameters_sgt.get_init_P();
    Eigen::Vector3d att_std{init_P_(0), init_P_(1), init_P_(2)};
    Eigen::Vector3d vel_std{init_P_(3), init_P_(4), init_P_(5)};
    Eigen::Vector3d pos_std{init_P_(6) / INS::glv.Re, init_P_(7) / INS::glv.Re, init_P_(8)};
    kf_ptr->P.block<3, 3>(0, 0) = att_std.cwiseAbs2().asDiagonal();
    kf_ptr->P.block<3, 3>(3, 3) = vel_std.cwiseAbs2().asDiagonal();
    kf_ptr->P.block<3, 3>(6, 6) = pos_std.cwiseAbs2().asDiagonal();

    Eigen::Vector3d gyro_bias_std{init_P_(9), init_P_(10), init_P_(11)};
    Eigen::Vector3d acc_bias_std{init_P_(12), init_P_(13), init_P_(14)};
    Eigen::Vector3d vehicle_error_std{init_P_(15), init_P_(16), init_P_(17)};
    kf_ptr->P.block<3, 3>(9, 9)   = gyro_bias_std.cwiseAbs2().asDiagonal();
    kf_ptr->P.block<3, 3>(12, 12) = acc_bias_std.cwiseAbs2().asDiagonal();
    kf_ptr->P.block<3, 3>(15, 15) = vehicle_error_std.cwiseAbs2().asDiagonal();

    Eigen::Vector3d map_error_std{init_P_(18), init_P_(19), init_P_(20)};
    kf_ptr->P.block<3, 3>(18, 18) = map_error_std.cwiseAbs2().asDiagonal();

    gyro_statistics.set_max_size(100);
    gyro_bias_init_estimate_ = Eigen::Vector3d::Zero();

    constrain_gyro_bias = parameters_sgt.get_constrain_gyro_bias();
}

void Initializer::AddImuData(const ImuDataPtr imu_data_ptr, double dt_) {
    if (std::abs(dt_) < 1e-5) {
        return;
    };

    // 输入为增量，这里将IMU的值转换为测量值
    ImuDataPtr data_             = std::make_shared<ImuData>();
    data_->measurement_timestamp = imu_data_ptr->measurement_timestamp;
    data_->publish_timestamp     = imu_data_ptr->publish_timestamp;
    data_->sequence_num          = imu_data_ptr->sequence_num;
    data_->acc                   = imu_data_ptr->acc / dt_;
    data_->gyro                  = imu_data_ptr->gyro / dt_;

    {
        // 如果是首次初始化、并且进入了零速状态，则对GYRO的零偏做一个粗略的估计。
        if (process_control_sgt.initialization_count == 0 && process_control_sgt.vehicle_motion_type == process_control_sgt.ZERO_VELOCITY) {
            Eigen::Vector3d gyro_ = data_->gyro;
            if (gyro_.norm() < 0.3 / 180.0 * M_PI) {
                gyro_statistics.insert(gyro_);
            }
            if (gyro_statistics.calc_mean_std()) {
                Eigen::Vector3d mean_       = gyro_statistics.mean();
                Eigen::Vector3d std_        = gyro_statistics.std();
                double          std_bound_  = 0.1 / 180.0 * M_PI;
                double          mean_bound_ = 0.2 / 180.0 * M_PI;
                if (std_.x() < std_bound_ &&
                    std_.y() < std_bound_ &&
                    std_.z() < std_bound_ &&
                    std::abs(mean_.x()) < mean_bound_ &&
                    std::abs(mean_.y()) < mean_bound_ &&
                    std::abs(mean_.z()) < mean_bound_) {
                    gyro_bias_init_estimate_ = gyro_bias_init_estimate_ - 0.01 * (gyro_bias_init_estimate_ - mean_);

                    StateConstrain(gyro_bias_init_estimate_, constrain_gyro_bias);
                }
                gyro_statistics.clear();
            }
        }
    }

    imu_buffer_.push_back(std::move(data_));

    // 控制IMU队列大小
    while (imu_buffer_.size() > kImuDataBufferLength) {
        imu_buffer_.pop_front();
    }
}

bool Initializer::AddGnssData(const GnssDataPtr gps_data_ptr, StatePtr state_ptr, KfPtr<21, 3> kf_ptr, const KinematicDataPtr delta_ptr) {

    double traj_az_ = 0.0;

    if (pre_gnss_data_ready_) {
        traj_az_ = local_trans.HeadingFromBLH(pre_gnss_data_.lla, gps_data_ptr->lla) / 180.0 * M_PI;

        if (traj_az_ > M_PI) {
            traj_az_ = traj_az_ - 2.0 * M_PI;
        }

        {
            // 根据前后帧位置，算一个位移量
            double traj_dmileage_ = local_trans.LLAtoEgoRfu(pre_gnss_data_.lla, gps_data_ptr->lla, Eigen::Quaterniond::Identity(), true).norm();
            // 平面速度乘以时间，算一个位移量
            double vel_dmileage_ = std::fabs((pre_gnss_data_.vel + gps_data_ptr->vel).block<2, 1>(0, 0).norm() / 2.0 * (gps_data_ptr->measurement_timestamp - pre_gnss_data_.measurement_timestamp));

            double dmileage_traj_vel_ = traj_dmileage_ - vel_dmileage_;

            // 这里计算一下，两个位移量的不匹配程度
            double dmileage_offset_ratio_ = std::fabs(dmileage_traj_vel_) / (std::fabs(traj_dmileage_ + vel_dmileage_) / 2.0 + 1e-7);
            double gps_spd_               = gps_data_ptr->vel.block<2, 1>(0, 0).norm();

            AINFO_IF(gnss_hdg_ready_count_ > 0 && gps_spd_ > 3.0)
                << "dmileage_offset_ratio_: " << dmileage_offset_ratio_
                << " traj_dmileage_: " << traj_dmileage_
                << " vel_dmileage_: " << vel_dmileage_;

            if ((dmileage_offset_ratio_ < kDmileageOffsetRatioThreshold || std::fabs(dmileage_traj_vel_) < kDmileageThresholdMeters) //
                && std::fabs(dmileage_traj_vel_) < kDmileageMaxThresholdMeters) {
                gnss_pos_ready_count_++;
            } else {
                gnss_pos_ready_count_ = 0;
            }
        }
    }
    pre_gnss_data_ = *gps_data_ptr;
    if (!pre_gnss_data_ready_) {
        pre_gnss_data_ready_ = true;
        // 在前一帧未赋值的时候，认为不可靠
        AINFO << "gnss skip first frame";
        return false;
    }

    if (imu_buffer_.size() < kImuDataBufferLength) {
        AINFO_EVERY(10) << "[AddGnssData]: No enought imu data!";
        return false;
    }

    const ImuDataPtr last_imu_ptr = imu_buffer_.back();

    // 依据GNSS与IMU时间差，判断是否同步
    // 因为INS还未初始化，此处无法进行GNSS与INS的时间同步。
    // 但如果GNSS比IMU延迟大于1s，这种情况，GNSS的结果或者融合定位的链路应该有比较大问题，报错
    double dt_gnss_imu = std::abs(gps_data_ptr->measurement_timestamp - last_imu_ptr->measurement_timestamp);
    if (dt_gnss_imu > 1.0) {
        AERROR << "[AddGnssData]: Gps and imu timestamps are not synchronized! dt " << dt_gnss_imu;
        return false;
    }

    // 单天线GNSS在足够的时速下，航向才具备参考性
    // 首次初始化必须要判断速度
    // 如果不是首次初始化，则看一下是否需要重置航向，如果需要重置航向，则要判断速度
    // 这里优化一下速度的要求，如果固定解情况下，则放宽速度限制，如果非固定，则严格限制速度
    bool bad_vel_ =
        (gps_data_ptr->status == 6 && gps_data_ptr->vel.norm() < parameters_sgt.get_gnss_minimum_speed_required_for_initialization() / 1.6) // 固定解放宽速度限制
        || (gps_data_ptr->status != 6 && gps_data_ptr->vel.norm() < parameters_sgt.get_gnss_minimum_speed_required_for_initialization());   // 非固定则使用原始速度限制

    if (                                                                                                                                                         //
        (process_control_sgt.initialization_count == 0 || (process_control_sgt.initialization_count != 0 && process_control_sgt.reinitialization_state.heading)) //
        && bad_vel_                                                                                                                                              //
    ) {
        AINFO_EVERY(30) << "Speed too low to init: " << gps_data_ptr->vel.norm();
        return false;
    }

    // 为保证航向的可靠性，这里对初始化的角速度做一个限制
    if (process_control_sgt.initialization_count == 0 && state_ptr->gyro.norm() > 10.0 / 180.0 * M_PI) {
        AINFO_EVERY(30) << "angle velocity too big to init: " << state_ptr->gyro.norm() * 180.0 / M_PI;
        return false;
    }

    {
        // 这里添加一条记录，表明运动状态具备初始化条件
        static auto dynamic_state_ready_call_once_ = [&last_imu_ptr] {
            // 执行一次的逻辑
            AINFO << "[startup] Dynamic ready (gnss)! imu time: " << fmt::format("{:14.4f}", last_imu_ptr->measurement_timestamp);
            return 0;
        }(); // 定义后立即执行
        (void)dynamic_state_ready_call_once_;
    }

    // 这里进行速度与航向一致性的一个判断
    // 同时考虑轨迹追踪的航向
    {
        if (gps_data_ptr->vel.block<2, 1>(0, 0).norm() > 1e-2) {
            double vel_hdg_        = std::atan2(gps_data_ptr->vel.x(), gps_data_ptr->vel.y());
            double dhdg_vel_gnss_  = vel_hdg_ - gps_data_ptr->hdg;
            double dhdg_traj_gnss_ = traj_az_ - gps_data_ptr->hdg;

            // 航向差异的绝对值可能超过 PI ，可能处于 -2PI ~ 2PI
            // 将航向差异的绝对值控制在 -PI ~ PI 之间
            dhdg_vel_gnss_ = dhdg_vel_gnss_ > M_PI ? dhdg_vel_gnss_ - 2 * M_PI : dhdg_vel_gnss_;
            dhdg_vel_gnss_ = dhdg_vel_gnss_ < -M_PI ? dhdg_vel_gnss_ + 2 * M_PI : dhdg_vel_gnss_;

            dhdg_traj_gnss_ = dhdg_traj_gnss_ > M_PI ? dhdg_traj_gnss_ - 2 * M_PI : dhdg_traj_gnss_;
            dhdg_traj_gnss_ = dhdg_traj_gnss_ < -M_PI ? dhdg_traj_gnss_ + 2 * M_PI : dhdg_traj_gnss_;

            if (std::fabs(dhdg_vel_gnss_) > kHeadingDiffThresholdRad || std::fabs(dhdg_traj_gnss_) > kHeadingDiffThresholdRad) {
                // 如果航向不满足的话，计数置零
                gnss_hdg_ready_count_ = 0;
                AINFO << "heading not valid, vel | hdg | trj (deg): "
                      << vel_hdg_ * 180.0 / M_PI << " | "
                      << gps_data_ptr->hdg * 180.0 / M_PI << " | "
                      << traj_az_ * 180.0 / M_PI
                      << " hdg ready count: "
                      << gnss_hdg_ready_count_;
            } else {
                gnss_hdg_ready_count_++;
            }
        }
        if (gnss_hdg_ready_count_ < kGnssReadyCountThreshold || gnss_pos_ready_count_ < kGnssReadyCountThreshold) {
            AINFO << "gnss ready count not enough: <hdg | pos> " << gnss_hdg_ready_count_ << " | " << gnss_pos_ready_count_;
            // 如果航向满足条件的计数小于一定值，则认为不符合条件
            return false;
        }
    }

    // 使用重力计算横滚、俯仰
    // 使用GNSS航向设置INS的航向
    Eigen::Quaterniond attitude_ = Eigen::Quaterniond::Identity();

    if (process_control_sgt.initialization_count == 0) {
        // 只有首次初始化的时候，尝试使用加计计算俯仰和横滚角
        // 因为融合定位可以保证横滚和俯仰稳定，后续再初始化的时候，使用历史状态
        if (!EstimateAttitudeFromImuData(attitude_)) {
            AWARN << "[AddGnssData]: Failed to compute G_R_I!";
            return false;
        }
        state_ptr->gyro_bias = gyro_bias_init_estimate_;
    } else {
        attitude_ = INS::euler2quaternion({state_ptr->eulr_.x(), state_ptr->eulr_.y(), 0.0});
    }

    Eigen::Quaterniond hdg0_q = Eigen::Quaterniond::Identity();
    hdg0_q                    = Eigen::AngleAxisd(gps_data_ptr->hdg, Eigen::Vector3d::UnitZ()).toRotationMatrix().transpose();

    INS::EARTH earth(gps_data_ptr->lla, gps_data_ptr->vel);

    // 设置融合定位的状态，粗对准
    if (process_control_sgt.reinitialization_state.heading) {
        AINFO << "[initialization] reset attitude";
        state_ptr->att = hdg0_q * attitude_ * state_ptr->q_imu2vehicle;
        AINFO << "att (deg): " << INS::quaternion2euler(state_ptr->att).transpose() * 180.0 / M_PI;
    }
    if (process_control_sgt.reinitialization_state.velocity) {
        AINFO << "[initialization] reset velocity";
        state_ptr->vel = gps_data_ptr->vel;
        AINFO << "vel (m/s): " << state_ptr->vel.transpose();
    }

    if (process_control_sgt.reinitialization_state.position) {
        AINFO << "[initialization] reset position";
        if (process_control_sgt.initialization_count == 0) {
            // 这里在第一次初始化的时候，使用GNSS位姿获得的初值，将杆臂转换到绝对坐标系下。
            // 由此避免第一次设置初值的时候，位置有较大偏差的问题。
            Eigen::Matrix3d Mpv_;
            Mpv_ << 0.0, 1.0 / earth.RMh, 0.0, 1.0 / earth.clRNh, 0.0, 0.0, 0.0, 0.0, 1.0;
            Eigen::Vector3d lever_ = -(Mpv_ * state_ptr->att.toRotationMatrix() * state_ptr->lever_imu2gnss);
            state_ptr->lla         = gps_data_ptr->lla + lever_;
        } else {
            Eigen::Vector3d lever_ = -(state_ptr->MpvCnb * state_ptr->q_imu2vehicle.toRotationMatrix() * state_ptr->lever_imu2gnss);
            AINFO << "reset lever: " << lever_.transpose();
            if (process_control_sgt.fusion_status.imu_update_count_after_vision_fusion_update < 100) {
                // 在有视觉融合的时候
                // 这里在重置位置的时候，同时尝试消去制图误差
                state_ptr->mapBiasAblationWhenReset(gps_data_ptr->lla + lever_);
            }
            state_ptr->lla = gps_data_ptr->lla + lever_;
            if (!process_control_sgt.reinitialization_state.heading && !process_control_sgt.reinitialization_state.velocity) {
                // 如果系统重置了航向或速度，则这个标志位不清零
                process_control_sgt.fusion_status.imu_update_count_aftr_pos_reset = 0;
            }
        }

        if (delta_ptr) {
            // 对初始化阶段的位置进行补偿。。
            Eigen::Matrix3d R_rfu2enu        = state_ptr->att.toRotationMatrix();
            Eigen::Vector3d dpos             = R_rfu2enu * INS::frame_trans.FLU2RFU * delta_ptr->pos;
            auto            lla              = geotrans.enu2blh({state_ptr->lla.x() * 180.0 / M_PI, state_ptr->lla.y() * 180.0 / M_PI, state_ptr->lla.z()}, {dpos.x(), dpos.y(), dpos.z()});
            state_ptr->lla                   = Eigen::Vector3d{lla.b / 180.0 * M_PI, lla.l / 180.0 * M_PI, lla.h};
            state_ptr->measurement_timestamp = state_ptr->measurement_timestamp + delta_ptr->measurement_timestamp;
        }

        state_ptr->pos_reset_timestamp = state_ptr->measurement_timestamp;
        AINFO << "pos: " << state_ptr->lla.x() * 180.0 / M_PI << ", " << state_ptr->lla.y() * 180.0 / M_PI << ", " << state_ptr->lla.z();
    }
    AINFO << "[initialization] timestamp: " << std::setprecision(14) << gps_data_ptr->measurement_timestamp;
    { // 重置滤波器粗对准AVP状态
        auto            init_P_ = parameters_sgt.get_init_P();
        Eigen::Vector3d att_std{init_P_(0), init_P_(1), init_P_(2)};
        Eigen::Vector3d vel_std{init_P_(3), init_P_(4), init_P_(5)};
        Eigen::Vector3d pos_std{init_P_(6) / earth.RMh, init_P_(7) / earth.clRNh, init_P_(8)};
        kf_ptr->P.block<9, 9>(0, 0).setZero();
        Eigen::Matrix<bool, 21, 1> idx;
        idx << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0;
        CovarianceUpdateByIdx(kf_ptr->P, 0.0, idx, true);

        kf_ptr->P.block<3, 3>(0, 0) = att_std.cwiseAbs2().asDiagonal();
        kf_ptr->P.block<3, 3>(3, 3) = vel_std.cwiseAbs2().asDiagonal();
        kf_ptr->P.block<3, 3>(6, 6) = pos_std.cwiseAbs2().asDiagonal();

        // 这几个状态是缓变的，重新初始化的时候，不需要重置协方差。
        // Eigen::Vector3d gyro_bias_std{init_P_(9), init_P_(10), init_P_(11)};
        // Eigen::Vector3d acc_bias_std{init_P_(12), init_P_(13), init_P_(14)};
        // Eigen::Vector3d vehicle_error_std{init_P_(15), init_P_(16), init_P_(17)};
        // kf_ptr->P.block<3, 3>(9, 9)   = gyro_bias_std.cwiseAbs2().asDiagonal();
        // kf_ptr->P.block<3, 3>(12, 12) = acc_bias_std.cwiseAbs2().asDiagonal();
        // kf_ptr->P.block<3, 3>(15, 15) = vehicle_error_std.cwiseAbs2().asDiagonal();

        Eigen::Vector3d map_error_std{init_P_(18), init_P_(19), init_P_(20)};
        kf_ptr->P.block<3, 3>(18, 18) = map_error_std.cwiseAbs2().asDiagonal();
    }

    // 设置过程控制标志位到 粗对准
    process_control_sgt.msf_align_type                  = process_control_sgt.COARSE_ALIGN;
    process_control_sgt.reinitialization_state.heading  = false;
    process_control_sgt.reinitialization_state.velocity = false;
    process_control_sgt.reinitialization_state.position = false;

    gnss_hdg_ready_count_ = 0;
    gnss_pos_ready_count_ = 0;

    // 这里添加一条记录，表明具备初始化条件，进入初始化逻辑
    static auto tcmsf_init_call_once_ = [&last_imu_ptr] {
        // 执行一次的逻辑
        AINFO << "[startup] TCMSF Start Init! imu time: " << fmt::format("{:14.4f}", last_imu_ptr->measurement_timestamp);
        return 0;
    }(); // 定义后立即执行
    (void)tcmsf_init_call_once_;

    process_control_sgt.initialization_count++;
    AINFO << "initialization_count: " << process_control_sgt.initialization_count;
    return true;
}

bool Initializer::EstimateAttitudeFromImuData(Eigen::Quaterniond &attitude_) {
    // 采用加计测量，粗略计算横滚和俯仰
    // TODO：对于车辆，横滚和俯仰都是小量，这种方式应该就够用了。

    // Compute mean and std of the imu buffer.
    Eigen::Vector3d sum_acc{0., 0., 0.};
    for (const auto imu_data : imu_buffer_) {
        sum_acc += imu_data->acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();

    Eigen::Vector3d sum_err2{0., 0., 0.};
    for (const auto imu_data : imu_buffer_) {
        sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    }
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();

    if (std_acc.maxCoeff() > kAccStdLimit) {
        AWARN << "[Compute pitch & roll FromImuData]: Too big acc std: " << std_acc.transpose();
        return false;
    }
    if (mean_acc.norm() < 5.0 || mean_acc.norm() > 15.0) {
        AWARN << "abnormal acc measurement: " << mean_acc.transpose();
        return false;
    }

    double roll_  = std::atan2(mean_acc.x(), std::sqrt(mean_acc.y() * mean_acc.y() + mean_acc.z() * mean_acc.z()));
    double pitch_ = std::atan2(mean_acc.y(), std::sqrt(mean_acc.x() * mean_acc.x() + mean_acc.z() * mean_acc.z()));

    Eigen::Matrix3d att_xy = Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitX()).toRotationMatrix();
    attitude_              = att_xy;
    return true;
}

void AttReference::AttRef(const ImuData &cur_imu, const VehicleData &cur_veh, double veh_dt, StatePtr state_ptr, const KinematicDataPtr inner_motion_state_ptr) {
    if (std::abs(veh_dt) > 1e-10 && std::abs(cur_imu.measurement_timestamp - cur_veh.measurement_timestamp) < 0.1) { // 重力参考
        Eigen::Vector3d acc_       = cur_imu.acc - state_ptr->acc_bias;
        Eigen::Vector3d gyro_      = cur_imu.gyro - state_ptr->gyro_bias;
        double          acc_lon_sm = as_lon(acc_.y());
        double          acc_lat_sm = as_lat(acc_.x());

        auto lpfout = vlpf({gyro_.x(), gyro_.y(), gyro_.z()});

        if (step_count % VDSA_SKIP == 0) {
            auto EPS     = std::numeric_limits<double>::epsilon();
            auto g_      = state_ptr->gravity.norm() + EPS;
            auto result_ = vdsa({0.0, (cur_veh.speed_rl + cur_veh.speed_rr) / 2.0, 0.0}, {lpfout[0], lpfout[1], lpfout[2]}, veh_dt * VDSA_SKIP, acc_lon_sm / g_, acc_lat_sm / g_);
            // auto result_ = vdsa(state_ptr->vel, {lpfout[0], lpfout[1], lpfout[2]}, veh_dt * VDSA_SKIP, acc_lon_sm / g_, acc_lat_sm / g_);

            Eigen::Matrix3d C_n2b = state_ptr->C_b2n.transpose();
            Eigen::Vector3d acc_b = acc_;
            if (result_.lat_ac && result_.lon_ac) {
                acc_b.x() = acc_b.x() - vdsa.latitudinal_acc;
                acc_b.y() = acc_b.y() - vdsa.longitudinal_acc;
            }
            acc_b.normalize();
            Eigen::Vector3d C3 = C_n2b.col(2);

            Eigen::Matrix3d C_n2b_dr = inner_motion_state_ptr->att.toRotationMatrix().transpose();
            Eigen::Vector3d C3_dr    = C_n2b_dr.col(2);

            if (result_.alp_a) {
                // AINFO << "alpha_factor: " << vdsa_ptr->alpha_factor;
                state_ptr->phi_b_gravity    = 0.1 * vdsa.alpha_factor * acc_b.cross(C3);
                state_ptr->dr_phi_b_gravity = 0.1 * vdsa.alpha_factor * acc_b.cross(C3_dr);
            }
        }
        step_count++;
    } else {
        state_ptr->phi_b_gravity = {0.0, 0.0, 0.0};
    }
}

void ImuProcessor::Predict(const ImuDataPtr cur_imu, double dt_, StatePtr state_ptr, KfPtr<21, 3> kf_ptr) {

#if (defined __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES) && (defined __TCMSF_ENABLE_RTS_SMOOTHER_)
    static auto rts_ptr_       = RTS::RtsSmoother::create();
    static auto rts_step_info_ = RTS::ForwardStepInfo();

    auto get_rts_state_func = [](RTS::Nx1_RTS_D &rts_X_, StatePtr state_ptr) { // 获取并填入滤波器状态
        Eigen::Matrix<double, 18, 1> filter_state_;
        Eigen::Vector3d              rts_eulr_ = INS::quaternion2euler(state_ptr->q_imu2vehicle * state_ptr->att.conjugate());
        filter_state_ << rts_eulr_.x(), rts_eulr_.y(), rts_eulr_.z(),                              //
            state_ptr->vel.x(), state_ptr->vel.y(), state_ptr->vel.z(),                            //
            state_ptr->lla.x(), state_ptr->lla.y(), state_ptr->lla.z(),                            //
            state_ptr->gyro_bias.x(), state_ptr->gyro_bias.y(), state_ptr->gyro_bias.z(),          //
            state_ptr->acc_bias.x(), state_ptr->acc_bias.y(), state_ptr->acc_bias.z(),             //
            state_ptr->vehicle_bias.x(), state_ptr->vehicle_bias.y(), state_ptr->vehicle_bias.z(); //
        rts_X_ = filter_state_.block<RTS::RTS_D, 1>(0, 0);

    };

    RTS::Nx1_RTS_D rts_Xk_;
    get_rts_state_func(rts_Xk_, state_ptr);
    rts_step_info_.Xk  = rts_Xk_;
    rts_step_info_.Pk  = kf_ptr->P.block<RTS::RTS_D, RTS::RTS_D>(0, 0);
    rts_step_info_.sow = rts_step_info_.timestamp - process_control_sgt.last_valid_dt;
    if (rts_step_info_.Xkk1.block<3, 1>(6, 0).norm() > 1e-10) {
        rts_ptr_->insert(rts_step_info_);
    }

#endif

    imu_msg_count_++;

    // 更新侧滑系数
    state_ptr->slip_index = std::abs(state_ptr->gyro.z() * state_ptr->vel.norm());

    // 预测更新

    Eigen::Matrix<double, 1, 6> imudata_;
    imudata_.block<1, 3>(0, 0) = cur_imu->gyro;
    imudata_.block<1, 3>(0, 3) = cur_imu->acc;

    // 姿态稳定算法：使用加速度计算参考姿态，使用参考姿态稳定融合定位的水平姿态
    // 姿态稳定算法有害于精细的姿态计算，这里需要较为严格的准入条件
    // 零速状态、初始化状态、10分钟没有GNSS量测，再使用姿态稳定算法
    if (parameters_sgt.get_attitude_stablization_by_gravity() ||                                  //
        (process_control_sgt.vehicle_motion_type == process_control_sgt.ZERO_VELOCITY ||          //
         process_control_sgt.fusion_status.imu_update_count_after_gnss_update >= 10 * 60 * 100 || //
         process_control_sgt.msf_align_type == process_control_sgt.INITIALIZATION)                //
    ) {
        AINFO_EVERY(300) << "INS update with Gravity Att Ref";
        // attitude stablization
        sins_ptr->update(imudata_, state_ptr->phi_b_gravity, dt_, state_ptr->ctl);
        if (state_ptr->phi_b_gravity.norm() > 1e-8) {
            // 使用重力参考约束姿态的时候，自车横滚和俯仰姿态是稳定的，这里做一个协方差的更新，以免姿态误差协方差病态发散
            // 这里重力参考姿态也可以做成完整的滤波形式，但是此处沿用了DR里面的代码（AttRef函数内），懒得调了
            // TODO：做成完整的滤波形式
            Eigen::Matrix<double, 3, 21> H;
            Eigen::Vector3d              innovation;
            Eigen::Matrix3d              V;

            H.setZero();
            H.block<3, 3>(0, 0) = Eigen::Vector3d{1.0, 1.0, 0.0}.asDiagonal();
            innovation.setZero();

            Eigen::Vector3d g_ref_std{10.0 / 180.0 * M_PI, 10.0 / 180.0 * M_PI, 1.0e7};
            V = g_ref_std.cwiseAbs2().asDiagonal();

            kf_ptr->kf_update(H, V, innovation);

            kf_ptr->P = kf_ptr->cov;

            // Eigen::Matrix<bool, 21, 1> idx;
            // idx << 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
            // CovarianceUpdateByIdx(kf_ptr->P, kf_ptr->cov, idx, true);
        }
    } else {
        sins_ptr->update(imudata_, {0.0, 0.0, 0.0}, dt_, state_ptr->ctl);
    }

    KF<21, 3>::MxM Tk = KF<21, 3>::MxM::Identity();

    if (dt_ > 0.05) {
        Tk.block<15, 15>(0, 0) = (sins_ptr->Ft * dt_).exp();
    } else {
        Tk.block<15, 15>(0, 0) = INS::Matrix15d::Identity() + (sins_ptr->Ft * dt_);
    }

    KF<21, 3>::MxM Qk = KF<21, 3>::MxM::Zero();

    { // set Q by imu's parameters
        Eigen::Vector3d gyro_rw{dt_ * dt_ * gyro_rw_ * gyro_rw_, dt_ * dt_ * gyro_rw_ * gyro_rw_, dt_ * dt_ * gyro_rw_ * gyro_rw_};
        Eigen::Vector3d gyro_nd{dt_ * dt_ * gyro_ND_ * gyro_ND_, dt_ * dt_ * gyro_ND_ * gyro_ND_, dt_ * dt_ * gyro_ND_ * gyro_ND_};
        Eigen::Vector3d acc_rw{dt_ * dt_ * acc_rw_ * acc_rw_, dt_ * dt_ * acc_rw_ * acc_rw_, dt_ * dt_ * acc_rw_ * acc_rw_};
        Eigen::Vector3d acc_nd{dt_ * dt_ * acc_ND_ * acc_ND_, dt_ * dt_ * acc_ND_ * acc_ND_, dt_ * dt_ * acc_ND_ * acc_ND_};

        Qk.block<3, 3>(0, 0) = gyro_nd.asDiagonal();
        Qk.block<3, 3>(3, 3) = acc_nd.asDiagonal();

        Qk.block<3, 3>(9, 9)   = gyro_rw.asDiagonal();
        Qk.block<3, 3>(12, 12) = acc_rw.asDiagonal();
    }

    { // set Q by wheel's parameters
        Eigen::Vector3d wheel_cov_{dt_ * dt_ * wheel_pitch_rw * wheel_pitch_rw, dt_ * dt_ * wheel_spd_scale_rw * wheel_spd_scale_rw, dt_ * dt_ * wheel_yaw_rw * wheel_yaw_rw};
        Qk.block<3, 3>(15, 15) = wheel_cov_.asDiagonal();
    }

    if (process_control_sgt.fusion_status.imu_update_count_after_vision_fusion_update < 300) {
        // set Q by map's parameters
        Eigen::Vector3d map_cov_{dt_ * dt_ * map_pos_east_bias_rw * map_pos_east_bias_rw, dt_ * dt_ * map_pos_north_bias_rw * map_pos_north_bias_rw, dt_ * dt_ * map_heading_bias_rw * map_heading_bias_rw};
        Qk.block<3, 3>(18, 18) = map_cov_.asDiagonal();
    } else {
        Qk.block<3, 3>(18, 18) = Eigen::Matrix3d::Zero();
    }

    kf_ptr->kf_update(Tk, Qk);
    kf_ptr->P = kf_ptr->cov;

    { // set state by time update
        state_ptr->measurement_timestamp = cur_imu->measurement_timestamp;

        state_ptr->att       = sins_ptr->avp.att;
        state_ptr->vel       = sins_ptr->avp.vel;
        state_ptr->lla       = sins_ptr->avp.pos;
        state_ptr->gyro_bias = sins_ptr->eb;
        state_ptr->acc_bias  = sins_ptr->db;

        state_ptr->web    = sins_ptr->web;
        state_ptr->MpvCnb = sins_ptr->MpvCnb;
        state_ptr->Mpv    = sins_ptr->Mpv;
        state_ptr->C_b2n  = sins_ptr->C_b2n;

        state_ptr->gravity = state_ptr->C_b2n.transpose() * sins_ptr->earth.g;

        if (std::abs(dt_) > 1e-5) {
            state_ptr->acc  = cur_imu->acc / dt_ + state_ptr->gravity;
            state_ptr->gyro = cur_imu->gyro / dt_;
            // std::cout << "acc: " << state_ptr->acc.transpose() << std::endl;
        } else {
            state_ptr->acc  = {0.0, 0.0, 0.0};
            state_ptr->gyro = {0.0, 0.0, 0.0};
        }

        state_ptr->eulr_ = INS::quaternion2euler(state_ptr->att);

        state_ptr->error_state_std    = kf_ptr->P.diagonal().cwiseSqrt();
        state_ptr->error_state_std[6] = sins_ptr->earth.RMh * state_ptr->error_state_std[6];
        state_ptr->error_state_std[7] = sins_ptr->earth.clRNh * state_ptr->error_state_std[7];
    }

#if (defined __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES) && (defined __TCMSF_ENABLE_RTS_SMOOTHER_)

    rts_step_info_.timestamp = cur_imu->measurement_timestamp;
    rts_step_info_.Tkk1      = Tk.block<RTS::RTS_D, RTS::RTS_D>(0, 0);
    rts_step_info_.Pkk1      = kf_ptr->P.block<RTS::RTS_D, RTS::RTS_D>(0, 0);
    RTS::Nx1_RTS_D rts_Xkk1_;
    get_rts_state_func(rts_Xkk1_, state_ptr);
    rts_step_info_.Xkk1  = rts_Xkk1_;
    rts_step_info_.zupt  = process_control_sgt.vehicle_motion_type == process_control_sgt.ZERO_VELOCITY;
    rts_step_info_.m_sta = (RTS::ManeuverStatus)process_control_sgt.maneuver_status_by_imu;
#endif

    // 以下是IMU更新驱动的系统状态、标志位更新

    {
        if (process_control_sgt.vehicle_motion_type == process_control_sgt.MOVING) {
            if (process_control_sgt.fusion_status.continous_zupt_imu_bias_estimate_count > 60) {
                // 如果零速更新过程，进行了60次以上的IMU零偏估计
                // 那么我们认为这次零速更新成功的估计了IMU零偏，计数++
                state_ptr->zupt_imu_bias_estimate_ok_count++;
                AINFO << "ZUPT IMU bias estimate ok count: " << state_ptr->zupt_imu_bias_estimate_ok_count
                      << "\n[imu bias info zupt]:"
                      << " bg(deg/s): " << state_ptr->gyro_bias.transpose() * 180.0 / M_PI
                      << " ba(m/s^2): " << state_ptr->acc_bias.transpose();
            }
            process_control_sgt.fusion_status.continous_zupt_imu_bias_estimate_count = 0;
        }
    }

    process_control_sgt.fusion_status.imu_update_count_aftr_pos_reset++;

    process_control_sgt.fusion_status.imu_update_count_after_gnss_update++;
    process_control_sgt.fusion_status.imu_update_count_after_rtk_fix++;
    process_control_sgt.fusion_status.imu_update_count_after_vision_fusion_update++;
    // IMU更新计数，考虑到GNSS帧率不稳定，此处放开到若超过100次IMU更新无GNSS更新，则认为GNSS未能维持连续更新
    if (process_control_sgt.fusion_status.imu_update_count_after_gnss_update > 100) {
        process_control_sgt.fusion_status.continous_gnss_fusion_count    = 0;
        process_control_sgt.fusion_status.continous_gnss_fix_count       = 0;
        process_control_sgt.fusion_status.continous_gnss_float_fix_count = 0;
    }

    if (std::abs(cur_imu->measurement_timestamp - state_ptr->gnss_inno_(0)) > 1.0) {
        state_ptr->rtk_status = 0;
    }

    if (((process_control_sgt.fusion_status.imu_update_count_after_gnss_update > 500 || process_control_sgt.fusion_status.imu_update_count_after_rtk_fix > 500) && //
         process_control_sgt.msf_align_type == process_control_sgt.ALIGNED)) {

        // 如果长时间未能融合卫星信息，则需要重新进行精对准，置位 精对准 控制位
        process_control_sgt.msf_align_type = process_control_sgt.FINE_ALIGN;
    }

    //
    static double mileage_with_gnss_ = 0.0;
    // 只要1秒钟之内有卫星量测更新并持续更新了5帧，且新息距离在0.5米内，即认为处于有卫星量测状态
    if (process_control_sgt.fusion_status.imu_update_count_after_gnss_update < 100 && //
        process_control_sgt.fusion_status.continous_gnss_fusion_count > 5 &&          //
        std::fabs(state_ptr->gnss_inno_(0) - cur_imu->measurement_timestamp) < 1.0 && //
        state_ptr->gnss_inno_.block<2, 1>(1, 0).norm() < 0.5) {
        mileage_with_gnss_ = state_ptr->mileage;
    }
    bool long_range_invalid_ =
        std::fabs(state_ptr->mileage - mileage_with_gnss_) > 1500.0 ||                    // 1.5公里无卫星认为递推精度无法维持
        process_control_sgt.fusion_status.imu_update_count_after_gnss_update > 100 * 100; // 100秒无卫星认为递推精度无法维持

    //

    if (long_range_invalid_ && //
        (process_control_sgt.msf_align_type == process_control_sgt.ALIGNED || process_control_sgt.msf_align_type == process_control_sgt.FINE_ALIGN)) {
        // 如果长时间未能融合卫星信息，则需要重新进行对准，置位 对准 控制位
        AINFO << "GNSS long range invalid, MSF align status turn to coarse align.";
        process_control_sgt.msf_align_type = process_control_sgt.COARSE_ALIGN;
    }

    { // 依据加速度判断车辆状态

        // 这里给IMU的零偏做个补偿
        auto acc_  = state_ptr->acc - state_ptr->acc_bias;
        auto gyro_ = state_ptr->gyro - state_ptr->gyro_bias;
        // 低通
        auto            lpf_rslt_ = vlpf_imu_({acc_.x(), acc_.y(), acc_.z(), gyro_.x(), gyro_.y(), gyro_.z()});
        Eigen::Vector3d acc_lp_{lpf_rslt_[0], lpf_rslt_[1], lpf_rslt_[2]};
        Eigen::Vector3d gyro_lp_{lpf_rslt_[3], lpf_rslt_[4], lpf_rslt_[5]};

        state_ptr->acc_lp_  = acc_lp_;
        state_ptr->gyro_lp_ = gyro_lp_;

        double horizontal_acc_   = acc_lp_.block<2, 1>(0, 0).norm();
        double angular_velocity_ = gyro_lp_.norm();

        // 这里考虑零速更新的次数，如果没有进行零速更新，那么陀螺零偏估计的其实应该不太好。
        // 这种情况下，放宽动态性的限制。
        // 如果进行过零速更新，那么零偏大概率估计的很好了，这里就可以对动态性做一下更严格的限制。
        double maneuver_factor_about_zupt_ = 3.0;
        if (state_ptr->zupt_imu_bias_estimate_ok_count >= 1) {
            maneuver_factor_about_zupt_ = 1.0;
        }

        if (horizontal_acc_ < parameters_sgt.get_maneuver_status_low_dynamic_acc_bound() * maneuver_factor_about_zupt_) {
            double maneuver_ratio_                            = horizontal_acc_ / (std::abs(parameters_sgt.get_maneuver_status_low_dynamic_acc_bound() * maneuver_factor_about_zupt_) + 1e-10);
            process_control_sgt.maneuver_status_by_imu_scale_ = 1.0 - maneuver_ratio_;
            if (process_control_sgt.maneuver_status_by_imu_scale_ < 0.1) {
                process_control_sgt.maneuver_status_by_imu_scale_ = 0.1;
            }
        } else {
            process_control_sgt.maneuver_status_by_imu_scale_ = 1.0;
        }

        if (horizontal_acc_ < parameters_sgt.get_maneuver_status_steady_acc_bound() &&   //
            angular_velocity_ < parameters_sgt.get_maneuver_status_steady_rotate_bound() //
        ) {
            process_control_sgt.maneuver_status_by_imu = process_control_sgt.IMU_STEADY;
        } else if (horizontal_acc_ < parameters_sgt.get_maneuver_status_low_dynamic_acc_bound() &&   //
                   angular_velocity_ < parameters_sgt.get_maneuver_status_low_dynamic_rotate_bound() //
        ) {
            process_control_sgt.maneuver_status_by_imu = process_control_sgt.IMU_DYNAMIC_LOW;
        } else {
            process_control_sgt.maneuver_status_by_imu = process_control_sgt.IMU_DYNAMIC_HIGH;
        }
    }
    // state_ptr->imu_update_count_after_gnss_update.operator++();
    // if (state_ptr->imu_update_count_after_gnss_update.load() > 20) {
    //     state_ptr->gnss_update_count_after_gnss_deny.store(0);
    //     state_ptr->rtk_status = 0;
    // }

    state_ptr->imu_dt = dt_;

    state_ptr->vel_ego = state_ptr->GetEgoVel();

    // {
    //     if (MSF::process_control_sgt.msf_align_type == MSF::process_control_sgt.ALIGNED || MSF::process_control_sgt.msf_align_type == MSF::process_control_sgt.FINE_ALIGN) {
    //         double          ego_lat_vel_constrain = parameters_sgt.get_ego_lat_velocity_constrain();
    //         Eigen::Matrix3d C_veh2ref             = (state_ptr->att * (INS::euler2quaternion({state_ptr->vehicle_bias.x(), 0.0, state_ptr->vehicle_bias.z()})).conjugate()).toRotationMatrix();
    //         Eigen::Matrix3d C_ref2veh             = C_veh2ref.transpose();
    //         Eigen::Vector3d vel_ego               = C_ref2veh * state_ptr->vel;

    //         byd::geo::LocalTrans::constrain_if_in_bound(vel_ego.x(), 1e10, ego_lat_vel_constrain);

    //         Eigen::Vector3d vel_lat_constrained = C_veh2ref * vel_ego;

    //         state_ptr->vel = vel_lat_constrained;
    //     }
    // }

    {

        // 非长时间无卫星量测情况下
        // 如果位置新息的距离加权小于一定值
        // 则不更新粗对准状态
        bool coarse_align_omit_ =
            !long_range_invalid_ &&
            state_ptr->pos_rfu_inno_range_norm_ < 10.0;

        switch (process_control_sgt.msf_align_type) {
            case process_control_sgt.INITIALIZATION: {
                state_ptr->align_type = state_ptr->UNALIGNED;
                break;
            }
            case process_control_sgt.COARSE_ALIGN: {
                if (coarse_align_omit_) {
                    if (state_ptr->align_type == state_ptr->UNALIGNED) {
                        // 如果原对准状态为非对准，则升档对准状态到粗对准
                        state_ptr->align_type = state_ptr->COARSE_ALIGN;
                    }
                    if (parameters_sgt.get_gnss_fusion_mode() == Parameters::GnssFusionMode::GNSS_LOOSE_COUPLE) {
                        // 如果在单点融合模式下，进入粗对准，则升档到精对准
                        state_ptr->align_type = state_ptr->FINE_ALIGN;
                    }
                } else {
                    state_ptr->align_type = state_ptr->COARSE_ALIGN;
                }
                break;
            }
            case process_control_sgt.FINE_ALIGN: {
                state_ptr->align_type = state_ptr->FINE_ALIGN;
                break;
            }
            case process_control_sgt.ALIGNED: {
                state_ptr->align_type = state_ptr->ALIGNED;
                break;
            }
            default: {
                state_ptr->align_type = state_ptr->UNALIGNED;
                break;
            }
        }
        if (process_control_sgt.msf_align_type != process_control_sgt.INITIALIZATION) {
            if (process_control_sgt.fusion_status.imu_update_count_after_gnss_update > 20) {
                if (process_control_sgt.fusion_status.imu_update_count_after_vision_fusion_update > 20) {
                    state_ptr->fusion_status = state_ptr->DRMODE;
                } else {
                    state_ptr->fusion_status = state_ptr->VFMODE;
                }
            } else {
                if (process_control_sgt.fusion_status.imu_update_count_after_vision_fusion_update <= 20) {
                    state_ptr->fusion_status = state_ptr->FULLSTATE;
                } else {
                    state_ptr->fusion_status = state_ptr->GPSONLY;
                }
            }
        } else {
            state_ptr->fusion_status = state_ptr->INIT;
        }
    }

    {

        // 正定性
        Eigen::Matrix<double, 21, 21> P_ = kf_ptr->P.eval();
        kf_ptr->P                        = (P_ + P_.transpose()) / 2.0;

        // 限制P阵范围
        Eigen::Matrix<double, 21, 1> constrain_P_std_min_ = constrain_P_std_min;
        Eigen::Matrix<double, 21, 1> constrain_P_std_max_ = constrain_P_std_max;
        constrain_P_std_min_(6, 0)                        = constrain_P_std_min(6, 0) / sins_ptr->earth.RMh;
        constrain_P_std_min_(7, 0)                        = constrain_P_std_min(7, 0) / sins_ptr->earth.clRNh;
        constrain_P_std_max_(6, 0)                        = constrain_P_std_max(6, 0) / sins_ptr->earth.RMh;
        constrain_P_std_max_(7, 0)                        = constrain_P_std_max(7, 0) / sins_ptr->earth.clRNh;
        CovarianceConstrain(kf_ptr->P, constrain_P_std_min_, constrain_P_std_max_);
    }

    {
        // if (imu_msg_count_ % 200 == 0) { // 每两秒检测一次正定性
        //     // P矩阵正定性检测
        //     Eigen::LLT<Eigen::Matrix<double, 21, 21>> llt_(kf_ptr->P);
        //     if (llt_.info() != Eigen::Success) {
        //         AERROR << "P mat is not positive definite!";
        //     }
        // }

        // P矩阵NaN值检测，如果检测到NaN，打印消息，并且对异常值进行处理
        // 先根据经纬度调整一下P矩阵中经纬度协方差的初始值
        if (!std::isnan(sins_ptr->earth.RMh) && !std::isnan(sins_ptr->earth.clRNh)) {
            init_P_cov_(6) = init_P_(6) * init_P_(6) / sins_ptr->earth.RMh / sins_ptr->earth.RMh;
            init_P_cov_(7) = init_P_(7) * init_P_(7) / sins_ptr->earth.clRNh / sins_ptr->earth.clRNh;
        } else {
            init_P_cov_(6) = init_P_(6) * init_P_(6) / 6378137.0 / 6378137.0;
            init_P_cov_(7) = init_P_(7) * init_P_(7) / 6378137.0 / 6378137.0;
        }
        for (size_t i = 0; i < kf_ptr->P.rows(); i++) {
            for (size_t j = 0; j < kf_ptr->P.cols(); j++) {
                if (std::isnan(kf_ptr->P(i, j))) {
                    if (i != j) {
                        // 如果是非对角线元素，则直接置零
                        kf_ptr->P(i, j) = 0.0;
                    } else {
                        // 如果是对角线元素，则重置为初始值
                        kf_ptr->P(i, i) = init_P_cov_(i);
                    }
                    AERROR << "P mat, NaN detected! P[" << i << "][" << j << "]";
                }
            }
        }
    }

    {
        // 小区域内制图误差是一定的，但是自车姿态是变化的，所以如果要在自车系下描述制图误差，则需要依据姿态进行变换
        // 把这个放在IMU的更新里面，主要是为了保证平滑
        Eigen::Quaterniond datt_              = pre_att * state_ptr->att.conjugate();
        Eigen::Vector3d    pos_               = {state_ptr->map_bias.x(), state_ptr->map_bias.y(), 0.0};
        state_ptr->map_bias.block<2, 1>(0, 0) = (datt_.toRotationMatrix() * pos_).block<2, 1>(0, 0);
        pre_att                               = state_ptr->att;
    }

    {
        if (state_ptr->vel.norm() > 2.0) {
            // 逐渐消去制图系统性偏差
            // 放在IMU更新里的目的是做个强制的消除策略
            // 大概的周期大于10秒
            // 另外在vision processor里面也额外做了消除策略，消除周期大于10秒
            state_ptr->map_bias = state_ptr->map_bias - state_ptr->map_bias / 5000.0;
        }
    }

    {
        if (std::abs(state_ptr->acc.z()) > 20.0) {
            state_ptr->violent_bump_detected = true;
        }
    }
}

// 将状态复制到SINS中，实现反馈回路
void ImuProcessor::Feedback(const StatePtr state_ptr, bool is_set_att) {
    INS::AVP avp;
    avp.att = state_ptr->att;
    avp.pos = state_ptr->lla;

    avp.vel            = state_ptr->vel;
    Eigen::Vector3d eb = state_ptr->gyro_bias;
    Eigen::Vector3d db = state_ptr->acc_bias;
    sins_ptr->set_state(avp, eb, db, is_set_att);
}

} // namespace MSF