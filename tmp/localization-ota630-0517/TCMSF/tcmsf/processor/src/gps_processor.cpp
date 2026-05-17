#include "gps_processor.h"
#include "calc.h"
#include "cyber/common/log.h"
#include "rigid_transform.h"
#include <iomanip>

#include "fmt/format.h"

#include "Coord.h"
#include "processor_debug.h"

namespace MSF {

bool GpsProcessor::UpdateStateByGps(GnssDataPtr gps_data_ptr, StatePtr state_ptr, KfPtr<21, 3> kf_ptr, const KinematicDataPtr dr_ptr) {
    // 每200个打印一个心跳信息
    AINFO_EVERY(200) << "[HeartBeat] GNSS Fusion, TS: " << fmt::format("{:14.4f}", gps_data_ptr->measurement_timestamp);

    tcmsf_not_init_ = !state_ptr->tcmsf_1st_time_initialized_;

    rtk_cross_validation.insert(gps_data_ptr, state_ptr, dr_ptr);

    double gps_interframe_dt_ = gps_data_ptr->measurement_timestamp - pre_gnss_measurement_timestamp;
    double tcmsf_spd_         = state_ptr->vel.norm();

    // 纵向补偿策略相关标志位计算逻辑
    {
        // 计算RTK无法FIX解之后的里程
        // 如果大于5000.0米，则启用纵向补偿策略
        pos_cmp_.delta_mileage = state_ptr->mileage - pos_cmp_.last_mileage_with_gnss_fix;
        if (pos_cmp_.last_mileage_with_gnss_fix == 0) {
            pos_cmp_.delta_mileage = 0.0;
        }

        if (gps_data_ptr->status == 6 && rsa_.is_rtk_fix_valid_) {
            pos_cmp_.last_mileage_with_gnss_fix = state_ptr->mileage;
        }

        // 在无GNSS状态下，行进5000米之后，认为需要进行纵向的补偿。
        if (std::abs(pos_cmp_.delta_mileage) > 5000.0) {
            pos_cmp_.longi_compe_count_ = 0;
        }

        pos_cmp_.longi_compe_count_++;

        // 高速场景下，如果卫星观测连续维持较好的状态，也启用纵向补偿策略

        // 这个地方，只考虑高速场景
        if (
            (gps_data_ptr->status == parameters_sgt.FIX || gps_data_ptr->status == parameters_sgt.FLOAT) && //
            std::abs(gps_interframe_dt_) < 0.5 &&                                                           //
            tcmsf_spd_ > 20.0                                                                               //
        ) {
            pos_cmp_.continous_dynamic_gnss_good_inno_++;
        } else {
            pos_cmp_.continous_dynamic_gnss_good_inno_ = 0;
        }
        pre_gnss_measurement_timestamp = gps_data_ptr->measurement_timestamp;
    }

    // 组合导航状态相关
    INS::EARTH earth(gps_data_ptr->lla, gps_data_ptr->vel);
    double     RMh2_   = earth.RMh * earth.RMh;
    double     clRNh2_ = earth.clRNh * earth.clRNh;

    // 计算位置新息
    Eigen::Vector3d pos_lla_inno_{state_ptr->lla - (gps_data_ptr->lla - state_ptr->MpvCnb * state_ptr->C_imu2vehicle * state_ptr->lever_imu2gnss)};
    Eigen::Vector3d pos_enu_inno_{pos_lla_inno_.y() * earth.clRNh, pos_lla_inno_.x() * earth.RMh, pos_lla_inno_.z()};
    Eigen::Vector3d pos_rfu_inno_{state_ptr->C_imu2vehicle * state_ptr->C_b2n.transpose() * pos_enu_inno_};

    state_ptr->pos_rfu_inno_ = pos_rfu_inno_;

    //
    // 认为位置是比较准的条件
    bool gnss_position_may_good_ =
        (gps_data_ptr->status == Parameters::GNSS_STATUS::FIX || gps_data_ptr->status == Parameters::GNSS_STATUS::FLOAT) &&               // 固定、浮点解
        std::fabs(rtk_cross_validation.diffinfo.timestamp - gps_data_ptr->measurement_timestamp) < RtkCrossValidation::CROSS_VALID_DT_ && // 交叉验证满足时间有效性
        std::fabs(rtk_cross_validation.diffinfo.gnss_dr_diff_index) < 0.2;                                                                // 卫星与DR交叉验证相差较小

    if (gnss_position_may_good_) {
        AINFO_EVERY(100) << "gnss_position_may_good_";
        if (std::fabs(process_control_sgt.state_nearby_info.pose_inno_info.pos_timestamp - state_ptr->measurement_timestamp) > process_control_sgt.state_nearby_info.pose_inno_info.TIME_RANGE) {
            process_control_sgt.state_nearby_info.pose_inno_info.pos_count    = 0;
            process_control_sgt.state_nearby_info.pose_inno_info.pos_mean_rfu = Eigen::Vector3d::Zero();
            AINFO << "reset state_nearby_info.pose_inno_info";
        } else {
            auto       &pos_mean_rfu = process_control_sgt.state_nearby_info.pose_inno_info.pos_mean_rfu;
            const auto &weight       = process_control_sgt.state_nearby_info.pose_inno_info.MEAN_WEIGHT_PER_MEASUREMENT;

            pos_mean_rfu = pos_mean_rfu * (1.0 - weight) + pos_rfu_inno_ * weight;

            process_control_sgt.state_nearby_info.pose_inno_info.pos_count++;
        }
        process_control_sgt.state_nearby_info.pose_inno_info.pos_timestamp = state_ptr->measurement_timestamp;
    }

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
    std::string cx_str_ = fmt::format(
        "{:>14.4f},{:>6.3f},{:>6.3f},{:>14.4f},{:>6.3f},{:>6.3f},{:>6.3f}\n",
        gps_data_ptr->measurement_timestamp,               // 1
        state_ptr->state_stability_index_ratio_,           // 2
        pos_enu_inno_.block<2, 1>(0, 0).norm(),            // 3
        rtk_cross_validation.diffinfo.timestamp,           // 4
        rtk_cross_validation.diffinfo.gnss_dr_diff_index,  // 5
        rtk_cross_validation.diffinfo.msf_gnss_diff_index, // 6
        rtk_cross_validation.diffinfo.cross_diff_index     // 7
    );
    debug::debug_sgt.cx_vali_state.line(cx_str_);
#endif

    // GNSS MSG 预处理
    gnss_data_pre_process(gps_data_ptr, state_ptr, rtk_cross_validation.diffinfo);

    rsa_.is_rtk_fix_valid_ = rsa_.rtk_fix_valid_judgement(gps_data_ptr, 15);

    ssi_.state_stability_index_update(gps_data_ptr, state_ptr, pos_enu_inno_, rtk_cross_validation.diffinfo, pos_inno_statistic_info_);

    if (process_control_sgt.vehicle_motion_type == process_control_sgt.MOVING && !rsa_.is_rtk_false_fix_) {

        // RTK状态统计分析
        rsa_.rtk_overall_state_analysis(gps_data_ptr, state_ptr);

        // 位置重置逻辑
        gfc_.reset_state_control(gps_data_ptr, state_ptr, rsa_.is_rtk_fix_valid_);
    }

    // 计算FIX解卫星数均值，如果当前卫星数比这个均值少，则降低当前量测的权重
    double sat_num_diff_index_ = 0.0;
    if (gps_data_ptr->status == Parameters::GNSS_STATUS::FIX) {
        gnss_rtk_fix_count++;
        fix_sat_num_mean = fix_sat_num_mean * 0.99 + gps_data_ptr->num_sats * 0.01;

        state_ptr->rtk_fix_sat_num_mean_ = fix_sat_num_mean;
    }
    if (gnss_rtk_fix_count > 200) {
        sat_num_diff_index_ = (fix_sat_num_mean - gps_data_ptr->num_sats) / (std::abs(fix_sat_num_mean) + 1e-5);
    }

    {
        // 非固定或者浮点解场景下，对卫星进行一个校验
        // 此处主要目的是应对隧道内存在卫星转发器的场景
        bad_gnss_detected_ =
            (fix_sat_num_mean - gps_data_ptr->num_sats > 5.0 || std::fabs(rtk_cross_validation.diffinfo.gnss_dr_diff_index) > 1.0) && // 卫星数变少或者卫星与DR的交叉验证差值过大
            (gps_data_ptr->status != Parameters::GNSS_STATUS::FIX && gps_data_ptr->status != Parameters::GNSS_STATUS::FLOAT)          // 非固定或者浮点解状态
            ;
    }

    // 滤波器相关
    Eigen::Matrix<double, 3, 21> H;
    Eigen::Vector3d              innovation;
    Eigen::Matrix3d              V;
    Eigen::Matrix<double, 8, 1>  inno_ = Eigen::Matrix<double, 8, 1>::Zero();

    Eigen::Matrix<bool, 21, 1> idx;

    // 对于没有进行过零速更新，且RTK为固定解的场景
    // 使用宽松的融合策略
    bool NoSuccessZuptYet_ = state_ptr->zupt_imu_bias_estimate_ok_count == 0;
    bool RtkFix_           = gps_data_ptr->status == Parameters::GNSS_STATUS::FIX;
    bool ShortMileage_     = std::fabs(state_ptr->mileage) < 5e2;

    // 在卫星量测更新过程中，也更新车辆参数
    bool estimate_vehicle_bias_ =
        std::fabs(process_control_sgt.good_state_for_vehicle_bias_estimation.first - gps_data_ptr->measurement_timestamp) < 1.0 &&
        gps_data_ptr->status == Parameters::GNSS_STATUS::FIX &&
        ssi_.get_state_stability_index_ratio_() > 0.02;

    if (parameters_sgt.get_enable_gnss_pos_fusion()) { // 配置是否融合GNSS的位置

        // 这里做个纵向补偿
        if (rtk_cross_validation.CurNaviInfo.first &&          // 有交叉验证结果
            gps_data_ptr->raw_status == 6 &&                   // RTK给的状态是固定解
            state_ptr->vel.norm() > 2.0 &&                     // 有一定速度
            std::abs(state_ptr->gyro.z()) < 2.0 / 180.0 * M_PI // 直行状态
        ) {
            auto           &cur_navi_info        = rtk_cross_validation.CurNaviInfo.second;
            Eigen::Vector3d cur_dr_gnss_diff_rfu = (cur_navi_info.DrDeltaRfu - cur_navi_info.GnssDeltaRfu).cwiseAbs();
            if (cur_dr_gnss_diff_rfu.x() < 0.3 && cur_dr_gnss_diff_rfu.y() < 0.3) {
                // 满足纵向补偿条件
                Eigen::Vector3d inno_  = state_ptr->lla - (gps_data_ptr->lla - state_ptr->MpvCnb * state_ptr->C_imu2vehicle * state_ptr->lever_imu2gnss);
                Eigen::Vector3d dp_rfu = local_trans.DPos_LLAtoEgo(state_ptr->lla, inno_, state_ptr->att);
                if (std::fabs(dp_rfu.y()) > 0.5) {
                    Eigen::Vector3d dp_comp_ = Eigen::Vector3d::Zero();
                    dp_comp_.y()             = dp_rfu.y() / 5.0;
                    local_trans.constrain_if_in_bound(dp_comp_.y(), 1e10, 0.3);
                    Eigen::Vector3d lla_ = local_trans.DPos_Ego2LLA(state_ptr->lla, -dp_comp_, state_ptr->att);
                    state_ptr->lla       = lla_;
                    AINFO << "pos diff comp, diff: " << dp_rfu.y() << " comp: " << dp_comp_.y() << " t:" << fmt::format("{:14.3f}", gps_data_ptr->measurement_timestamp);
                }
            }
        }

        // Compute residual.

        innovation = state_ptr->lla - (gps_data_ptr->lla - state_ptr->MpvCnb * state_ptr->C_imu2vehicle * state_ptr->lever_imu2gnss);

        bool stable_position_error_detected = false;
        { // 记录位置新息
            Eigen::Vector3d pos_{innovation.y() * earth.clRNh, innovation.x() * earth.RMh, innovation.z()};

            rsa_.is_rtk_false_fix_ = rsa_.RTK_false_fix_detect(gps_data_ptr, state_ptr, pos_enu_inno_, pos_rfu_inno_, ssi_.get_state_stability_index_ratio_());

            inno_(0)                = gps_data_ptr->measurement_timestamp;
            inno_.block<3, 1>(1, 0) = pos_;

            // 统计
            pos_cmp_.ref_pos_inno_statistic.insert(pos_);

            if (pos_cmp_.ref_pos_inno_statistic.calc_mean_std()) {
                Eigen::Vector3d pos_inno_mean_ = pos_cmp_.ref_pos_inno_statistic.mean();
                Eigen::Vector3d pos_inno_std_  = pos_cmp_.ref_pos_inno_statistic.std();
                pos_inno_statistic_info_.first = gps_data_ptr->measurement_timestamp;
                pos_inno_statistic_info_.second << std::fabs(pos_inno_mean_.x()) + std::fabs(pos_inno_mean_.y()), pos_inno_std_.x() + pos_inno_std_.y();
                state_ptr->gnss_pos_inno_statistics_.block<3, 1>(0, 0) = pos_;
                state_ptr->gnss_pos_inno_statistics_.block<3, 1>(3, 0) = pos_inno_mean_;
                state_ptr->gnss_pos_inno_statistics_.block<3, 1>(6, 0) = pos_inno_std_;

                if (pos_inno_mean_.block<2, 1>(0, 0).norm() > 0.6 && (pos_ - pos_inno_mean_).block<2, 1>(0, 0).norm() < 0.1 && pos_inno_std_.block<2, 1>(0, 0).norm() < 0.04) {
                    stable_position_error_detected = true;
                }
            }
        }

        if ((rsa_.is_rtk_false_fix_ && !NoSuccessZuptYet_) || bad_gnss_detected_) {
            if (bad_gnss_detected_) {
                AINFO_EVERY(30) << "Bad gnss detected!";
            } else {
                AINFO_EVERY(30) << "RTK false fix detected!";
            }
        } else {
            // RTK连续FIX状态计数
            if (gps_data_ptr->status == Parameters::GNSS_STATUS::FIX &&                    //
                process_control_sgt.fusion_status.imu_update_count_after_gnss_update < 100 //
            ) {
                process_control_sgt.fusion_status.imu_update_count_after_rtk_fix = 0;
                process_control_sgt.fusion_status.continous_gnss_fix_count++;
            } else {
                process_control_sgt.fusion_status.continous_gnss_fix_count = 0;
            }

            if (
                (gps_data_ptr->status == Parameters::GNSS_STATUS::FIX || gps_data_ptr->status == Parameters::GNSS_STATUS::FLOAT) && //
                process_control_sgt.fusion_status.imu_update_count_after_gnss_update < 100                                          //
            ) {
                process_control_sgt.fusion_status.continous_gnss_float_fix_count++;
            } else {
                process_control_sgt.fusion_status.continous_gnss_float_fix_count = 0;
            }

            if (state_ptr->vel.norm() > 10.0 &&
                parameters_sgt.get_enable_ego_position_compensation() && // 启用位置补偿策略
                (gps_data_ptr->status != Parameters::GNSS_STATUS::FIX || process_control_sgt.msf_align_type != process_control_sgt.ALIGNED)) {
                pos_cmp_.ego_position_compensation(state_ptr, innovation);
            }

            H.setZero();

            // Compute jacobian.
            H.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();

            // fuse position

            const auto     &lla_cov_ = gps_data_ptr->lla_cov;
            Eigen::Vector3d lla_cov{lla_cov_(0) / RMh2_, lla_cov_(1) / clRNh2_, lla_cov_(2)};
            Eigen::Vector3d lla_cov_additional = innovation.cwiseAbs2();
            if (gps_data_ptr->status == Parameters::GNSS_STATUS::FIX) {
                lla_cov_additional = lla_cov_additional * 0.36;
            }

            // 构建协方差
            // 通过新息大小自适应协方差
            // 在float时，对量测进行IGG3抗差处理

            double rtk_status_scale_ = gnss_position_additional_std_scale[gps_data_ptr->status];

            if (process_control_sgt.rtk_overall_status == process_control_sgt.MAJORITY_FLOAT && //
                gps_data_ptr->status == Parameters::GNSS_STATUS::FLOAT &&                       //
                process_control_sgt.fusion_status.continous_gnss_float_fix_count > 10) {
                rtk_status_scale_ = gnss_position_additional_std_scale[gps_data_ptr->status] / 1.4;
            }

            Eigen::Vector3d lla_std = lla_cov.cwiseSqrt() + 3.0 * kf_ptr->P.block<3, 3>(6, 6).diagonal().cwiseSqrt();

            auto lla_cov_scale =
                Eigen::Vector3d{
                    1.0 / igg3_pos(innovation.x() / lla_std.x()),
                    1.0 / igg3_pos(innovation.y() / lla_std.y()),
                    1.0 / igg3_pos(innovation.z() / lla_std.z())};
            double pos_norm_scale_ = 1.0 / igg3_pos_norm(innovation.block<2, 1>(0, 0).norm() / (lla_std.block<2, 1>(0, 0).norm() + 1e-10));

            if (sat_num_diff_index_ > 0.0) {
                lla_cov = lla_cov * std::pow(1.0 + 3.0 * sat_num_diff_index_, 2);
            }

            bool msf_position_may_bad_ =
                std::fabs(process_control_sgt.state_nearby_info.pose_inno_info.pos_timestamp - state_ptr->measurement_timestamp) < 0.5 &&
                process_control_sgt.state_nearby_info.pose_inno_info.pos_count > 20 &&
                process_control_sgt.state_nearby_info.pose_inno_info.pos_mean_rfu.block<2, 1>(0, 0).norm() > 10.0;

            if (msf_position_may_bad_ && gnss_position_may_good_) {
                AINFO_EVERY(10) << "[Robustness Suppression] position, msf may bad while gnss may good";
                // 如果RTK比较稳定，融合定位的位置新息又比较大
                // 那么认为融合定位不是很准，这里降低抗差
                lla_cov_additional = lla_cov_additional * 0.5;
                rtk_status_scale_  = rtk_status_scale_ > 1.0 ? 1.0 : rtk_status_scale_;
            }

            if (gps_data_ptr->status == Parameters::GNSS_STATUS::FLOAT || gps_data_ptr->status == Parameters::GNSS_STATUS::FIX) {
                if (NoSuccessZuptYet_ && RtkFix_ && ShortMileage_) {
                    V = lla_cov.asDiagonal();
                } else {
                    switch (process_control_sgt.msf_align_type) {

                        case process_control_sgt.INITIALIZATION:
                            break;

                        case process_control_sgt.COARSE_ALIGN: {
                            V = (lla_cov + lla_cov_additional * rtk_status_scale_).asDiagonal();
                            break;
                        }

                        case process_control_sgt.FINE_ALIGN: {
                            V = (lla_cov + lla_cov_additional * rtk_status_scale_).asDiagonal();
                            break;
                        }

                        case process_control_sgt.ALIGNED: {
                            if (gps_data_ptr->status == Parameters::GNSS_STATUS::FIX) {
                                V = (lla_cov + lla_cov_additional * rtk_status_scale_).asDiagonal();
                            } else {
                                // V = (lla_cov + lla_cov_additional * rtk_status_scale_).asDiagonal();
                                V = (pos_norm_scale_ * lla_cov_scale.asDiagonal() * lla_cov + lla_cov_additional * rtk_status_scale_).asDiagonal();
                            }
                            break;
                        }

                        default:
                            break;
                    }
                }
            } else {
                V = (pos_norm_scale_ * lla_cov_scale.asDiagonal() * lla_cov + lla_cov_additional * rtk_status_scale_).asDiagonal();
            }

            if (tcmsf_not_init_) {
                V = lla_cov.asDiagonal();
            }

            // AINFO_EVERY(100) << "lla cov: " << std::sqrt(V(0, 0) * RMh2_) << " " << std::sqrt(V(1, 1) * clRNh2_);
            kf_ptr->kf_update(H, V, innovation);

            switch (process_control_sgt.vehicle_motion_type) {

                case process_control_sgt.ZERO_VELOCITY: {
                    if (gps_data_ptr->status == Parameters::GNSS_STATUS::FIX && inno_.block<2, 1>(1, 0).norm() < 0.5) {
                        // 处于零速状态时，在GNSS量测更新周期内，只对位置观测进行量测更新
                        state_ptr->lla -= kf_ptr->dx.block<3, 1>(6, 0);
                        process_control_sgt.lla0_zupt = state_ptr->lla;
                        idx << 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
                        CovarianceUpdateByIdx(kf_ptr->P, kf_ptr->cov, idx, true);
                    }
                    break;
                }

                case process_control_sgt.MOVING: {
                    double dpos_bound_ = 1e2;
                    if (process_control_sgt.rtk_overall_status == process_control_sgt.MAJORITY_FIX && gps_data_ptr->status != Parameters::GNSS_STATUS::FIX) {
                        dpos_bound_ = parameters_sgt.get_position_max_delta_lat_feedback();
                    } else {
                        dpos_bound_ = 1e2;
                    }

                    Eigen::Vector3d lla_ = state_ptr->lla;

                    Eigen::Vector3d pos_update_linear_ = positionUpdateLinearBalance(innovation, -kf_ptr->dx.block<3, 1>(6, 0), earth);

                    {
                        // 横向30cm，航向1度
                        static constexpr double VIS_GOOD_POS_INNO_BOUND_ = 0.3;
                        static constexpr double VIS_GOOD_HDG_INNO_BOUND_ = 1.0 / 180.0 * M_PI;

                        bool vis_may_good_ =
                            std::fabs(process_control_sgt.state_nearby_info.vision_inno_info.vis_timestamp - state_ptr->measurement_timestamp) < 3.0 && // 时效性限制
                            process_control_sgt.state_nearby_info.vision_inno_info.vis_count > 10 &&                                                    // 数量限制
                            std::fabs(process_control_sgt.state_nearby_info.vision_inno_info.pos_mean_r) < VIS_GOOD_POS_INNO_BOUND_ &&                  // 横向误差伪均值限制
                            std::fabs(process_control_sgt.state_nearby_info.vision_inno_info.hdg_mean) < VIS_GOOD_HDG_INNO_BOUND_;                      // 航向误差伪均值限制

                        static constexpr double GNSS_POS_INNO_BAD_BOUND_ = 0.5;

                        bool gnss_pos_may_bad_ = (std::fabs(pos_rfu_inno_.x()) > GNSS_POS_INNO_BAD_BOUND_); // 横向新息超过一定值，认为卫星定位可能不好

                        // 假设城区 15 m/s 速度，限制横向修正量 0.45cm
                        // 假设横向 30 cm，限制横向修正量 0.3cm
                        // 差不多1秒限制修正量在7.5cm，前进15m；距离比例 0.5%
                        double ego_dlat_bound_while_vis_good_ = state_ptr->vel.norm() * 3e-4 + std::fabs(process_control_sgt.state_nearby_info.vision_inno_info.pos_mean_r) * 1e-2;
                        if (vis_may_good_ && gnss_pos_may_bad_) {
                            AINFO_EVERY(10) << "[Robustness Suppression] position, Vis may good";
                            dpos_bound_ = ego_dlat_bound_while_vis_good_;
                        }
                    }
                    lla_ = local_trans.LLAUpdateWithEgoLatConstrain(
                        state_ptr->lla,     //
                        pos_update_linear_, //
                        state_ptr->att,     //
                        1e14,               //
                        dpos_bound_         //
                    );
                    pos_cmp_.mapping_bias_ablation_and_compensation(state_ptr->lla, lla_, earth, state_ptr);

                    state_ptr->lla = lla_;

                    // 横向新息大的时候，会拉歪一些速度和航向
                    // 设置一个阈值，横向位置新息很大的时候，就不更新速度和航向了
                    if (std::abs(pos_rfu_inno_.x()) < 10.0) {

                        // if (process_control_sgt.initialization_count >= 2 && process_control_sgt.fusion_status.imu_update_count_aftr_pos_reset < 200) {
                        //     // 如果运行中重置了位置，则缓冲2sec，不使用位置新息更新速度
                        //     AINFO_EVERY(10) << "skip velocity update after LLA reset.";
                        // } else {
                        //     state_ptr->vel -= kf_ptr->dx.block<3, 1>(3, 0);
                        // }

                        state_ptr->vel -= kf_ptr->dx.block<3, 1>(3, 0);

                        // 姿态更新
                        Eigen::Vector3d    datt = constrain(kf_ptr->dx.block<3, 1>(0, 0), MaxEulrAnglePerMeasurementUpdate);
                        Eigen::Quaterniond dq   = INS::rv2q(datt);

                        state_ptr->att = dq * state_ptr->att;
                        state_ptr->att.normalize();

                        // if (process_control_sgt.initialization_count >= 2 && process_control_sgt.fusion_status.imu_update_count_aftr_pos_reset < 200) {
                        //     // 如果运行中重置了位置，则缓冲2sec，不使用位置新息更新航向
                        //     AINFO_EVERY(10) << "skip attitude update after LLA reset.";
                        // } else {
                        //     state_ptr->att = dq * state_ptr->att;
                        // }

                        state_ptr->gyro_bias += kf_ptr->dx.block<3, 1>(9, 0);
                        state_ptr->acc_bias += kf_ptr->dx.block<3, 1>(12, 0);

                        if (estimate_vehicle_bias_) {
                            double spd_scale_bias_ = constrain(kf_ptr->dx(16), 0.0003);
                            state_ptr->vehicle_bias(1) -= spd_scale_bias_;
                            state_ptr->wheel_spd_scale_bias_ = wheel_spd_scale_adapter(state_ptr->vehicle_bias(1), state_ptr->vel.norm());
                        }
                    }
                    StateConstrain(state_ptr->gyro_bias, constrain_gyro_bias);
                    StateConstrain(state_ptr->acc_bias, constrain_acc_bias);

                    kf_ptr->P = kf_ptr->cov;

                    // // 姿态、速度、位置、陀螺零偏、加计零偏
                    // kf_ptr->es.block<15, 1>(0, 0) = 0.0 * kf_ptr->dx.block<15, 1>(0, 0);

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
                    Eigen::Matrix<double, 15, 1> dx_ = kf_ptr->dx.block<15, 1>(0, 0);

                    auto line_ =
                        fmt::format("{:>14.4f},{:>6.8f},{:>6.8f},{:>6.8f},{:>6.8f},{:>6.8f},{:>6.8f},{:>6.8f},{:>6.8f},{:>6.8f},{:>6.8f},{:>6.8f},{:>6.8f},{:>6.8f},{:>6.8f},{:>6.8f},"
                                    "{:>6.8f},{:>6.8f},{:>6.8f},{:>6.8f},{:>6.8f},{:>6.8f},{:d}\n",
                                    gps_data_ptr->measurement_timestamp, // 1
                                    dx_(0),                              // 2
                                    dx_(1),                              // 3
                                    dx_(2),                              // 4
                                    dx_(3),                              // 5
                                    dx_(4),                              // 6
                                    dx_(5),                              // 7
                                    dx_(6),                              // 8
                                    dx_(7),                              // 9
                                    dx_(8),                              // 10
                                    dx_(9),                              // 11
                                    dx_(10),                             // 12
                                    dx_(11),                             // 13
                                    dx_(12),                             // 14
                                    dx_(13),                             // 15
                                    dx_(14),                             // 16
                                    pos_enu_inno_.x(),                   // 17
                                    pos_enu_inno_.y(),                   // 18
                                    pos_enu_inno_.z(),                   // 19
                                    pos_rfu_inno_.x(),                   // 20
                                    pos_rfu_inno_.y(),                   // 21
                                    pos_rfu_inno_.z(),                   // 22
                                    gps_data_ptr->status                 // 23
                        );
                    debug::debug_sgt.bias_esti_state.line(line_);
#endif

                    break;
                }

                default:
                    break;
            }

            // 通过GNSS的权重判断GNSS更新有效性，若更新有效，则重置IMU更新计数
            bool gnss_update_fix_ =
                kf_ptr->K.block<2, 2>(6, 0).diagonal().norm() > 2e-2 || gps_data_ptr->status == Parameters::GNSS_STATUS::FIX;
            bool gnss_update_float_ =
                kf_ptr->K.block<2, 2>(6, 0).diagonal().norm() > 2e-3 &&
                (process_control_sgt.rtk_overall_status == process_control_sgt.MAJORITY_FLOAT) &&
                (gps_data_ptr->status == Parameters::GNSS_STATUS::FIX || gps_data_ptr->status == Parameters::GNSS_STATUS::FLOAT);
            if (gnss_update_fix_ || gnss_update_float_) {
                process_control_sgt.fusion_status.imu_update_count_after_gnss_update = 0;
                process_control_sgt.fusion_status.continous_gnss_fusion_count++;
            }
        }

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
        Eigen::Matrix<double, 6, 1> imu_P = kf_ptr->P.block<6, 6>(9, 9).diagonal().cwiseSqrt();
        debug::debug_sgt.imu_state.line(fmt::format(
            "{:>14.4f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f}",
            state_ptr->measurement_timestamp,
            state_ptr->gyro_bias.x() * 180 / M_PI,
            state_ptr->gyro_bias.y() * 180 / M_PI,
            state_ptr->gyro_bias.z() * 180 / M_PI,
            state_ptr->acc_bias.x(),
            state_ptr->acc_bias.y(),
            state_ptr->acc_bias.z()));
        debug::debug_sgt.imu_state.line(fmt::format(",{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f}\n", imu_P[0] * 180 / M_PI, imu_P[1] * 180 / M_PI, imu_P[2] * 180 / M_PI, imu_P[3], imu_P[4], imu_P[5]));
#endif
#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
        Eigen::Matrix<double, 9, 1> avp_P_ = kf_ptr->P.block<9, 9>(0, 0).diagonal().cwiseSqrt();
        Eigen::Matrix<double, 9, 1> davp_  = kf_ptr->dx.block<9, 1>(0, 0);
        Eigen::Matrix<double, 3, 1> pos_k_ = kf_ptr->K.block<3, 3>(6, 0).diagonal();
        debug::debug_sgt.gps_update.line(fmt::format(
            "{:>14.4f},"
            "{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},"
            "{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},"
            "{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f}\n",
            state_ptr->measurement_timestamp, // 1
            davp_(0),                         // 2
            davp_(1),                         // 3
            davp_(2),                         // 4
            davp_(3),                         // 5
            davp_(4),                         // 6
            davp_(5),                         // 7
            davp_(6) * earth.RMh,             // 8
            davp_(7) * earth.clRNh,           // 9
            davp_(8),                         // 10
            avp_P_(0),                        // 11
            avp_P_(1),                        // 12
            avp_P_(2),                        // 13
            avp_P_(3),                        // 14
            avp_P_(4),                        // 15
            avp_P_(5),                        // 16
            avp_P_(6) * earth.RMh,            // 17
            avp_P_(7) * earth.clRNh,          // 18
            avp_P_(8),                        // 19
            pos_k_.x(),                       // 20
            pos_k_.y(),                       // 21
            pos_k_.z(),                       // 22
            pos_k_.block<2, 1>(0, 0).norm()   // 23
            ));
#endif
    }

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
    Eigen::Vector3d lla_std_origin_   = gps_data_ptr->lla_cov.cwiseSqrt();
    Eigen::Vector3d lla_std_modified_ = (Eigen::Vector3d{V(0, 0) * RMh2_, V(1, 1) * clRNh2_, V(2, 2)}).cwiseSqrt();
    std::string     pos_std_info_ =
        fmt::format("{:>14.4f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f}", //
                    state_ptr->measurement_timestamp,                                                             // 1
                    lla_std_origin_.x(),                                                                          // 2
                    lla_std_origin_.y(),                                                                          // 3
                    lla_std_origin_.z(),                                                                          // 4
                    lla_std_modified_.x(),                                                                        // 5
                    lla_std_modified_.y(),                                                                        // 6
                    lla_std_modified_.z(),                                                                        // 7
                    innovation.x() * earth.RMh,                                                                   // 8
                    innovation.y() * earth.clRNh,                                                                 // 9
                    innovation.z()                                                                                // 10

        );
#endif

    // --------------------------------
    if (parameters_sgt.get_enable_gnss_vel_fusion() && !bad_gnss_detected_) { // 配置是否融合GNSS的速度
        {
            H.setZero();
            innovation = state_ptr->vel + state_ptr->C_b2n * (INS::Skew(state_ptr->web) * state_ptr->C_imu2vehicle * state_ptr->lever_imu2gnss) - gps_data_ptr->vel;

            // Compute jacobian.
            H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
            if (state_ptr->vel.norm() > parameters_sgt.get_slow_speed_bound()) {
                inno_.block<3, 1>(4, 0) = innovation;
            }
        }
        // 低速GNSS速度不可靠
        if (state_ptr->vel.norm() > parameters_sgt.get_slow_speed_bound()) {
            // LOG(ERROR) << "fuse gnss vel";
            // fuse velocity
            Eigen::Vector3d vel_cov = gps_data_ptr->vel_cov;

            if (sat_num_diff_index_ > 0.0) {
                vel_cov = vel_cov * std::pow(1.0 + 3.0 * sat_num_diff_index_, 2);
            }

            double          hori_acc_                      = (state_ptr->acc - state_ptr->acc_bias).block<2, 1>(0, 0).norm();
            Eigen::Vector3d acc_related_vel_cov_additional = (hori_acc_ * Eigen::Vector3d::Ones()).cwiseAbs2();

            Eigen::Vector3d vel_cov_additional = innovation.cwiseAbs2() + acc_related_vel_cov_additional;

            auto vel_std = vel_cov.cwiseSqrt() + 9.0 * kf_ptr->P.block<3, 3>(3, 3).diagonal().cwiseSqrt();

            auto vel_cov_scale =
                Eigen::Vector3d{
                    1.0 / igg3_vel(innovation.x() / vel_std.x()),
                    1.0 / igg3_vel(innovation.y() / vel_std.y()),
                    1.0 / igg3_vel(innovation.z() / vel_std.z())};
            if (NoSuccessZuptYet_ && RtkFix_ && ShortMileage_) {
                V = (vel_cov + vel_cov_additional).asDiagonal();
            } else {
                V = (vel_cov_scale.asDiagonal() * vel_cov + vel_cov_additional * gnss_velocity_additional_std_scale[gps_data_ptr->status]).asDiagonal();
            }

            kf_ptr->kf_update(H, V, innovation);

            bool update_att_by_vel_update_ = true;

            switch (process_control_sgt.vehicle_motion_type) {

                case process_control_sgt.ZERO_VELOCITY: {
                    break;
                }

                case process_control_sgt.MOVING: {

                    state_ptr->vel -= kf_ptr->dx.block<3, 1>(3, 0);
                    state_ptr->lla -= kf_ptr->dx.block<3, 1>(6, 0);

                    Eigen::Vector3d    datt = constrain(kf_ptr->dx.block<3, 1>(0, 0), MaxEulrAnglePerMeasurementUpdate);
                    Eigen::Quaterniond dq   = INS::rv2q(datt);
                    state_ptr->att          = dq * state_ptr->att;
                    state_ptr->att.normalize();

                    state_ptr->gyro_bias += kf_ptr->dx.block<3, 1>(9, 0);
                    state_ptr->acc_bias += kf_ptr->dx.block<3, 1>(12, 0);

                    if (estimate_vehicle_bias_) {
                        double spd_scale_bias_ = constrain(kf_ptr->dx(16), 0.0003);
                        state_ptr->vehicle_bias(1) -= spd_scale_bias_;
                        state_ptr->wheel_spd_scale_bias_ = wheel_spd_scale_adapter(state_ptr->vehicle_bias(1), state_ptr->vel.norm());
                    }

                    StateConstrain(state_ptr->gyro_bias, constrain_gyro_bias);
                    StateConstrain(state_ptr->acc_bias, constrain_acc_bias);

                    kf_ptr->P = kf_ptr->cov;

                    break;
                }

                default:
                    break;
            }
        }
    }

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
    Eigen::Vector3d vel_std_origin_   = gps_data_ptr->vel_cov.cwiseSqrt();
    Eigen::Vector3d vel_std_modified_ = Eigen::Vector3d{V(0, 0), V(1, 1), V(2, 2)}.cwiseSqrt();
    std::string     vel_std_info_ =
        fmt::format("{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f},{:>6.6f}", //
                    vel_std_origin_.x(),                                                                // 11
                    vel_std_origin_.y(),                                                                // 12
                    vel_std_origin_.z(),                                                                // 13
                    vel_std_modified_.x(),                                                              // 14
                    vel_std_modified_.y(),                                                              // 15
                    vel_std_modified_.z(),                                                              // 16
                    innovation.x(),                                                                     // 17
                    innovation.y(),                                                                     // 18
                    innovation.z()                                                                      // 19
        );
#endif

    if (parameters_sgt.get_enable_gnss_heading_fusion() && !bad_gnss_detected_) { // 配置是否融合GNSS的航向

        // innovation.setZero();
        H.setZero();

        // 这里要考虑下，要不要补偿IMU的安装角偏差
        // 比较担心的一点是如果补偿的话，这个安装角偏差的约束性比较弱，可能会不受控制的增加
        // 但是理论上，肯定是要补偿，才可以得到最优的航向
        double veh_hdg_ = INS::quaternion2euler(state_ptr->att * state_ptr->q_imu2vehicle.conjugate()).z();
        double dhdg_    = -gps_data_ptr->hdg - veh_hdg_;

        // 侧滑角补偿
        if (state_ptr->vel_ego.y() > 0.0) {
            Eigen::Vector3d gyro_ = state_ptr->gyro - state_ptr->gyro_bias;

            double vel_norm_   = state_ptr->vel.norm();
            double K           = 1.0 / 3.2 / 180.0 * M_PI;
            double slip_angle_ = gyro_.z() * vel_norm_ * K;
            {
                // 做个限制，侧滑角补偿最大量控制在4度以内，以免出现数值异常情况
                slip_angle_ = slip_angle_ > 4.0 / 180.0 * M_PI ? 4.0 / 180.0 * M_PI : slip_angle_;
                slip_angle_ = slip_angle_ < -4.0 / 180.0 * M_PI ? -4.0 / 180.0 * M_PI : slip_angle_;
            }
            dhdg_ += slip_angle_;
        }

        if (dhdg_ >= M_PI) {
            dhdg_ = dhdg_ - 2.0 * M_PI;
        }
        if (dhdg_ <= -M_PI) {
            dhdg_ = dhdg_ + 2.0 * M_PI;
        }
        innovation = Eigen::Vector3d{0.0, 0.0, dhdg_};

        // 认为航向是比较准的条件
        bool gnss_heading_may_good_ =
            state_ptr->vel.norm() > 7.0 &&                        // 速度较快
            gps_data_ptr->status == Parameters::GNSS_STATUS::FIX; //固定解

        if (gnss_heading_may_good_) {
            // 在卫星量测持续更新中，记录航向的新息
            if (std::fabs(process_control_sgt.state_nearby_info.heading_inno_info.timestamp - state_ptr->measurement_timestamp) > process_control_sgt.state_nearby_info.heading_inno_info.TIME_RANGE) {
                process_control_sgt.state_nearby_info.heading_inno_info.mean_compensated = 0.0;
                process_control_sgt.state_nearby_info.heading_inno_info.mean_raw_imu     = 0.0;
                process_control_sgt.state_nearby_info.heading_inno_info.mean_raw_veh     = 0.0;
                process_control_sgt.state_nearby_info.heading_inno_info.hdg_count        = 0;
            } else {
                auto       &mean_compensated = process_control_sgt.state_nearby_info.heading_inno_info.mean_compensated;
                auto       &mean_raw_imu     = process_control_sgt.state_nearby_info.heading_inno_info.mean_raw_imu;
                auto       &mean_raw_veh     = process_control_sgt.state_nearby_info.heading_inno_info.mean_raw_veh;
                const auto &weight_          = process_control_sgt.state_nearby_info.heading_inno_info.MEAN_WEIGHT_PER_MEASUREMENT;

                double inno_compensated_ = dhdg_;
                double inno_raw_imu_     = -gps_data_ptr->hdg - INS::quaternion2euler(state_ptr->att).z();
                double inno_raw_veh_     = inno_raw_imu_ + state_ptr->vehicle_bias.z();

                mean_compensated = mean_compensated * (1.0 - weight_) + inno_compensated_ * weight_;
                mean_raw_imu     = mean_raw_imu * (1.0 - weight_) + inno_raw_imu_ * weight_;
                mean_raw_veh     = mean_raw_veh * (1.0 - weight_) + inno_raw_veh_ * weight_;

                process_control_sgt.state_nearby_info.heading_inno_info.hdg_count++;
            }
            process_control_sgt.state_nearby_info.heading_inno_info.timestamp = state_ptr->measurement_timestamp;
        }

        // Compute jacobian.
        H.block<1, 1>(2, 2) << 1.0;

        if (state_ptr->vel.norm() > parameters_sgt.get_slow_speed_bound()) {
            inno_(7) = innovation(2);
        }

        // 判断一下，刚开始初始化的时候，放宽航向抗差，以提高航向收敛速度
        bool fast_heading_convergence_ =
            state_ptr->vel.norm() > parameters_sgt.get_slow_speed_bound() &&                                                                                     // 有一定速度
            gps_data_ptr->status == Parameters::GNSS_STATUS::FIX &&                                                                                              // 固定解状态
            std::fabs(state_ptr->mileage) < 100.0 &&                                                                                                             // 前100米里程
            (process_control_sgt.msf_align_type == process_control_sgt.INITIALIZATION || process_control_sgt.msf_align_type == process_control_sgt.COARSE_ALIGN) // 初始化 或者 粗对准
            ;

        // 有速度场景，融合GNSS航向
        if (
            state_ptr->vel.block<2, 1>(0, 0).norm() > parameters_sgt.get_slow_speed_bound() && // 有速度
            !state_ptr->tire_slip &&                                                           // 底盘非打滑状态
            std::abs(state_ptr->slip_index) < parameters_sgt.get_slip_index_rejection_bound()  // 低动态
        ) {

            // 构造航向量测的方差
            // 弯道场景，航向不可靠
            double hdg_scale_ = 1.0;
            double hdg_std_   = 0.03;
            // RTK的航向，权重高其实会引入较高的噪声
            // 这里也做一些策略
            if (state_ptr->zupt_imu_bias_estimate_ok_count == 0) {
                hdg_scale_ = 1.0 + std::abs(state_ptr->gyro.z()) / (8.0 / 180.0 * M_PI);
                hdg_std_   = hdg_scale_ * (1.0 + 10.0 / (state_ptr->vel.norm() + 1.0)) / 180.0 * M_PI;
            } else {
                hdg_scale_ = 1.0 + std::abs(state_ptr->gyro.z()) / (2.0 / 180.0 * M_PI);
                hdg_std_   = hdg_scale_ * (1.0 + 20.0 / (state_ptr->vel.norm() + 1.0)) / 180.0 * M_PI;
            }
            double          att_cov_igg_scale = 1.0 / igg3_hdg(innovation.z() / hdg_std_);
            Eigen::Vector3d att_cov           = (att_cov_igg_scale * std::pow(hdg_std_, 2) + gps_data_ptr->hdg_cov) * Eigen::Vector3d::Ones();

            if (sat_num_diff_index_ > 0.0) {
                att_cov = att_cov * std::pow(1.0 + 3.0 * sat_num_diff_index_, 2);
            }

            Eigen::Vector3d att_cov_additional = innovation.cwiseAbs2();

            Eigen::Vector3d dh_additional = Eigen::Vector3d::Zero();

            if (gps_data_ptr->status != parameters_sgt.FIX) {
                dh_additional.z() = ssi_.get_state_stability_index_() * ssi_.get_state_stability_index_() / 1e4;
            }

            if (fast_heading_convergence_) {
                AINFO_EVERY(10) << "init stage, accelerate heading convergence";
                V = (gps_data_ptr->hdg_cov * Eigen::Vector3d::Ones() + att_cov_additional).asDiagonal();
            } else {
                bool msf_heading_may_bad_ =
                    std::fabs(process_control_sgt.state_nearby_info.heading_inno_info.timestamp - state_ptr->measurement_timestamp) < 0.5 &&
                    process_control_sgt.state_nearby_info.heading_inno_info.hdg_count > 20 &&
                    std::fabs(process_control_sgt.state_nearby_info.heading_inno_info.mean_compensated) > 0.3 / 180.0 * M_PI;
                if (msf_heading_may_bad_ && gnss_heading_may_good_) {
                    AINFO_EVERY(10) << "msf heading may bad, accelerate heading convergence";
                    V = ((0.01 * 0.01 + gps_data_ptr->hdg_cov) * Eigen::Vector3d::Ones() + att_cov_additional).asDiagonal();
                } else {
                    V = (att_cov + att_cov_additional * gnss_heading_additional_std_scale[gps_data_ptr->status] + dh_additional).asDiagonal();
                }
            }

            kf_ptr->kf_update(H, V, innovation);

            switch (process_control_sgt.vehicle_motion_type) {
                case process_control_sgt.ZERO_VELOCITY: {
                    break;
                }

                case process_control_sgt.MOVING: {

                    if (fast_heading_convergence_) {
                        const static Eigen::Vector3d MaxEulrUpdateForFHC = (1.0 * kDegreeToRadian) * Eigen::Vector3d::Ones();

                        Eigen::Vector3d    datt = constrain(kf_ptr->dx.block<3, 1>(0, 0), MaxEulrUpdateForFHC);
                        Eigen::Quaterniond dq   = INS::rv2q(datt);
                        state_ptr->att          = dq * state_ptr->att;
                        state_ptr->att.normalize();
                        kf_ptr->P = kf_ptr->cov;
                    } else {

                        Eigen::Vector3d    datt = constrain(kf_ptr->dx.block<3, 1>(0, 0), MaxEulrAnglePerMeasurementUpdate);
                        Eigen::Quaterniond dq   = INS::rv2q(datt);
                        state_ptr->att          = dq * state_ptr->att;
                        state_ptr->att.normalize();
                        state_ptr->vel -= kf_ptr->dx.block<3, 1>(3, 0);
                        // state_ptr->lla -= kf_ptr->dx.block<3, 1>(6, 0);

                        state_ptr->gyro_bias += kf_ptr->dx.block<3, 1>(9, 0);
                        state_ptr->acc_bias += kf_ptr->dx.block<3, 1>(12, 0);

                        kf_ptr->P = kf_ptr->cov;
                    }
                    break;
                }

                default:
                    break;
            }
        } else {
        }
    }

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
    double      hdg_std_origin_   = std::sqrt(gps_data_ptr->hdg_cov);
    double      hdg_std_modified_ = std::sqrt(V(2, 2));
    std::string hdg_std_info_ =
        fmt::format("{:>6.6f},{:>6.6f},{:>6.6f}", //
                    hdg_std_origin_,              // 20
                    hdg_std_modified_,            // 21
                    innovation.z()                // 22
        );

    debug::debug_sgt.gps_std.line(pos_std_info_ + "," + vel_std_info_ + "," + hdg_std_info_ + "\n");
#endif

    if (process_control_sgt.vehicle_motion_type == process_control_sgt.MOVING) {
        // 判断融合定位的滤波器状态
        gfc_.msf_align_type_control(gps_data_ptr, state_ptr, inno_);
    }

    state_ptr->gnss_inno_ = inno_;

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
    {

        auto &mean_compensated = process_control_sgt.state_nearby_info.heading_inno_info.mean_compensated;
        auto &mean_raw_imu     = process_control_sgt.state_nearby_info.heading_inno_info.mean_raw_imu;
        auto &mean_raw_veh     = process_control_sgt.state_nearby_info.heading_inno_info.mean_raw_veh;

        Eigen::Matrix3d ref2body     = state_ptr->C_imu2vehicle * state_ptr->C_b2n.transpose();
        Eigen::Vector3d inno_pos_ego = ref2body * inno_.block<3, 1>(1, 0);
        Eigen::Vector3d inno_vel_ego = ref2body * inno_.block<3, 1>(4, 0);
        Eigen::Vector3d lla_std_ego  = ref2body * gps_data_ptr->lla_cov.cwiseSqrt();
        Eigen::Vector3d vel_std_ego  = ref2body * gps_data_ptr->vel_cov.cwiseSqrt();
        double          lat_mars = 0.0, lon_mars = 0.0;
        // wgtochina_lb(0, gps_data_ptr->lla.y() * 180.0 / M_PI, gps_data_ptr->lla.x() * 180.0 / M_PI, gps_data_ptr->lla.z(), 0, 0, &lon_mars, &lat_mars);
        lon_mars      = gps_data_ptr->lla.y() * 180.0 / M_PI;
        lat_mars      = gps_data_ptr->lla.x() * 180.0 / M_PI;
        auto gps_str_ = fmt::format("{:>14.4f},{:>14.10f},{:>14.10f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:d},{:d},{:>14.10f},{:>14.10f},{:>6.3f},"
                                    "{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:d},{:d},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},"
                                    "{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f}\n",
                                    gps_data_ptr->measurement_timestamp,              // 1
                                    gps_data_ptr->lla.x() * 180.0 / M_PI,             // 2
                                    gps_data_ptr->lla.y() * 180.0 / M_PI,             // 3
                                    gps_data_ptr->lla.z(),                            // 4
                                    gps_data_ptr->vel.x(),                            // 5
                                    gps_data_ptr->vel.y(),                            // 6
                                    gps_data_ptr->vel.z(),                            // 7
                                    gps_data_ptr->hdg,                                // 8
                                    gps_data_ptr->num_sats,                           // 9
                                    gps_data_ptr->status,                             // 10
                                    lat_mars,                                         // 11
                                    lon_mars,                                         // 12
                                    std::sqrt(gps_data_ptr->lla_cov.x()),             // 13
                                    std::sqrt(gps_data_ptr->lla_cov.y()),             // 14
                                    std::sqrt(gps_data_ptr->lla_cov.z()),             // 15
                                    std::sqrt(gps_data_ptr->vel_cov.x()),             // 16
                                    std::sqrt(gps_data_ptr->vel_cov.y()),             // 17
                                    std::sqrt(gps_data_ptr->vel_cov.z()),             // 18
                                    std::sqrt(gps_data_ptr->hdg_cov),                 // 19
                                    (uint64_t)process_control_sgt.rtk_overall_status, // 20 rtk overall state
                                    (uint64_t)process_control_sgt.msf_align_type,     // 21
                                    inno_pos_ego.x(),                                 // 22
                                    inno_pos_ego.y(),                                 // 23
                                    inno_pos_ego.z(),                                 // 24
                                    inno_vel_ego.x(),                                 // 25
                                    inno_vel_ego.y(),                                 // 26
                                    inno_vel_ego.z(),                                 // 27
                                    inno_(7, 0) * 180.0 / M_PI,                       // 28
                                    lla_std_ego.x(),                                  // 29
                                    lla_std_ego.y(),                                  // 30
                                    lla_std_ego.z(),                                  // 31
                                    vel_std_ego.x(),                                  // 32
                                    vel_std_ego.y(),                                  // 33
                                    vel_std_ego.z(),                                  // 34
                                    fix_sat_num_mean,                                 // 35
                                    mean_compensated,                                 // 36
                                    mean_raw_imu,                                     // 37
                                    mean_raw_veh                                      // 38
        );
        debug::debug_sgt.gps_state.line(gps_str_);
    }
#endif

    return true;
}

bool RtkStatusAnalysis::rtk_fix_valid_judgement(GnssDataPtr gps_data_ptr, uint64_t bound_) {
    if (gps_data_ptr->status == 6) {
        continous_rtk_fix_count_++;
    } else {
        continous_rtk_fix_count_ = 0;
    }
    if (continous_rtk_fix_count_ >= bound_) {
        return true;
    } else {
        return false;
    }
}

void SinsStableIndex::state_stability_index_update(const GnssDataPtr gps_data_ptr, const StatePtr state_ptr, const Eigen::Vector3d pos_enu_inno_, const RtkCrossValidation::DiffInfo &diffinfo, const std::pair<double, Eigen::Vector2d> &pos_inno_info_) {
    double delta_mileage_ = std::abs(ssi_pre_mileage_ - state_ptr->mileage);
    double delta_time_    = std::abs(ssi_pre_timestamp_ - gps_data_ptr->measurement_timestamp);

    // 考虑一种情况，卫星看起来稳定，但是新息比较大
    // 这种就认为递推的结果不太可靠了
    bool good_gnss_while_inno_large_ =
        std::fabs(diffinfo.timestamp - gps_data_ptr->measurement_timestamp) < RtkCrossValidation::CROSS_VALID_DT_ && // 统计时间小于 3 秒
        state_ptr->vel.norm() > 0.3 &&                                                                               // 速度大于 0.3 m/s
        std::fabs(diffinfo.gnss_dr_diff_index) < 0.2 &&                                                              // GNSS与DR的增量差均值小于 0.2 米，量测前后帧变化量稳定
        std::fabs(pos_inno_info_.first - gps_data_ptr->measurement_timestamp) < 3.0 &&                               // 统计时间小于 3 秒
        pos_inno_info_.second.y() < 0.2 &&                                                                           // 平面位置标准差L1范数小于 0.2 米，量测新息稳定
        pos_inno_info_.second.x() > 0.5                                                                              // 平面位置新息L1范数大于 0.5 米，递推偏离卫星位置
        ;

    double inno_index_ = pos_inno_info_.second.x() / 0.5;
    inno_index_        = inno_index_ > 5.0 ? 5.0 : inno_index_;

    double good_gnss_while_inno_large_index_ = inno_index_ + (0.2 - pos_inno_info_.second.y() + 0.2 - std::fabs(diffinfo.gnss_dr_diff_index)) * 20.0;

    if (delta_time_ < 1.0 && gps_data_ptr->status == parameters_sgt.FIX) {
        double inno_norm_ = pos_enu_inno_.block<2, 1>(0, 0).norm();
        double scale_     = 1.0;
        if (inno_norm_ < 0.05) {
            scale_ = 1.0 + (0.05 - inno_norm_) / 0.01;
        } else if (inno_norm_ < 0.1) {
            scale_ = 1.0;
        } else if (inno_norm_ < 0.2) {
            scale_ = -0.2;
        } else if (inno_norm_ < 0.3) {
            scale_ = -1.0;
        } else if (inno_norm_ < 0.5) {
            scale_ = -2.0;
        } else {
            scale_ = -5.0;
        }

        state_stability_index_ += delta_mileage_ * scale_;
    } else {
        if (good_gnss_while_inno_large_) {
            state_stability_index_ -= good_gnss_while_inno_large_index_ * delta_mileage_;
            state_stability_index_ -= 2.0 * good_gnss_while_inno_large_index_ * delta_time_;
            AINFO_EVERY(30) << "good_gnss_while_inno_large_: " << good_gnss_while_inno_large_index_;
        } else {
            state_stability_index_ -= delta_mileage_;
        }
        if (state_ptr->vel.norm() > 1e-2) {
            state_stability_index_ -= delta_time_ * 10.0;
        } else {
            state_stability_index_ -= delta_time_;
        }
    }

    if (state_ptr->violent_bump_detected) {
        state_ptr->violent_bump_detected = false;
        AWARN << "high dynamic scene detected!";
        if (state_stability_index_ > 30.0) {
            state_stability_index_ = 30.0;
        }
    }

    if (state_stability_index_ > SSI_MAXIMUM_) {
        state_stability_index_ = SSI_MAXIMUM_;
    }

    if (state_stability_index_ < SSI_MINIMUM_) {
        state_stability_index_ = SSI_MINIMUM_;
    }

    ssi_pre_mileage_   = state_ptr->mileage;
    ssi_pre_timestamp_ = gps_data_ptr->measurement_timestamp;

    state_ptr->state_stability_index_ratio_ = get_state_stability_index_ratio_();
}

void GpsProcessor::gnss_data_pre_process(GnssDataPtr gps_data_ptr, StatePtr state_ptr, const RtkCrossValidation::DiffInfo &diffinfo) {

    if (parameters_sgt.get_gnss_override_raw_std()) { // 使用配置文件替代GNSS消息中的标准差
        double pos_fix_std    = parameters_sgt.get_gnss_position_bias_std(Parameters::GNSS_STATUS::FIX);
        double pos_float_std  = parameters_sgt.get_gnss_position_bias_std(Parameters::GNSS_STATUS::FLOAT);
        double pos_single_std = parameters_sgt.get_gnss_position_bias_std(Parameters::GNSS_STATUS::SINGLE);
        double vel_fix_std    = parameters_sgt.get_gnss_velocity_bias_std(Parameters::GNSS_STATUS::FIX);
        double vel_float_std  = parameters_sgt.get_gnss_velocity_bias_std(Parameters::GNSS_STATUS::FLOAT);
        double vel_single_std = parameters_sgt.get_gnss_velocity_bias_std(Parameters::GNSS_STATUS::SINGLE);

        // GNSS 高度上的不确定性，通常较高
        if (gps_data_ptr->status == 6) { // gnss fix
            Eigen::Vector3d std(pos_fix_std, pos_fix_std, pos_fix_std * 3.0);
            gps_data_ptr->lla_cov = std.array().pow(2).matrix();

            std << vel_fix_std, vel_fix_std, vel_fix_std * 3.0;
            gps_data_ptr->vel_cov = std.array().pow(2).matrix();
        } else if (gps_data_ptr->status == 5) { // gnss float
            Eigen::Vector3d std(pos_float_std, pos_float_std, pos_float_std * 3.0);
            gps_data_ptr->lla_cov = std.array().pow(2).matrix();

            std << vel_float_std, vel_float_std, vel_float_std * 3.0;
            gps_data_ptr->vel_cov = std.array().pow(2).matrix();
        } else {
            Eigen::Vector3d std(pos_single_std, pos_single_std, pos_single_std * 3.0);
            gps_data_ptr->lla_cov = std.array().pow(2).matrix();

            std << vel_single_std, vel_single_std, vel_single_std * 3.0;
            gps_data_ptr->vel_cov = std.array().pow(2).matrix();
        }

    } else {

        { // 避免出现异常值

            double spd_               = gps_data_ptr->vel.norm();
            double pos_sd_additional_ = spd_ / 15.0 * 0.05;

            if (state_ptr->zupt_imu_bias_estimate_ok_count == 0) {
                // 如果还没进行过零速更新
                // 陀螺零偏应当还没有较好的估计
                // 这个时候，稍微放宽一些RTK的权重
                pos_sd_additional_ = 0.0;
            }

            if (pos_sd_additional_ > 0.05) {
                pos_sd_additional_ = 0.05;
            }

            for (int i = 0; i < 2; ++i) {
                double &pos_sd = gps_data_ptr->lla_cov[i];
                double &vel_sd = gps_data_ptr->vel_cov[i];
                if (pos_sd < 0.10 + pos_sd_additional_) {
                    pos_sd = 0.10 + pos_sd_additional_;
                }
                if (vel_sd < 0.15) {
                    vel_sd = 0.15;
                }
            }
        }

        {

            if (gps_data_ptr->status != state_ptr->rtk_status) {
                process_control_sgt.fusion_status.rtk_status_maintain_count = 0;
            } else {
                process_control_sgt.fusion_status.rtk_status_maintain_count++;
            }
            state_ptr->rtk_status = gps_data_ptr->status;
        }

        // 已经完成初始化之后，再做额外的限制逻辑
        if (tcmsf_not_init_) {
        } else {
            {
                bool rtk_overall_good_ =
                    process_control_sgt.rtk_overall_status == ProcessControl::MAJORITY_FIX ||
                    process_control_sgt.rtk_overall_status == ProcessControl::MEDIUM_FIX;
                // 在融合定位比较稳定、RTK整体上良好的时候，对固定解做一定的剔除策略
                // 即方差过大的固定解，认为是浮点解
                // 如果融合定位本身都不稳定或者RTK整体上也不怎么好，那么应该没什么必要剔除给了固定解的卫星数据
                if ((gps_data_ptr->lla_cov.block<2, 1>(0, 0).norm() > 1.5 &&   //
                     gps_data_ptr->status == 6 &&                              //
                     rtk_overall_good_                                         //
                     )                                                         //
                    ||                                                         //
                    (gps_data_ptr->status == 6 && gps_data_ptr->rtk_age > 5.0) // RTK 基站信息如果过时的话，认为是浮点解
                ) {
                    AINFO_EVERY(40) << "RTK fix while std is too large, set to float.";
                    gps_data_ptr->status = 5;
                }
            }

            {
                // 位置协方差很大的时候，也不信任速度协方差
                if (gps_data_ptr->lla_cov.block<2, 1>(0, 0).norm() > 10.0) {
                    if (gps_data_ptr->vel_cov.block<2, 1>(0, 0).norm() < 10.0) {
                        gps_data_ptr->vel_cov = Eigen::Vector3d{10.0, 10.0, 10.0};
                    }
                }
            }

            {
                // 弯道场景，速度不可靠
                double scale_         = 1.0 + std::abs(state_ptr->gyro.z()) / (2.0 / 180.0 * M_PI);
                gps_data_ptr->vel_cov = scale_ * gps_data_ptr->vel_cov;
            }

            {
                // 非固定解场景，看下RTK与DR的一致性
                if (std::fabs(diffinfo.timestamp - gps_data_ptr->measurement_timestamp) < RtkCrossValidation::CROSS_VALID_DT_ && gps_data_ptr->status != 6) {
                    gps_data_ptr->lla_cov = gps_data_ptr->lla_cov + 2.0 * std::fabs(diffinfo.gnss_dr_diff_index) * Eigen::Vector3d::Ones();
                }
            }

            {
                constexpr int64_t SWITCH_RANGE = 50;

                if (process_control_sgt.fusion_status.rtk_status_maintain_count < SWITCH_RANGE) {
                    // RTK状态切换
                    // 这种情况往往对应RTK偏差有系统性变化
                    // 如果有视觉融合，临时降低RTK的权重，以应对这种场景
                    uint64_t switch_range_ramain     = SWITCH_RANGE - process_control_sgt.fusion_status.rtk_status_maintain_count;
                    double   switch_range_additinal_ = (double)switch_range_ramain / (double)SWITCH_RANGE;
                    double   dp_                     = 0.4;
                    double   dv_                     = 2.0;
                    double   additional_dp_          = 0.0;
                    double   additional_dv_          = 0.0;
                    if (gps_data_ptr->status == parameters_sgt.FIX) {
                        dp_ = 0.4;
                        dv_ = 0.8;
                    } else {
                        dp_            = 2.0;
                        dv_            = 3.0;
                        additional_dp_ = 1.0;
                        additional_dv_ = 1.0;
                    }

                    AINFO_EVERY(50) << "RTK state switch, std additional: "
                                    << (dp_ * switch_range_additinal_ + additional_dp_)
                                    << ", "
                                    << (dv_ * switch_range_additinal_ + additional_dv_);
                    gps_data_ptr->lla_cov = gps_data_ptr->lla_cov + ((dp_ * switch_range_additinal_ + additional_dp_) * Eigen::Vector3d::Ones());
                    gps_data_ptr->vel_cov = gps_data_ptr->vel_cov + ((dv_ * switch_range_additinal_ + additional_dv_) * Eigen::Vector3d::Ones());
                }
            }
        }

        {
            // 做一个RTK非固定状态的剔除策略
            // 如果有Vision信息，且横向修正量较小。
            // 则认为可以依赖VF维持精度，对非固定解状态的卫星量测，降低权重
            if (std::fabs(state_ptr->vis_t_inno_mean_std_(0) - gps_data_ptr->measurement_timestamp) < 3.0 && std::fabs(state_ptr->vis_inno_(0)) < 2.0) {
                if (
                    gps_data_ptr->status != parameters_sgt.FIX && // 非固定解状态
                    state_ptr->vis_t_inno_mean_std_(1) < 0.2 &&   // 横向VF均值小于20cm
                    state_ptr->vis_t_inno_mean_std_(4) < 0.1      // 横向VF标准差小于10cm
                ) {
                    AINFO_EVERY(30) << "RTK's bad while VF's good, decrease RTK, info: "
                                    << state_ptr->vis_t_inno_mean_std_(1)
                                    << ", "
                                    << state_ptr->vis_t_inno_mean_std_(4);
                    gps_data_ptr->lla_cov = gps_data_ptr->lla_cov + 40.0 * Eigen::Vector3d::Ones();
                    gps_data_ptr->vel_cov = gps_data_ptr->vel_cov + 20.0 * Eigen::Vector3d::Ones();
                }
            }
        }

        {
            if (state_ptr->rtk_status != parameters_sgt.FIX) {
                double dp_additional   = ssi_.get_state_stability_index_() / 10.0;
                double dv_additional   = ssi_.get_state_stability_index_() / 20.0;
                double dhdg_additional = ssi_.get_state_stability_index_() / 3200.0;
                if (ssi_.get_state_stability_index_ratio_() > 0.02 &&                                                            //
                    std::fabs(diffinfo.timestamp - gps_data_ptr->measurement_timestamp) < RtkCrossValidation::CROSS_VALID_DT_ && //
                    state_ptr->vel.norm() > 3.0 &&                                                                               //
                    std::fabs(diffinfo.gnss_dr_diff_index) < 0.2                                                                 //
                ) {
                    double gnss_pos_invalid_index_ = 20.0 + std::fabs(diffinfo.gnss_dr_diff_index) * 30.0 + std::fabs(diffinfo.cross_diff_index) / 2.0;
                    double gnss_vel_invalid_index_ = 10.0 + std::fabs(diffinfo.gnss_dr_diff_index) * 10.0 + std::fabs(diffinfo.cross_diff_index) / 2.0;

                    dp_additional = dp_additional > gnss_pos_invalid_index_ ? gnss_pos_invalid_index_ : dp_additional;
                    dv_additional = dv_additional > gnss_vel_invalid_index_ ? gnss_vel_invalid_index_ : dv_additional;

                    if (std::fabs(pos_inno_statistic_info_.first - gps_data_ptr->measurement_timestamp) < 3.0 && //
                        pos_inno_statistic_info_.second.x() < 2.0 &&                                             //
                        pos_inno_statistic_info_.second.y() < 0.2                                                //
                    ) {
                        // 这里，额外针对新息稳定的卫星量测，放宽限制
                        dp_additional = dp_additional > 10.0 ? 10.0 : dp_additional;
                        dv_additional = dv_additional > 4.0 ? 4.0 : dv_additional;
                    }

                } else {
                    dp_additional = dp_additional > 100.0 ? 100.0 : dp_additional;
                    dv_additional = dv_additional > 30.0 ? 30.0 : dv_additional;
                }
                gps_data_ptr->lla_cov = gps_data_ptr->lla_cov + dp_additional * Eigen::Vector3d::Ones();
                gps_data_ptr->vel_cov = gps_data_ptr->vel_cov + dv_additional * Eigen::Vector3d::Ones();
                gps_data_ptr->hdg_cov = gps_data_ptr->hdg_cov + dhdg_additional;
                AINFO_EVERY(100) << "SSI info(not fix), std additional: " << dp_additional << ", " << dv_additional << ", " << dhdg_additional;
            } else {
                double dp_additional   = ssi_.get_state_stability_index_() / 60000.0;
                double dv_additional   = ssi_.get_state_stability_index_() / 60000.0;
                double dhdg_additional = ssi_.get_state_stability_index_() / 32000.0;
                gps_data_ptr->lla_cov  = gps_data_ptr->lla_cov + dp_additional * Eigen::Vector3d::Ones();
                gps_data_ptr->vel_cov  = gps_data_ptr->vel_cov + dv_additional * Eigen::Vector3d::Ones();
                gps_data_ptr->hdg_cov  = gps_data_ptr->hdg_cov + dhdg_additional;
                AINFO_EVERY(1000) << "SSI info(fix), std additional: " << dp_additional << ", " << dv_additional << ", " << dhdg_additional;
            }
        }

        // 源消息中带的是标准差，这里转换成方差
        gps_data_ptr->lla_cov = gps_data_ptr->lla_cov.cwiseAbs2();
        gps_data_ptr->vel_cov = gps_data_ptr->vel_cov.cwiseAbs2();
        gps_data_ptr->hdg_cov = (gps_data_ptr->hdg_cov / 180.0 * M_PI) * (gps_data_ptr->hdg_cov / 180.0 * M_PI);
    }
}

// 量测抗差处理的时候，会导致位置新息在两个维度的分量上产生显著的不一致
// 这种在有较大误差的时候，修正过程的曲线会出现一些明显的扭曲
// 为了消除这种现象，如果新息很大，在做修正的时候，进行线性化的平衡
Eigen::Vector3d GpsProcessor::positionUpdateLinearBalance(const Eigen::Vector3d &pos_inno_, const Eigen::Vector3d &pos_update_, const INS::EARTH &earth_) {

    double pos_inno_distance_ = std::sqrt(std::pow(pos_inno_.x() * earth_.RMh, 2.0) + std::pow(pos_inno_.y() * earth_.clRNh, 2.0));

    static constexpr double DISTANCE_BOUND_     = 3.0;
    static constexpr double UPDATE_RATIO_BOUND_ = 0.9;

    if (pos_inno_distance_ < DISTANCE_BOUND_ || std::fabs(pos_inno_.x()) < 1e-12 || std::fabs(pos_inno_.y()) < 1e-12) {
        return pos_update_;
    } else {
        double r1_ = pos_update_.x() / pos_inno_.x();
        double r2_ = pos_update_.y() / pos_inno_.y();

        r1_ = r1_ > UPDATE_RATIO_BOUND_ ? UPDATE_RATIO_BOUND_ : r1_;
        r2_ = r2_ > UPDATE_RATIO_BOUND_ ? UPDATE_RATIO_BOUND_ : r2_;
        r1_ = r1_ < -UPDATE_RATIO_BOUND_ ? -UPDATE_RATIO_BOUND_ : r1_;
        r2_ = r2_ < -UPDATE_RATIO_BOUND_ ? -UPDATE_RATIO_BOUND_ : r2_;

        return pos_inno_ * (r1_ + r2_) / 2.0;
    }
}

void GnssFusionControl::reset_state_control(const GnssDataPtr gps_data_ptr, const StatePtr state_ptr, bool is_rtk_fix_valid) {
    Eigen::Vector3d pos_inno = state_ptr->lla + (state_ptr->MpvCnb * state_ptr->C_imu2vehicle * state_ptr->lever_imu2gnss) - gps_data_ptr->lla;

    // TODO:
    // 考虑出隧道场景，迅速重置位置误差
    // TODO:
    // 考虑一类长期无FIX解场景，在FIX时迅速重置位置误差
    bool reset_ =
        (process_control_sgt.fusion_status.imu_update_count_after_rtk_fix > 2000 &&             //
         process_control_sgt.fusion_status.imu_update_count_after_vision_fusion_update < 300 && //
         gps_data_ptr->status == Parameters::GNSS_STATUS::FIX &&
         std::sqrt(gps_data_ptr->lla_cov.norm()) < 0.8) ||
        (process_control_sgt.fusion_status.imu_update_count_after_rtk_fix > 800 &&                                  //
         std::abs(gps_data_ptr->hdg + (state_ptr->eulr_.z() - state_ptr->vehicle_bias.z())) < 3.0 / 180.0 * M_PI && //
         gps_data_ptr->status == Parameters::GNSS_STATUS::FIX &&
         std::sqrt(gps_data_ptr->lla_cov.norm()) < 0.8);

    if (reset_) {
        Eigen::Vector3d dp_rfu = local_trans.DPos_LLAtoEgo(state_ptr->lla, pos_inno, state_ptr->att);
        if (process_control_sgt.fusion_status.rtk_status_maintain_count < 5) {
            // 如果状态无法维持5帧，不重置
            return;
        }
        if (std::abs(dp_rfu.y()) < 0.5 &&
            process_control_sgt.fusion_status.rtk_status_maintain_count < 50 &&
            dp_rfu.block<2, 1>(0, 0).norm() < 3.0) {
            // 如果纵向误差很小，RTK状态维持时间短，适当放宽重置
            return;
        }
        if (dp_rfu.block<2, 1>(0, 0).norm() > 2.0) {
            AINFO << "reset pos to RTK-fix position\n"                                                                 //
                  << std::setprecision(14)                                                                             //
                  << "lla: " << state_ptr->lla.x() * 180.0 / M_PI << ", " << state_ptr->lla.y() * 180.0 / M_PI << "\n" //
                  << "timestamp: " << state_ptr->measurement_timestamp;

            Eigen::Vector3d new_lla_ = gps_data_ptr->lla - (state_ptr->MpvCnb * state_ptr->C_imu2vehicle * state_ptr->lever_imu2gnss);
            if (process_control_sgt.fusion_status.imu_update_count_after_vision_fusion_update < 100) {
                // 在有视觉融合的时候
                // 这里在重置位置的时候，同时尝试消去制图误差
                state_ptr->mapBiasAblationWhenReset(new_lla_);
            }
            state_ptr->lla = new_lla_;
            if (!process_control_sgt.reinitialization_state.heading && !process_control_sgt.reinitialization_state.velocity) {
                // 如果系统重置了航向或速度，则这个标志位不清零
                process_control_sgt.fusion_status.imu_update_count_aftr_pos_reset = 0;
            }
            return;
        }
    }
}

void RtkStatusAnalysis::rtk_overall_state_analysis(const GnssDataPtr gps_data_ptr, const StatePtr state_ptr) {

    // RTK 状态统计分析

    if (state_ptr->vel.norm() > parameters_sgt.get_slow_speed_bound() && // 只统计运动时的RTK状态
        (gps_data_ptr->status == (uint64_t)Parameters::FIX ||            // 只考虑这四种状态
         gps_data_ptr->status == (uint64_t)Parameters::FLOAT ||          //
         gps_data_ptr->status == (uint64_t)Parameters::DGPS ||           //
         gps_data_ptr->status == (uint64_t)Parameters::SINGLE)) {
        rtk_state_statistic.insert_per_sec({gps_data_ptr->measurement_timestamp, (int)gps_data_ptr->status});
    }

    // 依据STATE_SEQ_MAX_SIZE的采样统计，分析RTK整体状态

    uint64_t samples = rtk_state_statistic.total_state_count();
    if (samples < STATE_SEQ_MAX_SIZE) {
        return;
    }

    double fix_count    = rtk_state_statistic.state_count((uint64_t)Parameters::FIX);
    double float_count  = rtk_state_statistic.state_count((uint64_t)Parameters::FLOAT);
    double dgps_count   = rtk_state_statistic.state_count((uint64_t)Parameters::DGPS);
    double single_count = rtk_state_statistic.state_count((uint64_t)Parameters::SINGLE);

    // fmt::print("RTK state count: {:>5} {:>5} {:>5} {:>5} \n", fix_count, float_count, dgps_count, single_count);

    double rtk_count  = fix_count + float_count;
    double gnss_count = dgps_count + single_count;

    // 正常状态，预期80%以上的FIX率。默认这个状态
    if (fix_count / samples > 0.6) {
        process_control_sgt.rtk_overall_status = ProcessControl::MAJORITY_FIX;
        AINFO_EVERY(1000) << "RTK overall status: MAJORITY_FIX";
        return;
    }

    // 部分正常，预期40%以上的FIX率，FIX和FLOAT占比超过60%，看起来RTK本身是OK的，只是遮挡比较多
    if (rtk_count / samples > 0.6 && fix_count / samples > 0.4) {
        process_control_sgt.rtk_overall_status = ProcessControl::MEDIUM_FIX;
        AINFO_EVERY(1000) << "RTK overall status: MEDIUM_FIX";
        return;
    }

    // 亚稳状态，FIX和FLOAT占比超过80%，但是60%以上FLOAT解
    if (rtk_count / samples > 0.8 && float_count / samples > 0.6) {
        process_control_sgt.rtk_overall_status = ProcessControl::MAJORITY_FLOAT;
        AINFO_EVERY(1000) << "RTK overall status: MAJORITY_FLOAT";
        return;
    }

    // 非稳状态，FIX和FLOAT占比超过20%，但是达不到上述其他状态
    if (rtk_count / samples > 0.2) {
        process_control_sgt.rtk_overall_status = ProcessControl::UNSTABLE;
        AINFO_EVERY(1000) << "RTK overall status: UNSTABLE";
        return;
    }

    process_control_sgt.rtk_overall_status = ProcessControl::BAD;
    AINFO_EVERY(1000) << "RTK overall status: BAD";
    // 异常状态，RTK难以得到FIX或者FLOAT解，FIX和FLOAT占比低于20%
    return;
}

bool PositionCompensation::ego_position_compensation(const StatePtr state_ptr, const Eigen::Vector3d &pos_inno) {

    Eigen::Vector3d dp_rfu = local_trans.DPos_LLAtoEgo(state_ptr->lla, pos_inno, state_ptr->att);

    bool clear_buffer_ =
        // process_control_sgt.msf_align_type == process_control_sgt.FINE_ALIGN ||
        process_control_sgt.msf_align_type == process_control_sgt.ALIGNED ||
        state_ptr->vel.norm() < parameters_sgt.get_slow_speed_bound() * 2.0;

    if (clear_buffer_) {
        ego_pos_inno_statistic.clear();
        return false;
    }

    ego_pos_inno_statistic.insert(dp_rfu);
    if (!ego_pos_inno_statistic.calc_mean_std()) {
        return false;
    }

    { // map bias transfer

        if (process_control_sgt.fusion_status.imu_update_count_after_vision_fusion_update < 30) {

            bool low_dynamic_state_ =
                std::abs(state_ptr->slip_index) < parameters_sgt.get_slip_index_rejection_bound(); // 侧滑系数小于阈值

            bool should_lat_map_bias_ablation =
                std::abs(dp_rfu.x() - ego_pos_inno_statistic.mean().x()) < 0.2 &&  // 横向误差未跳变
                ego_pos_inno_statistic.std().x() < 0.2 &&                          // 横向误差稳定
                std::abs(ego_pos_inno_statistic.mean().x()) > 0.2 &&               // 非偶然性横向误差
                ego_pos_inno_statistic.mean().x() * state_ptr->map_bias.x() < 0.0; // 制图误差与卫星新息方向不一致

            if (should_lat_map_bias_ablation && low_dynamic_state_) {
                constexpr double cmp_ratio_         = 1.0 / 6.0;
                constexpr double max_cmp_           = 0.05;
                Eigen::Vector3d  map_bias_ablation_ = Eigen::Vector3d::Zero();
                double           cmp_x_             = state_ptr->map_bias.x() * cmp_ratio_;
                if (cmp_x_ > max_cmp_) {
                    cmp_x_ = max_cmp_;
                } else if (cmp_x_ < -max_cmp_) {
                    cmp_x_ = -max_cmp_;
                }
                state_ptr->map_bias.x() = state_ptr->map_bias.x() - cmp_x_;
                map_bias_ablation_.x()  = cmp_x_;

                Eigen::Vector3d lla_ = local_trans.DPos_Ego2LLA(state_ptr->lla, map_bias_ablation_, state_ptr->att);
                state_ptr->lla       = lla_;
                AINFO_EVERY(20) << "map bias ablation RFU: " << map_bias_ablation_.transpose();
            }

            // 这里如果RTK有系统性偏差的话，会导致制图误差错误的持续增大，注释掉不启用
            // bool should_lat_map_bias_transfer =
            //     std::abs(dp_rfu.x() - ego_pos_inno_statistic.mean().x()) < 0.2 &&                                               // 横向误差未跳变
            //     ego_pos_inno_statistic.std().x() < 0.2 &&                                                                       // 横向误差稳定
            //     std::abs(ego_pos_inno_statistic.mean().x()) > 0.2 &&                                                            // 非偶然性横向误差
            //     (std::abs(state_ptr->map_bias.x()) < 0.1 || ego_pos_inno_statistic.mean().x() * state_ptr->map_bias.x() > 0.0); // 制图误差较小或者与卫星新息方向一致

            // if (should_lat_map_bias_transfer && low_dynamic_state_) {
            //     constexpr double cmp_ratio_         = 1.0 / 6.0;
            //     constexpr double max_cmp_           = 0.05;
            //     Eigen::Vector3d  map_bias_transfer_ = Eigen::Vector3d::Zero();
            //     double           cmp_x_             = dp_rfu.x() * cmp_ratio_;
            //     if (cmp_x_ > max_cmp_) {
            //         cmp_x_ = max_cmp_;
            //     } else if (cmp_x_ < -max_cmp_) {
            //         cmp_x_ = -max_cmp_;
            //     }
            //     state_ptr->map_bias.x() = state_ptr->map_bias.x() + cmp_x_;
            //     map_bias_transfer_.x()  = -cmp_x_;
            //     Eigen::Vector3d lla_    = local_trans.DPos_Ego2LLA(state_ptr->lla, map_bias_transfer_, state_ptr->att);
            //     state_ptr->lla          = lla_;
            //     AINFO_EVERY(10) << "map bias transfer RFU: " << map_bias_transfer_.transpose();
            // }
        }
    }

    bool should_compensation_ =
        std::abs(ego_pos_inno_statistic.mean().x()) < 30.0 &&                                                          // 横向误差小于30米
        std::abs(dp_rfu.x() - ego_pos_inno_statistic.mean().x()) < parameters_sgt.get_ego_pos_inno_lat_diff_bound() && // 横向误差未跳变
        ego_pos_inno_statistic.std().x() < parameters_sgt.get_ego_pos_inno_lat_std_bound() &&                          // 横向误差稳定
        std::abs(dp_rfu.y()) > parameters_sgt.get_ego_pos_inno_lon_bound() &&                                          // 纵向有较大的误差
        std::abs(ego_pos_inno_statistic.mean().y()) > parameters_sgt.get_ego_pos_inno_lon_mean_bound() &&              // 非偶然性纵向误差
        ego_pos_inno_statistic.std().y() < parameters_sgt.get_ego_pos_inno_lon_std_bound() &&                          // 稳定的纵向误差
        (std::abs(state_ptr->gyro.z()) < 1.5 / 180.0 * M_PI);                                                          // 车辆处于直行状态
    if (should_compensation_) {
        should_compensation_count_++;
    } else {
        should_compensation_count_ = 0;
    }

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
    std::string str_ =
        fmt::format("{:14.4f},{:6.3f},{:6.3f},{:6.3f},{:6.3f},{:6.3f},{:d},{:6.3f}\n", //
                    state_ptr->measurement_timestamp,                                  // 1
                    std::abs(dp_rfu.x() - ego_pos_inno_statistic.mean().x()),          // 2
                    ego_pos_inno_statistic.std().x(),                                  // 3
                    std::abs(dp_rfu.y()),                                              // 4
                    std::abs(ego_pos_inno_statistic.mean().y()),                       // 5
                    ego_pos_inno_statistic.std().y(),                                  // 6
                    should_compensation_,                                              // 7
                    std::abs(state_ptr->gyro.z())                                      // 8

        );
    debug::debug_sgt.lpc_state.line(str_);
#endif

    // 顺带补偿天向误差
    bool should_compensation_hgt_ =
        std::abs(dp_rfu.z()) > parameters_sgt.get_ego_pos_inno_hgt_bound() &&                             // 天向有较大的误差
        std::abs(ego_pos_inno_statistic.mean().z()) > parameters_sgt.get_ego_pos_inno_hgt_mean_bound() && // 非偶然性天向误差
        ego_pos_inno_statistic.std().z() < parameters_sgt.get_ego_pos_inno_hgt_std_bound();               // 稳定的天向误差

    if (should_compensation_count_ >= 1 &&
        (longi_compe_count_ < long_distance_outage_longi_compe_max_count_ || continous_dynamic_gnss_good_inno_ > 100)) {

        double ego_lon_cmp_ = std::abs(dp_rfu.y() / 4.0);
        if (ego_lon_cmp_ > 0.6) {
            // 限制单次补偿的量
            ego_lon_cmp_ = 0.6;
        }

        Eigen::Vector3d dp_ref_comp = {0.0, ego_lon_cmp_, 0.0};
        if (dp_rfu.y() < 0.0) {
            dp_ref_comp.y() = -dp_ref_comp.y();
        }

        double ego_hgt_cmp_ = std::abs(dp_rfu.z() / 4.0);
        if (ego_hgt_cmp_ > 1.0) {
            ego_hgt_cmp_ = 1.0;
        }

        if (should_compensation_hgt_) {
            dp_ref_comp.z() = ego_hgt_cmp_;
            if (dp_rfu.z() < 0.0) {
                dp_ref_comp.z() = -dp_ref_comp.z();
            }
        }
        Eigen::Vector3d lla_ = local_trans.DPos_Ego2LLA(state_ptr->lla, -dp_ref_comp, state_ptr->att);
        state_ptr->lla       = lla_;
        AINFO_EVERY(20) << "ego position compensation RFU: " << -dp_ref_comp.transpose();
        return true;
    }
    return false;
}

void GnssFusionControl::msf_align_type_control(const GnssDataPtr gps_data_ptr, const StatePtr state_ptr, const Eigen::Matrix<double, 8, 1> &inno_) {
    bool dynamic_state_            = state_ptr->vel.norm() > parameters_sgt.get_slow_speed_bound();
    bool inno_situation_rtk_fix_   = gps_data_ptr->status == Parameters::GNSS_STATUS::FIX;
    bool inno_situation_rtk_float_ = (gps_data_ptr->status == Parameters::GNSS_STATUS::FIX || gps_data_ptr->status == Parameters::GNSS_STATUS::FLOAT) &&
                                     (process_control_sgt.rtk_overall_status == process_control_sgt.MAJORITY_FLOAT);

    // 这里额外考虑低速场景下，融合定位的状态
    {
        // 认为当前BEVLANE和LD是匹配的
        // 横向30cm，航向1度
        static constexpr double VIS_GOOD_POS_INNO_BOUND_ = 0.3;
        static constexpr double VIS_GOOD_HDG_INNO_BOUND_ = 1.0 / 180.0 * M_PI;

        // 认为融合定位位置是好的
        // 平面20cm
        static constexpr double MSF_GOOD_POS_INNO_BOUND_ = 0.2;

        // 认为视觉是比较准的条件
        bool vis_may_good_ =
            std::fabs(process_control_sgt.state_nearby_info.vision_inno_info.vis_timestamp - state_ptr->measurement_timestamp) < 1.0 && // 时效性限制
            process_control_sgt.state_nearby_info.vision_inno_info.vis_count > 10 &&                                                    // 统计数量限制
            std::fabs(process_control_sgt.state_nearby_info.vision_inno_info.pos_mean_r) < VIS_GOOD_POS_INNO_BOUND_ &&                  // 横向误差伪均值限制
            std::fabs(process_control_sgt.state_nearby_info.vision_inno_info.hdg_mean) < VIS_GOOD_HDG_INNO_BOUND_;                      // 航向误差伪均值限制

        // 认为融合位置是比较准的条件
        bool msf_position_may_good_ =
            gps_data_ptr->status == Parameters::GNSS_STATUS::FIX &&                                                                   // 当前固定解
            process_control_sgt.fusion_status.continous_gnss_fusion_count > 5 &&                                                      // 连续固定解计数大于5
            std::fabs(process_control_sgt.state_nearby_info.pose_inno_info.pos_timestamp - state_ptr->measurement_timestamp) < 0.5 && // 时效性限制
            process_control_sgt.state_nearby_info.pose_inno_info.pos_count > 20 &&                                                    // 统计数量限制
            process_control_sgt.state_nearby_info.pose_inno_info.pos_mean_rfu.block<2, 1>(0, 0).norm() < MSF_GOOD_POS_INNO_BOUND_;    // 位置新息精度限制

        // 低速场景下。如果各方面认为定位比较准，但是状态给粗对准的话，认为粗对准不合理，变为精对准状态
        if (vis_may_good_ && msf_position_may_good_ && !dynamic_state_) {
            if (process_control_sgt.msf_align_type == process_control_sgt.COARSE_ALIGN) {
                AINFO << "low speed, align state switch to FINE_ALIGN";
                process_control_sgt.msf_align_type = process_control_sgt.FINE_ALIGN;
            }
        }
    }

    // 这里专门判断下低速场景下的位置是否需要重置
    // 但是也过滤下速度过于低的场景
    if ((inno_situation_rtk_fix_ || inno_situation_rtk_float_) && !dynamic_state_ && state_ptr->vel.norm() > 0.1) {
        pos_inno_count++;
        if (pos_inno_count % POS_INNO_BUFFER_SKIP == 0) {
            pos_inno_buffer.push_back(inno_.block<4, 1>(0, 0));
            if (pos_inno_buffer.size() >= POS_INNO_BUFFER_SIZE) {
                pos_inno_buffer.pop_front();
                Eigen::Vector3d pos_inno_mean = Eigen::Vector3d::Zero();
                Eigen::Vector3d pos_inno_std  = Eigen::Vector3d::Zero();
                for (auto &inn : pos_inno_buffer) {
                    pos_inno_mean += inn.block<3, 1>(1, 0);
                }
                pos_inno_mean /= POS_INNO_BUFFER_SIZE;

                for (auto &inn : pos_inno_buffer) {
                    pos_inno_std += (inn.block<3, 1>(1, 0) - pos_inno_mean).array().square().matrix();
                }
                pos_inno_std = (pos_inno_std / POS_INNO_BUFFER_SIZE).cwiseSqrt();

                if (inno_situation_rtk_fix_) { // 退化到重新初始化
                    if (pos_inno_mean.block<2, 1>(0, 0).lpNorm<1>() > parameters_sgt.get_gnss_minimum_position_innovation_for_initialization()) {
                        AINFO << "reinitialization for position inno too big [low speed]";
                        process_control_sgt.reinitialization_state.position = true;

                        pos_inno_buffer.clear();
                        // 如果新息均值异常，则需要重新初始化
                        process_control_sgt.msf_align_type = process_control_sgt.INITIALIZATION;
                    }
                }
            }
        }
    }

    // 依据速度、位置、航向新息 判断是否完成精对准
    // inno 3维位置 3维速度 1维航向
    if ((inno_situation_rtk_fix_ || inno_situation_rtk_float_) && dynamic_state_) {
        inno_count++;
        if (inno_count % INNO_BUFFER_SKIP == 0) {
            inno_buffer.push_back(inno_);
            if (inno_buffer.size() >= INNO_BUFFER_SIZE) {
                inno_buffer.pop_front();
                Eigen::Matrix<double, 7, 1> inno_mean = Eigen::Matrix<double, 7, 1>::Zero();
                Eigen::Matrix<double, 7, 1> inno_std  = Eigen::Matrix<double, 7, 1>::Zero();
                for (auto &inn : inno_buffer) {
                    inno_mean += inn.block<7, 1>(1, 0);
                }
                inno_mean /= INNO_BUFFER_SIZE;

                for (auto &inn : inno_buffer) {
                    inno_std += (inn.block<7, 1>(1, 0) - inno_mean).array().square().matrix();
                }
                inno_std = (inno_std / INNO_BUFFER_SIZE).cwiseSqrt();

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
                std::string inno_str = "";
                inno_str += fmt::format("{:>14.4f},", gps_data_ptr->measurement_timestamp); // 1
                inno_str += fmt::format("{:8.5f},{:8.5f},{:8.5f},{:8.5f},{:8.5f},{:8.5f},", //
                                        inno_mean.block<2, 1>(0, 0).lpNorm<1>(),            // 2
                                        inno_mean.block<2, 1>(3, 0).lpNorm<1>(),            // 3
                                        inno_mean.block<1, 1>(6, 0).x() * 180 / M_PI,       // 4
                                        inno_std.block<2, 1>(0, 0).lpNorm<1>(),             // 5
                                        inno_std.block<2, 1>(3, 0).lpNorm<1>(),             // 6
                                        inno_std.block<1, 1>(6, 0).x() * 180 / M_PI         // 7
                );                                                                          //
                inno_str += fmt::format("{}\n", (int)process_control_sgt.msf_align_type);   //
                debug::debug_sgt.alg_state.line(inno_str);
#endif
                // GNSS纵向不太靠谱，只考虑平面状态
                // 完成粗对准，进入精对准状态
                if (state_ptr->vel.lpNorm<1>() > parameters_sgt.get_slow_speed_bound() &&
                    inno_mean.block<2, 1>(0, 0).lpNorm<1>() < parameters_sgt.get_gnss_maximum_position_innovation_for_coarse_align() &&
                    inno_mean.block<2, 1>(3, 0).lpNorm<1>() < parameters_sgt.get_gnss_maximum_speed_innovation_for_coarse_align() &&
                    inno_mean.block<1, 1>(6, 0).lpNorm<1>() < parameters_sgt.get_gnss_maximum_heading_innovation_for_coarse_align() &&
                    inno_std.block<2, 1>(0, 0).lpNorm<1>() < parameters_sgt.get_gnss_maximum_position_innovation_for_coarse_align_std() &&
                    inno_std.block<2, 1>(3, 0).lpNorm<1>() < parameters_sgt.get_gnss_maximum_speed_innovation_for_coarse_align_std() &&
                    inno_std.block<1, 1>(6, 0).lpNorm<1>() < parameters_sgt.get_gnss_maximum_heading_innovation_for_coarse_align_std() &&
                    std::abs(inno_buffer.front()(0) - inno_buffer.back()(0)) < (INNO_BUFFER_SKIP * INNO_BUFFER_SIZE * parameters_sgt.get_gnss_data_refresh_dt() * 5)) {
                    // AINFO << "align state enter FINE ALIGN";
                    process_control_sgt.msf_align_type = process_control_sgt.FINE_ALIGN;
                }

                // 完成精对准，进入已对准状态
                if (state_ptr->vel.lpNorm<1>() > parameters_sgt.get_slow_speed_bound() &&
                    inno_mean.block<2, 1>(0, 0).lpNorm<1>() < parameters_sgt.get_gnss_maximum_position_innovation_for_fine_align() &&
                    inno_mean.block<2, 1>(3, 0).lpNorm<1>() < parameters_sgt.get_gnss_maximum_speed_innovation_for_fine_align() &&
                    inno_mean.block<1, 1>(6, 0).lpNorm<1>() < parameters_sgt.get_gnss_maximum_heading_innovation_for_fine_align() &&
                    inno_std.block<2, 1>(0, 0).lpNorm<1>() < parameters_sgt.get_gnss_maximum_position_innovation_for_fine_align_std() &&
                    inno_std.block<2, 1>(3, 0).lpNorm<1>() < parameters_sgt.get_gnss_maximum_speed_innovation_for_fine_align_std() &&
                    inno_std.block<1, 1>(6, 0).lpNorm<1>() < parameters_sgt.get_gnss_maximum_heading_innovation_for_fine_align_std() &&
                    std::abs(inno_buffer.front()(0) - inno_buffer.back()(0)) < (INNO_BUFFER_SKIP * INNO_BUFFER_SIZE * parameters_sgt.get_gnss_data_refresh_dt() * 5)) {
                    // 如果从非ALIGNED进入ALIGNED，打印一下日志
                    // AINFO_IF(process_control_sgt.msf_align_type != process_control_sgt.ALIGNED) << "align state enter ALIGNED";
                    process_control_sgt.msf_align_type = process_control_sgt.ALIGNED;
                    state_ptr->tcmsf_aligned_count++;
                }

                // 退化到精对准
                if ((inno_mean.block<2, 1>(0, 0).lpNorm<1>() > parameters_sgt.get_gnss_minimum_position_innovation_for_exit_aligned() ||
                     inno_mean.block<2, 1>(3, 0).lpNorm<1>() > parameters_sgt.get_gnss_minimum_speed_innovation_for_exit_aligned() ||
                     inno_std.block<2, 1>(0, 0).lpNorm<1>() > parameters_sgt.get_gnss_minimum_position_innovation_for_exit_aligned_std() ||
                     inno_std.block<2, 1>(3, 0).lpNorm<1>() > parameters_sgt.get_gnss_minimum_speed_innovation_for_exit_aligned_std())) {
                    // 如果从非FINE_ALIGN进入FINE_ALIGN，打印一下日志
                    // AINFO_IF(process_control_sgt.msf_align_type != process_control_sgt.FINE_ALIGN) << "align state enter FINE ALIGN";
                    process_control_sgt.msf_align_type = process_control_sgt.FINE_ALIGN;
                }

                // 退化到粗对准
                // 这里对进入粗对准的条件进行一些优化
                // 1. 不考虑速度因素（遮挡场景速度的可靠性没那么高）
                // 2. 对位置新增一个系数，如果主体是float解，那么进入粗对准的阈值适当提高
                double rtk_status_scale_ = 1.0;
                if (process_control_sgt.rtk_overall_status == process_control_sgt.MAJORITY_FLOAT) {
                    rtk_status_scale_ = 2.0;
                }
                if ((inno_mean.block<2, 1>(0, 0).lpNorm<1>() > rtk_status_scale_ * parameters_sgt.get_gnss_minimum_position_innovation_for_exit_fine_align() ||
                     inno_std.block<2, 1>(0, 0).lpNorm<1>() > rtk_status_scale_ * parameters_sgt.get_gnss_minimum_position_innovation_for_exit_fine_align_std())) {
                    // 如果从非COARSE进入COARSE，打印一下日志
                    AINFO_IF(process_control_sgt.msf_align_type != process_control_sgt.COARSE_ALIGN) << "align state enter COARSE ALIGN";
                    process_control_sgt.msf_align_type = process_control_sgt.COARSE_ALIGN;
                }
                if (inno_situation_rtk_fix_) { // 退化到重新初始化
                    if (inno_mean.block<2, 1>(0, 0).lpNorm<1>() > parameters_sgt.get_gnss_minimum_position_innovation_for_initialization()) {
                        AINFO << "reinitialization for position inno too big";
                        process_control_sgt.reinitialization_state.position = true;
                        if (inno_mean.block<2, 1>(3, 0).lpNorm<1>() > parameters_sgt.get_gnss_minimum_speed_innovation_for_initialization()) {
                            // 如果新息均值异常，则需要重新初始化
                            AINFO << "need to reinit velocity";
                            process_control_sgt.reinitialization_state.velocity = true;
                        }

                        if (inno_mean.block<1, 1>(6, 0).lpNorm<1>() > parameters_sgt.get_gnss_minimum_heading_innovation_for_initialization() &&
                            inno_situation_rtk_fix_) {
                            // 如果新息均值异常，则需要重新初始化
                            AINFO << "need to reinit heading";
                            process_control_sgt.reinitialization_state.heading = true;
                        }
                        inno_buffer.clear();
                        // 如果新息均值异常，则需要重新初始化
                        process_control_sgt.msf_align_type = process_control_sgt.INITIALIZATION;
                    }
                    // if (inno_mean.block<2, 1>(3, 0).lpNorm<1>() > parameters_sgt.get_gnss_minimum_speed_innovation_for_initialization()) {
                    //     AINFO << "reinitialization for velocity inno too big";
                    //     process_control_sgt.reinitialization_state.position = true;
                    //     process_control_sgt.reinitialization_state.velocity = true;
                    //     process_control_sgt.reinitialization_state.heading  = true;
                    //     inno_buffer.clear();
                    //     // 如果新息均值异常，则需要重新初始化
                    //     process_control_sgt.msf_align_type = process_control_sgt.INITIALIZATION;
                    // }
                }
            }
        }
    }
}

bool PositionCompensation::mapping_bias_ablation_and_compensation(const Eigen::Vector3d &lla0_, Eigen::Vector3d &lla_, const INS::EARTH &earth_, StatePtr state_ptr) {

    Eigen::Vector3d lla_fd_ = lla_ - lla0_;

    Eigen::Vector3d pos_enu_fd_{lla_fd_.y() * earth_.clRNh, lla_fd_.x() * earth_.RMh, lla_fd_.z()};
    Eigen::Vector3d pos_rfu_fd_{state_ptr->C_imu2vehicle * state_ptr->C_b2n.transpose() * pos_enu_fd_};
    Eigen::Vector3d map_bias_rfu_{state_ptr->map_bias.x(), state_ptr->map_bias.y(), 0.0};

    if (true) {
        // 新增策略，如果GNSS的更新与制图误差方向一致，则在进行GNSS量测更新的同时，对制图误差进行一个补偿，以免制图误差来不及消掉
        if (std::abs(map_bias_rfu_.x() - pos_rfu_fd_.x()) < std::abs(map_bias_rfu_.x()) && std::abs(map_bias_rfu_.x()) > 0.1) {
            AINFO_EVERY(20) << "mapbias compensation.";
            state_ptr->map_bias.x() = state_ptr->map_bias.x() - pos_rfu_fd_.x();
        }
    }
    // 这里这个逻辑似乎不是很必要了
    // else {
    //     // 新增策略，如果非fix解的量测更新导致制图误差变大，那么认为这个rtk量测更新是导致制图误差变大的原因
    //     // 则放弃这一次RTK的位置更新
    //     if (std::abs(state_ptr->map_bias.x()) > 0.4 && std::abs(pos_rfu_fd_.x() - map_bias_rfu_.x()) > std::abs(map_bias_rfu_.x())) {
    //         AINFO_EVERY(20) << "abandon current RTK position update.";
    //         lla_ = lla0_;
    //     }
    // }

    return true;
}

bool RtkStatusAnalysis::RTK_false_fix_detect(const GnssDataPtr gps_data_ptr, const StatePtr state_ptr, const Eigen::Vector3d pos_inno_, const Eigen::Vector3d pos_inno_rfu_, double ssi_ratio) {

    { // 刹停过程中，RTK固定解时，如果横向新息比较大，会导致定位出现漂移.
        // 认为这个过程中的固定解是假的
        if (
            state_ptr->vel.norm() < 0.01 //
        ) {
            rffd_mileage_when_stop_ = state_ptr->mileage;
        }

        if (
            gps_data_ptr->status == 6 &&
            std::abs(rffd_mileage_when_stop_ - state_ptr->mileage) < RFFD_MAXIMUM_MILEAGE_AFTER_STOP_ &&
            std::abs(pos_inno_rfu_.x()) > 0.2 //
        ) {
            AINFO_EVERY(30) << "RTK false fix, stopping state";
            return true;
        }
    }

    if (
        ssi_ratio > 0.9 &&                       //
        gps_data_ptr->status == 6 &&             //
        pos_inno_.block<2, 1>(0, 0).norm() > 2.0 //
    ) {
        rffd_persistence_mileage_  = std::abs(rffd_pre_mileage_ - state_ptr->mileage);
        rffd_persistence_duration_ = std::abs(rffd_pre_timestamp_ - state_ptr->measurement_timestamp);
        if (state_ptr->vel.norm() > 1) {
            rffd_dynamic_persistence_duration_ += std::abs(pre_gnss_timestamp_ - gps_data_ptr->measurement_timestamp);
        }
        pre_gnss_timestamp_ = gps_data_ptr->measurement_timestamp;
        if (
            rffd_persistence_mileage_ < RFFD_MAXIMUM_MILEAGE_ &&                   //
            rffd_dynamic_persistence_duration_ < RFFD_MAXIMUM_DYNAMIC_DURATION_ && //
            rffd_persistence_duration_ < RFFD_MAXIMUM_DURATION_                    //
        ) {
            AINFO_EVERY(30) << "RTK false fix, moving state " << rffd_persistence_mileage_ << ", " << rffd_dynamic_persistence_duration_ << ", " << rffd_persistence_duration_;
            return true;
        } else {
            return false;
        }
    } else {
        rffd_pre_mileage_   = state_ptr->mileage;
        rffd_pre_timestamp_ = state_ptr->measurement_timestamp;
        pre_gnss_timestamp_ = gps_data_ptr->measurement_timestamp;
        return false;
    }
}

void RtkCrossValidation::insert(const GnssDataPtr gps_data_ptr_, const StatePtr state_ptr_gpst_, const KinematicDataPtr dr_ptr_gpst_) {

    msg_count++;

    if (msg_count % GNSS_MSG_SKIP == 0) {
        NaviInfo info_;

        info_.timestamp = gps_data_ptr_->measurement_timestamp;
        info_.GnssPos   = gps_data_ptr_->lla;
        info_.DrPos     = dr_ptr_gpst_->pos;
        info_.MsfPos    = state_ptr_gpst_->lla + state_ptr_gpst_->MpvCnb * state_ptr_gpst_->C_imu2vehicle * state_ptr_gpst_->lever_imu2gnss;
        info_.DrQuat    = dr_ptr_gpst_->att;
        info_.MsfQuat   = state_ptr_gpst_->att;
        info_.DrVel     = dr_ptr_gpst_->vel;
        navi_info_buffer.push_back(info_);
    }
    if (navi_info_buffer.size() >= BUFFER_SIZE) {

        auto front = navi_info_buffer.front();

        Eigen::Matrix3d FLU2RFU  = (Eigen::Matrix3d() << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();
        Eigen::Matrix3d dr_trans = FLU2RFU * front.DrQuat.toRotationMatrix().transpose();
        // fmt::print("-------------------------------------\n");
        for (int i = 1; i < BUFFER_SIZE; i++) {
            auto &cur          = navi_info_buffer.at(i);
            cur.GnssRfu        = local_trans.LLAtoEgoRfu(front.GnssPos, cur.GnssPos, front.MsfQuat);
            cur.DrRfu          = dr_trans * (cur.DrPos - front.DrPos);
            cur.DrDeltaRfu     = cur.DrRfu - front.DrRfu;
            cur.GnssDeltaRfu   = cur.GnssRfu - front.GnssRfu;
            cur.GnssMsfDiffRfu = local_trans.LLAtoEgoRfu(cur.MsfPos, cur.GnssPos, cur.MsfQuat);
            // fmt::print("  DrDeltaRfu: {:>6.3f},{:>6.3f},{:>6.3f}\n", cur.DrDeltaRfu.x(), cur.DrDeltaRfu.y(), cur.DrDeltaRfu.z());
            // fmt::print("GnssDeltaRfu: {:>6.3f},{:>6.3f},{:>6.3f}\n", cur.GnssDeltaRfu.x(), cur.GnssDeltaRfu.y(), cur.GnssDeltaRfu.z());
        }

        CurNaviInfo = std::make_pair(true, navi_info_buffer.back());

        {
            Eigen::Vector3d dr_gnss_diff_abs_mean  = Eigen::Vector3d::Zero();
            Eigen::Vector3d msf_gnss_diff_abs_mean = Eigen::Vector3d::Zero();
            Eigen::Vector3d cross_diff_abs_mean    = Eigen::Vector3d::Zero();
            for (int i = 1; i < BUFFER_SIZE; i++) {
                auto &cur              = navi_info_buffer.at(i);
                dr_gnss_diff_abs_mean  = dr_gnss_diff_abs_mean + (cur.DrDeltaRfu - cur.GnssDeltaRfu).cwiseAbs();
                msf_gnss_diff_abs_mean = msf_gnss_diff_abs_mean + cur.GnssMsfDiffRfu.cwiseAbs();
                cross_diff_abs_mean    = cross_diff_abs_mean + (cur.GnssMsfDiffRfu + (cur.DrDeltaRfu - cur.GnssDeltaRfu)).cwiseAbs();
            }
            dr_gnss_diff_abs_mean  = dr_gnss_diff_abs_mean / (BUFFER_SIZE - 1.0);
            msf_gnss_diff_abs_mean = msf_gnss_diff_abs_mean / (BUFFER_SIZE - 1.0);
            cross_diff_abs_mean    = cross_diff_abs_mean / (BUFFER_SIZE - 1.0);
            double gnss_           = 2.0 * dr_gnss_diff_abs_mean.x() + dr_gnss_diff_abs_mean.y() + 0.5 * dr_gnss_diff_abs_mean.z();
            double msf_            = 2.0 * msf_gnss_diff_abs_mean.x() + msf_gnss_diff_abs_mean.y() + 0.5 * msf_gnss_diff_abs_mean.z();
            double cross_          = 2.0 * cross_diff_abs_mean.x() + cross_diff_abs_mean.y() + 0.5 * cross_diff_abs_mean.z();

            diffinfo.timestamp           = gps_data_ptr_->measurement_timestamp;
            diffinfo.gnss_dr_diff_index  = gnss_;
            diffinfo.msf_gnss_diff_index = msf_;
            diffinfo.cross_diff_index    = cross_;
        }
        navi_info_buffer.clear();
    } else {
        CurNaviInfo.first = false;
    }
}

} // namespace MSF