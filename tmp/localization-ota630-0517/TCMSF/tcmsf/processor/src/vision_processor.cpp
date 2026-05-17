#include "vision_processor.h"

#include "calc.h"

#include "cyber/common/log.h"
#include "fmt/format.h"
#include "gps_processor.h"
#include "rigid_transform.h"

#include "processor_debug.h"
#include <iomanip>
#include <limits>

#include "Coord.h"
namespace MSF {

bool VisionProcessor::UpdateStateByVision(const VisionFusionDataPtr vis_data_ptr, StatePtr state_ptr, KfPtr<21, 3> kf_ptr) {

    // 每200个打印一个心跳信息
    AINFO_EVERY(200) << "[HeartBeat] Vision Fusion, TS: " << fmt::format("{:14.4f}", vis_data_ptr->measurement_timestamp);

    update_frequency(pre_timestamp, vis_data_ptr->measurement_timestamp);
    // AINFO_EVERY(100) << "vision gap time: " << vis_data_ptr->measurement_timestamp - pre_timestamp;

    AINFO_EVERY(200) << "vision info frequency stage: " << cur_freq;
    // 过滤非连续的VF新息
    // 可以认为非连续的视觉融合结果是不可靠的
    if (std::abs(vis_data_ptr->measurement_timestamp - pre_timestamp) > 5 * parameters_sgt.get_vision_data_refresh_dt()) {
        pre_timestamp = vis_data_ptr->measurement_timestamp;
        AINFO << "drop discontinuous(time) VF info";
        return false;
    } else {
        pre_timestamp = vis_data_ptr->measurement_timestamp;
    }

    if (vis_data_ptr->measurement_timestamp < state_ptr->pos_reset_timestamp) {
        // 剔除位置重置之前的VF结果
        AINFO << "drop VF info before pos reset";
        return false;
    }

    // // 剔除刚起步时刻的VF数据
    // if (std::abs(state_ptr->acc.y()) > 0.5 && state_ptr->vel.norm() < 6.0) {
    //     return false;
    // }

    double speed_related_weights_ = 1.0;
    if (state_ptr->vel.norm() < 4.0) {
        speed_related_weights_ = std::abs(5.0 / (state_ptr->vel.norm() + 1.0));
    }
    double slip_related_weights_ = 0.0;
    if (std::abs(state_ptr->slip_index) > 1.0) {
        slip_related_weights_ = std::abs(state_ptr->slip_index);
    }
    double vision_pos_lat_bias_std_ = vision_pos_lat_bias_std * (speed_related_weights_ + slip_related_weights_ / 3.0) + vis_data_ptr->pos_offset_std.x();
    double vision_pos_lon_bias_std_ = vision_pos_lon_bias_std * (speed_related_weights_ + slip_related_weights_ / 3.0) + vis_data_ptr->pos_offset_std.y();
    double vision_heading_bias_std_ = vision_heading_bias_std * (speed_related_weights_ + slip_related_weights_) + vis_data_ptr->hdg_offset_std;

    if (cur_freq == F_LOW || cur_freq == F_5Hz) {
        // 如果视觉的频率较低，则稍微增加视觉的权重。。
        vision_pos_lat_bias_std_ = vision_pos_lat_bias_std_ * 0.70;
        vision_pos_lon_bias_std_ = vision_pos_lon_bias_std_ * 0.70;
        vision_heading_bias_std_ = vision_heading_bias_std_ * 0.70;
    }

    {
        if (std::fabs(process_control_sgt.state_nearby_info.vision_inno_info.vis_timestamp - state_ptr->measurement_timestamp) > process_control_sgt.state_nearby_info.vision_inno_info.TIME_RANGE) {
            process_control_sgt.state_nearby_info.vision_inno_info.vis_count  = 0;
            process_control_sgt.state_nearby_info.vision_inno_info.pos_mean_r = 0.0;
            process_control_sgt.state_nearby_info.vision_inno_info.hdg_mean   = 0.0;
        } else {
            auto       &pos_mean_r = process_control_sgt.state_nearby_info.vision_inno_info.pos_mean_r;
            auto       &hdg_mean   = process_control_sgt.state_nearby_info.vision_inno_info.hdg_mean;
            const auto &weight     = process_control_sgt.state_nearby_info.vision_inno_info.MEAN_WEIGHT_PER_MEASUREMENT;

            pos_mean_r = pos_mean_r * (1.0 - weight) + vis_data_ptr->pos_offset.y() * weight;
            hdg_mean   = hdg_mean * (1.0 - weight) + vis_data_ptr->hdg_offset * weight;

            process_control_sgt.state_nearby_info.vision_inno_info.vis_count++;
        }
        process_control_sgt.state_nearby_info.vision_inno_info.vis_timestamp = state_ptr->measurement_timestamp;
    }

    // { // 低通
    //     auto lpf_rslt_               = vf_lpf_({vis_data_ptr->pos_offset.x(), vis_data_ptr->pos_offset.y(), vis_data_ptr->hdg_offset});
    //     vis_data_ptr->pos_offset.x() = lpf_rslt_[0];
    //     vis_data_ptr->pos_offset.y() = lpf_rslt_[1];
    //     vis_data_ptr->hdg_offset     = lpf_rslt_[2];
    // }

    // if (process_control_sgt.fusion_status.imu_update_count_after_gnss_update > 30) {
    //     vf_mode_switch_ = false;
    // } else {
    //     vf_mode_switch_ = !vf_mode_switch_;
    // }
    // vf_mode_switch_        = true;
    constexpr double LARGE = 1e5;

    Eigen::Matrix<double, 3, 21> H;

    Eigen::Matrix3d T_veh2imu = (Eigen::AngleAxisd(state_ptr->vehicle_bias.z(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(state_ptr->vehicle_bias.x(), Eigen::Vector3d::UnitX())).toRotationMatrix().transpose();

    bool estimate_map_bias = state_ptr->rtk_status == Parameters::GNSS_STATUS::FIX &&
                             (process_control_sgt.msf_align_type == process_control_sgt.ALIGNED || process_control_sgt.msf_align_type == process_control_sgt.FINE_ALIGN);
    bool use_vision_to_maintain_accuracy = !estimate_map_bias;

    //&& process_control_sgt.fusion_status.imu_update_count_after_gnss_update > 50;

    if (state_ptr->vel.norm() > 2.0) {
        // 逐渐消去制图系统性偏差
        if (cur_freq == F_LOW || cur_freq == F_5Hz) {
            // 如果视觉的频率较低，则稍微增加消去的大小
            state_ptr->map_bias.x() = state_ptr->map_bias.x() - state_ptr->map_bias.x() / 125.0;
            state_ptr->map_bias.y() = state_ptr->map_bias.y() - state_ptr->map_bias.y() / 80.0;
            state_ptr->map_bias.z() = state_ptr->map_bias.z() - state_ptr->map_bias.z() / 60.0;
        } else {
            state_ptr->map_bias.x() = state_ptr->map_bias.x() - state_ptr->map_bias.x() / 250.0;
            state_ptr->map_bias.y() = state_ptr->map_bias.y() - state_ptr->map_bias.y() / 100.0;
            state_ptr->map_bias.z() = state_ptr->map_bias.z() - state_ptr->map_bias.z() / 100.0;
        }
    }

    Eigen::Vector3d vis_inno_mean_{0.0, 0.0, 0.0};
    Eigen::Vector3d vis_inno_std_{0.0, 0.0, 0.0};

    {
        Eigen::Vector3d vis_inno_{vis_data_ptr->pos_offset.x(), vis_data_ptr->pos_offset.y(), vis_data_ptr->hdg_offset};
        state_ptr->vis_inno_ = vis_inno_;
        if (std::abs(vis_inno_.x()) < 3.0 && std::abs(vis_inno_.z()) < 1.5) {
            vf_mean_std_.insert(vis_inno_);
        }
        // Eigen::Matrix<double, 3, 1> stable_offset_detect{0.0, 0.0, 0.0};
        if (vf_mean_std_.calc_mean_std()) {
            vis_inno_mean_ = vf_mean_std_.mean();
            vis_inno_std_  = vf_mean_std_.std();

            state_ptr->vis_t_inno_mean_std_(0)                = vis_data_ptr->measurement_timestamp;
            state_ptr->vis_t_inno_mean_std_.block<3, 1>(1, 0) = vis_inno_mean_;
            state_ptr->vis_t_inno_mean_std_.block<3, 1>(4, 0) = vis_inno_std_;

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES

            Eigen::Matrix3d C_veh2ref = state_ptr->C_b2n * state_ptr->C_imu2vehicle.transpose();
            // Eigen::Vector3d lla_imu_      = state_ptr->lla;
            // Eigen::Vector3d lla_antenna_  = state_ptr->lla + (state_ptr->MpvCnb * state_ptr->C_imu2vehicle * state_ptr->lever_imu2gnss);
            Eigen::Vector3d map_bias{state_ptr->map_bias.x(), state_ptr->map_bias.y(), 0.0};
            Eigen::Vector3d lla_vehicle_ = state_ptr->lla + (state_ptr->MpvCnb * state_ptr->C_imu2vehicle * state_ptr->lever_imu2vehicle) + state_ptr->Mpv * C_veh2ref * map_bias;

            Eigen::Vector3d vis_lla_vehicle_          = -state_ptr->Mpv * C_veh2ref * state_ptr->vis_inno_ + lla_vehicle_;
            double          vis_lla_vehicle_lat_mars_ = 0.0, vis_lla_vehicle_lon_mars_ = 0.0;
            // wgtochina_lb(0, vis_lla_vehicle_.y() * 180.0 / M_PI, vis_lla_vehicle_.x() * 180.0 / M_PI, vis_lla_vehicle_.z(), 0, 0, &vis_lla_vehicle_lon_mars_, &vis_lla_vehicle_lat_mars_);
            vis_lla_vehicle_lon_mars_ = vis_lla_vehicle_.y() * 180.0 / M_PI;
            vis_lla_vehicle_lat_mars_ = vis_lla_vehicle_.x() * 180.0 / M_PI;

            std::string vis_data_str =
                fmt::format(
                    "{:>14.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>14.8f},{:>14.8f},"
                    "{:>6.4f},{:>6.4f}\n",                    //
                    vis_data_ptr->measurement_timestamp,      // 1
                    vis_inno_mean_.x(),                       // 2
                    vis_inno_mean_.y(),                       // 3
                    vis_inno_mean_.z(),                       // 4
                    vis_inno_std_.x(),                        // 5
                    vis_inno_std_.y(),                        // 6
                    vis_inno_std_.z(),                        // 7
                    vis_inno_.x(),                            // 8
                    vis_inno_.y(),                            // 9
                    vis_inno_.z(),                            // 10
                    vis_lla_vehicle_lat_mars_,                // 11
                    vis_lla_vehicle_lon_mars_,                // 12
                    state_ptr->map_bias.z(),                  // 13
                    std::sqrt(state_ptr->error_state_std[20]) // 14
                );
            debug::debug_sgt.vis_state.line(vis_data_str);
#endif
        }
    }

    // #ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
    //     static fileio::ToCSV vf_tmp;
    //     {
    //         static auto vf_tmp_call_once_ = [this] {
    //             // 执行一次的逻辑
    //             vf_tmp.open("data/tmp/vf_tmp.csv");
    //             AINFO << "vf tmp open file ";
    //             return 0;
    //         }(); // 定义后立即执行
    //         (void)vf_tmp_call_once_;
    //     }
    // #endif

    auto estimate_map_bias_func_ = [state_ptr, kf_ptr, &H, &vis_inno_mean_, &vis_inno_std_, this](const Eigen::Vector3d &innovation_, const Eigen::Vector3d &cov_, double inno_adap_std_scale) {
        Eigen::Matrix3d V;

        Eigen::Vector3d scale_ = Eigen::Vector3d::Ones();

        vis_weight_adaptation(innovation_.x(), vis_inno_mean_.x(), vis_inno_std_.x(), scale_.x());

        Eigen::Vector3d inno_additional_ = Eigen::Vector3d::Zero();
        inno_additional_.x()             = std::pow(innovation_.x() * inno_adap_std_scale, 2);
        inno_additional_.y()             = std::pow(innovation_.y() * inno_adap_std_scale, 2);

        if (state_ptr->rtk_status == parameters_sgt.FIX) {
            scale_ = scale_ * 0.6;
        }

        V = (scale_.asDiagonal() * cov_ + inno_additional_).asDiagonal();
        H.setZero();
        H.block<3, 3>(0, 18) = Eigen::Vector3d{1.0, 1.0, 1.0}.asDiagonal();
        kf_ptr->kf_update(H, V, innovation_);

        Eigen::Vector3d dpos_ego = {kf_ptr->dx(18, 0), kf_ptr->dx(19, 0), 0.0};

        constexpr double dpos_bound_base_ = 0.01;
        double           dpos_bound_      = dpos_bound_base_ + 1.5e-3 * state_ptr->vel.norm();

        if (cur_freq == F_LOW || cur_freq == F_5Hz) {
            // 视觉频率低的话，则增大更新限幅
            dpos_bound_ = dpos_bound_ * 1.65;
        }

        switch (state_ptr->rtk_status) {
            case parameters_sgt.FIX: {
                dpos_bound_ = dpos_bound_ / 4.0;
            } break;

            case parameters_sgt.FLOAT: {
                dpos_bound_ = dpos_bound_ / 2.0;
            } break;

            default: {
            } break;
        }
        dpos_bound_ = dpos_bound_ > 0.025 ? 0.025 : dpos_bound_;

        localtrans.constrain_if_in_bound(dpos_ego.x(), LARGE, dpos_bound_);
        localtrans.constrain_if_in_bound(dpos_ego.y(), LARGE, dpos_bound_);

        state_ptr->map_bias.block<2, 1>(0, 0) -= dpos_ego.block<2, 1>(0, 0);

        // state_ptr->map_bias.y() -= 0.01 * state_ptr->map_bias.y();

        Eigen::Matrix<bool, 21, 1> idx;
        idx << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1;
        CovarianceUpdateByIdx(kf_ptr->P, kf_ptr->cov, idx, true);
    };

    auto use_vision_to_maintain_accuracy_func_ = [state_ptr, kf_ptr, &H, this](const Eigen::Vector3d &innovation_, const Eigen::Vector3d &cov_, double inno_adap_std_scale) {
        Eigen::Matrix3d V;

        Eigen::Matrix3d vis_cov_lla_ = state_ptr->MpvCnb * (cov_.asDiagonal()) * state_ptr->MpvCnb.transpose();

        Eigen::Matrix3d inno_additional_ = Eigen::Matrix3d::Identity();
        inno_additional_(0, 0)           = std::pow(innovation_.x() * inno_adap_std_scale, 2);
        inno_additional_(1, 1)           = std::pow(innovation_.y() * inno_adap_std_scale, 2);

        Eigen::Matrix3d V1_ = vis_cov_lla_ + inno_additional_;
        Eigen::Matrix3d V2_ = (9.0 * kf_ptr->P.block<3, 3>(6, 6).diagonal()).eval().asDiagonal();
        V                   = V1_ + V2_;
        H.setZero();
        H.block<3, 3>(0, 6) = Eigen::Vector3d{1.0, 1.0, 1.0}.asDiagonal();
        kf_ptr->kf_update(H, V, innovation_);
        Eigen::Vector3d dlla_{-kf_ptr->dx(6, 0), -kf_ptr->dx(7, 0), 0.0};

        constexpr double dpos_bound_base_ = 0.02;
        double           dpos_bound_      = dpos_bound_base_ + 1e-3 * state_ptr->vel.norm();

        if (cur_freq == F_LOW || cur_freq == F_5Hz) {
            // 视觉频率低的话，则增大更新限幅
            dpos_bound_ = dpos_bound_ * 1.65;
        }

        switch (state_ptr->rtk_status) {
            case parameters_sgt.FIX: {
                dpos_bound_ = dpos_bound_ / 4.0;
                dpos_bound_ = dpos_bound_ > 0.01 ? 0.01 : dpos_bound_;
            } break;

            case parameters_sgt.FLOAT: {
                dpos_bound_ = dpos_bound_ / 2.0;
                dpos_bound_ = dpos_bound_ > 0.02 ? 0.02 : dpos_bound_;
            } break;

            default: {
                // 这里考虑长隧道等长期无法FIX的场景
                // 如果长期无法FIX则放宽一点限制
                if (process_control_sgt.fusion_status.imu_update_count_after_rtk_fix > 100 * 60 * 3) {
                    dpos_bound_ = dpos_bound_ > 0.05 ? 0.05 : dpos_bound_;
                } else {
                    dpos_bound_ = dpos_bound_ > 0.03 ? 0.03 : dpos_bound_;
                }
            } break;
        }

        // 如果有RTK，根据融合定位稳定程度调整幅值限制。越稳定，修正量越小。
        if (process_control_sgt.fusion_status.imu_update_count_after_gnss_update < 200) {
            double dpos_scale_when_gnss_good_ = std::fabs(1.0 - state_ptr->state_stability_index_ratio_);
            dpos_scale_when_gnss_good_        = dpos_scale_when_gnss_good_ > 0.3 ? 0.3 : dpos_scale_when_gnss_good_;
            dpos_bound_                       = dpos_bound_ * dpos_scale_when_gnss_good_;
        }

        //         {

        // #ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
        //             Eigen::Vector3d dpos_rfu_ = localtrans.DPos_LLAtoEgo(state_ptr->lla, dlla_, state_ptr->att);

        //             std::string str_;
        //             str_ = fmt::format("{:>6.5f},{:>6.5f},{:>6.5f},{:>14.10f},{:>14.10f},{:>14.10f},{:>14.10f},{:>14.10f},{:>14.10f}\n", //
        //                                dpos_rfu_.x(), dpos_rfu_.y(), dpos_rfu_.z(), innovation_.x(), innovation_.y(), innovation_.z(), dlla_.x(), dlla_.y(), dlla_.z());
        //             vf_tmp.line(str_);
        // #endif
        //         }

        auto lla_ = localtrans.LLAUpdateWithEgoLatScaleConstrain(state_ptr->lla, dlla_, state_ptr->att, dpos_bound_);

        if (lla_.array().isNaN().any()) {
            // 防一手滤波出现NAN值
            AWARN << "vision fusion result NAN detected!";
        } else {
            state_ptr->lla = lla_;
            // Eigen::Matrix<bool, 21, 1> idx;
            // idx << 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
            // CovarianceUpdateByIdx(kf_ptr->P, kf_ptr->cov, idx, true);
        }
    };

    if (parameters_sgt.get_enable_vision_lat_fusion()) { // 融合横向位置偏差
        Eigen::Vector3d dp_x  = {vis_data_ptr->pos_offset.x(), 0.0, 0.0};
        Eigen::Vector3d dlla_ = localtrans.DPos_Ego2LLA(state_ptr->lla, T_veh2imu * dp_x, state_ptr->att) - state_ptr->lla;

        //         {
        // #ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
        //             std::string str_;
        //             str_ = fmt::format("{:>6.5f},", dp_x.x());
        //             vf_tmp.line(str_);
        // #endif
        //         }

        Eigen::Vector3d vision_pos_lat_bias_cov_ = (Eigen::Vector3d{vision_pos_lat_bias_std_, LARGE, LARGE}).cwiseAbs2();

        estimate_map_bias_func_(dp_x, vision_pos_lat_bias_cov_, 0.7);
        if (process_control_sgt.msf_align_type == process_control_sgt.INITIALIZATION || process_control_sgt.msf_align_type == process_control_sgt.COARSE_ALIGN) {
            // 如果融合定位状态较差的话，则对视觉的抗差弱一点
            use_vision_to_maintain_accuracy_func_(dlla_, vision_pos_lat_bias_cov_, 1.6);
        } else {
            // 如果融合定位状态较好的话，则对视觉的抗差强一点
            use_vision_to_maintain_accuracy_func_(dlla_, vision_pos_lat_bias_cov_, 2.0);
        }
    }

    if (parameters_sgt.get_enable_vision_lon_fusion()) { // 融合纵向位置偏差
        Eigen::Vector3d dp_y  = {0.0, vis_data_ptr->pos_offset.y(), 0.0};
        Eigen::Vector3d dlla_ = localtrans.DPos_Ego2LLA(state_ptr->lla, T_veh2imu * dp_y, state_ptr->att) - state_ptr->lla;

        Eigen::Vector3d vision_pos_lon_bias_cov_ = (Eigen::Vector3d{LARGE, vision_pos_lon_bias_std_, LARGE}).cwiseAbs2();

        estimate_map_bias_func_(dp_y, vision_pos_lon_bias_cov_, 0.7);
        use_vision_to_maintain_accuracy_func_(dlla_, vision_pos_lon_bias_cov_, 2.0);
    }

    if (                                                                             // 融合航向偏差条件
        parameters_sgt.get_enable_vision_hdg_fusion() &&                             // 配置是否启用
        state_ptr->vel.norm() > 5.0 &&                                               // 速度大于5m/s
        process_control_sgt.maneuver_status_by_imu == process_control_sgt.IMU_STEADY // 平稳行使状态
    ) {                                                                              //

        // 设计上，CNOA预期是在路口切图，其余场景没有图，可以假定图具有较好的精度
        // 这里还是把BEV和LD的航向角偏差的观测合入到融合定位的量测更新中
        // 如果是一直有LD的话，可能得调整下融合的策略，LD主路的精度很难保证。

        Eigen::Vector3d dyaw = {0.0, 0.0, vis_data_ptr->hdg_offset};

        Eigen::Vector3d innovation = dyaw;
        Eigen::Matrix3d V;
        Eigen::Vector3d vision_heading_bias_cov_ = (Eigen::Vector3d{LARGE, LARGE, vision_heading_bias_std_}).cwiseAbs2();

        Eigen::Vector3d heading_scale_{1.0, 1.0, 1.0 / igg3_hdg(innovation.z() / vision_heading_bias_std_)};

        if (use_vision_to_maintain_accuracy) {

            H.setZero();
            H.block<3, 3>(0, 0) = Eigen::Vector3d{1.0, 1.0, 1.0}.asDiagonal();

            V = (heading_scale_.asDiagonal() * vision_heading_bias_cov_ + innovation.cwiseAbs2() * vision_heading_additional_std_scale).asDiagonal();

            kf_ptr->kf_update(H, V, innovation);
            Eigen::Vector3d datt = Eigen::Vector3d::Zero();

            double delta_hdg = kf_ptr->dx(2);
            localtrans.constrain_if_in_bound(delta_hdg, LARGE, 0.02 / 180.0 * M_PI);

            datt.z()              = delta_hdg;
            Eigen::Quaterniond dq = INS::rv2q(datt);
            state_ptr->att        = dq * state_ptr->att;

            Eigen::Matrix<bool, 21, 1> idx;
            idx << 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
            CovarianceUpdateByIdx(kf_ptr->P, kf_ptr->cov, idx, true);
        }

        // 利用VF的航向量测
        // 这里考虑使用这个量测来估计BEV和LD的航向不匹配角度
        if (true) {
            H.setZero();
            H.block<3, 3>(0, 18) = Eigen::Vector3d{1.0, 1.0, 1.0}.asDiagonal();

            V = (heading_scale_.asDiagonal() * vision_heading_bias_cov_ + innovation.cwiseAbs2() * vision_heading_additional_std_scale).asDiagonal();
            kf_ptr->kf_update(H, V, innovation);

            Eigen::Matrix<double, 1, 1> delta_hdg = kf_ptr->dx.block<1, 1>(20, 0);
            // 这里限制一下制图航向角的修正速度，大约2秒修0.1度
            localtrans.constrain_if_in_bound(delta_hdg.x(), LARGE, 0.005 / 180.0 * M_PI);

            state_ptr->map_bias.block<1, 1>(2, 0) += delta_hdg;

            Eigen::Matrix<bool, 21, 1> idx;
            idx << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0;
            CovarianceUpdateByIdx(kf_ptr->P, kf_ptr->cov, idx, true);
        }
    }

    StateConstrain(state_ptr->map_bias, {constrain_map_bias.x(), constrain_map_bias.y(), constrain_map_bias.z()});

    // AINFO << "constrain_map_bias: " << constrain_map_bias.transpose();

    if (vis_data_ptr->pos_offset.norm() < 3.0 && //
        std::abs(vis_data_ptr->hdg_offset) < 5.0 / 180.0 * M_PI) {
        process_control_sgt.fusion_status.imu_update_count_after_vision_fusion_update = 0;
    }

    return true;
}

void VisionProcessor::vis_weight_adaptation(double lat_, double lat_mean_, double lat_std_, double &scale_) {
    double x       = std::abs(lat_);
    double scale_x = 1.0;

    constexpr double x0 = 0.00;
    constexpr double x1 = 0.15;
    constexpr double x2 = 0.30;
    constexpr double x3 = 0.45;

    constexpr double y0 = 1.4;
    constexpr double y1 = 1.2;
    constexpr double y2 = 1.1;
    constexpr double y3 = 1.0;

    if (x < x1) {
        scale_x = linear_interpolation(x0, x1, y0, y1, x);
    } else if (x < x2) {
        scale_x = linear_interpolation(x1, x2, y1, y2, x);
    } else if (x < x3) {
        scale_x = linear_interpolation(x2, x3, y2, y3, x);
    }

    bool stable_lat_inno_ =
        std::abs(lat_mean_) > 0.2 &&
        lat_std_ < 0.02 &&
        std::abs(lat_ - lat_mean_) < 0.05;

    if (stable_lat_inno_) {
        scale_x = 0.8 * scale_x;
    }

    scale_ = scale_x;
}

void VisionProcessor::update_frequency(double pre_timestamp, double cur_timestamp) {

    // 似乎正负号不影响滤波过程，这里直接取时间差的绝对值
    double dt_ = std::fabs(cur_timestamp - pre_timestamp);

    // 时间间隔大于0.4000(频率小于2.5Hz)，认为是低频状态
    if (dt_ > 0.4000) {
        cur_freq = F_LOW;
        return;
    }
    // 时间间隔大于0.1333(频率小于7.5Hz)，认为是5Hz状态
    if (dt_ > 0.1333) {
        cur_freq = F_5Hz;
        return;
    }
    // 时间间隔大于0.0800(频率小于12.5Hz)，认为是10Hz状态
    if (dt_ > 0.0800) {
        cur_freq = F_10Hz;
        return;
    }
    // 时间间隔大于0.0571(频率小于17.5Hz)，认为是15Hz状态
    if (dt_ > 0.0571) {
        cur_freq = F_15Hz;
        return;
    }
    // 时间间隔大于0.0444(频率小于22.5Hz)，认为是20Hz状态
    if (dt_ > 0.0444) {
        cur_freq = F_20Hz;
        return;
    }
    // 时间间隔大于0.0，认为是高频状态
    if (dt_ > 0.0) {
        cur_freq = F_HIGH;
        return;
    }
}

} // namespace MSF