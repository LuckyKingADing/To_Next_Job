#include "gps_single_processor.h"
#include "calc.h"
#include "cyber/common/log.h"
#include "rigid_transform.h"
#include <iomanip>

#include "fmt/format.h"

#include "Coord.h"
#include "processor_debug.h"

namespace MSF {

bool GpsSingleProcessor::UpdateStateByGpsSingle(GnssDataPtr gps_data_ptr, StatePtr state_ptr, KfPtr<21, 3> kf_ptr, const KinematicDataPtr dr_ptr) {

    cross_validate.insert(gps_data_ptr, state_ptr, dr_ptr);

    // 组合导航状态相关
    INS::EARTH earth(gps_data_ptr->lla, gps_data_ptr->vel);
    double     RMh2_   = earth.RMh * earth.RMh;
    double     clRNh2_ = earth.clRNh * earth.clRNh;

    // 计算位置新息
    Eigen::Vector3d pos_lla_inno_{state_ptr->lla - (gps_data_ptr->lla - state_ptr->MpvCnb * state_ptr->C_imu2vehicle * state_ptr->lever_imu2gnss)};
    Eigen::Vector3d pos_enu_inno_{pos_lla_inno_.y() * earth.clRNh, pos_lla_inno_.x() * earth.RMh, pos_lla_inno_.z()};
    Eigen::Vector3d pos_rfu_inno_{state_ptr->C_imu2vehicle * state_ptr->C_b2n.transpose() * pos_enu_inno_};

    state_ptr->pos_rfu_inno_ = pos_rfu_inno_;

    {
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

    {
        gnss_msg_count_++;

        // 这里使用轨迹信息，分析是否需要进行重新初始化
        auto            gps_lla_ = gps_data_ptr->lla - state_ptr->MpvCnb * state_ptr->C_imu2vehicle * state_ptr->lever_imu2gnss;
        Eigen::Vector2d lonlat_msf_{state_ptr->lla.y(), state_ptr->lla.x()};
        Eigen::Vector2d lonlat_gps_{gps_lla_.y(), gps_lla_.x()};
        Eigen::Vector2d frtlft_dr_{dr_ptr->pos.x(), dr_ptr->pos.y()};
        double          timestamp_ = gps_data_ptr->measurement_timestamp;
        double          mileage_   = state_ptr->mileage;
        traj_info.update(timestamp_, mileage_, lonlat_gps_, lonlat_msf_, frtlft_dr_);
        if (traj_info.isWindowFull() && gnss_msg_count_ % 20 == 0) {
            // 每20个数据更新，做一次数据有效性检验
            double out_frechet_gnss_dr = 0.0;
            double out_rmse_gnss_fused = std::numeric_limits<double>::max();
            if (traj_info.computeMetrics(out_frechet_gnss_dr, out_rmse_gnss_fused)) {
                // AINFO << "frechet: " << out_frechet_gnss_dr << " rmse: " << out_rmse_gnss_fused;
                // 这里检测一下out_rmse_gnss_fused不为max值，认为才是合理的。
                if (std::fabs(out_frechet_gnss_dr) < kFrechetThresholdMeters &&
                    std::fabs(out_rmse_gnss_fused) > kRmseThresholdMeters &&
                    out_frechet_gnss_dr != 0.0 &&
                    out_rmse_gnss_fused != std::numeric_limits<double>::max()) {

                    AINFO << "traj_info frechet distance <gnss dr>: " << out_frechet_gnss_dr << " rmse <gnss msf>: " << out_rmse_gnss_fused;

                    AERROR << "reinit due to large pos error";
                    process_control_sgt.reinitialization_state.position = true;
                    process_control_sgt.reinitialization_state.velocity = true;
                    process_control_sgt.reinitialization_state.heading  = true;
                    process_control_sgt.msf_align_type                  = process_control_sgt.INITIALIZATION;
                }
            }
        }
    }

    {
        double h_spd_ = state_ptr->vel.block<2, 1>(0, 0).norm();
        if (pos_rfu_inno_.block<2, 1>(0, 0).norm() < 10.0) {
            // 位置新息小于10米 且 自车水平速度超过 10 m/s，认为是状态良好的卫星观测
            // 这里设计一下：300m 收敛差值的 0.6
            double scale_ = h_spd_ * 3.3e-3 * 0.1;
            scale_        = scale_ > 1e-2 ? 1e-2 : scale_;

            good_state_sat_num_mean_ = good_state_sat_num_mean_ * (1.0 - scale_) + gps_data_ptr->num_sats * scale_;
        } else {
            // 这里可能有一种情况导致计算的卫星数均值与实际不符
            // 即：刚开始卫星数量处于较高的水平，导致计算的卫星数量均值较大。但是后续卫星数量显著下降，同时融合定位的位置误差较大，导致无法重新计算卫星数量。
            // 这个时候，因为持续认为SPP状态差，不会融合卫星信息，导致陷入死循环
            // 所以，这里需要额外做一些措施，以保证能从此种困境出来。
            // 不过需要考虑长隧道持续有卫星转发器的场景，这里不能收敛太快。
            // 这里设计一下：30km 收敛差值的 0.6
            double scale_ = h_spd_ * 3.3e-5 * 0.1;
            scale_        = scale_ > 1e-3 ? 1e-3 : scale_;

            good_state_sat_num_mean_ = good_state_sat_num_mean_ * (1.0 - scale_) + gps_data_ptr->num_sats * scale_;
        }

        state_ptr->spp_good_sat_num_mean_ = good_state_sat_num_mean_;

        // 考虑到卫星状态可能会有复杂的模式
        // 这里定义一个新的值，计算预期卫星数
        double expect_sat_num_mean_ = 0.0;

        // 这里考虑RTK切换到SPP的情况
        // 预期卫星均值选取RTK和SPP模式中大的那一个
        expect_sat_num_mean_ = state_ptr->spp_good_sat_num_mean_ > state_ptr->rtk_fix_sat_num_mean_ ? state_ptr->spp_good_sat_num_mean_ : state_ptr->rtk_fix_sat_num_mean_;
        // 另外，考虑到RTK和SPP模式，可能时间间隔比较大，并不能保证RTK计算的卫星数均值是合理的。
        // 只要SPP模式下，卫星均值超过30，则使用SPP模式计算的卫星数量
        if (state_ptr->spp_good_sat_num_mean_ > 30) {
            expect_sat_num_mean_ = state_ptr->spp_good_sat_num_mean_;
        }

        // 对卫星进行一个校验
        // 卫星数这个指标本身并不是一个很可靠的参考
        // 此处主要目的是应对隧道内存在卫星转发器的场景
        // 这里引入位置新息和卫星帧间运动学偏差作为额外约束
        // 从设计上预防：卫星数因环境自身产生了显著变化，从而导致过度过滤卫星观测的情况
        bool big_inno_while_sat_num_decrease_ =
            expect_sat_num_mean_ - gps_data_ptr->num_sats > 10.0 &&                     // 卫星数显著减少
            pos_rfu_inno_.block<2, 1>(0, 0).norm() > 10.0 &&                            // 位置新息比较大
            (h_spd_ > 3.0 && std::fabs(cross_validate.get_gnss_dr_diff_index()) > 0.3); // 运动的情况下，帧间运动学偏差偏大
        bad_gnss_detected_ =
            (big_inno_while_sat_num_decrease_ || std::fabs(cross_validate.get_gnss_dr_diff_index()) > 1.5) &&                // 卫星数显著变少或者卫星与DR的交叉验证差值过大
            (gps_data_ptr->status != Parameters::GNSS_STATUS::FIX && gps_data_ptr->status != Parameters::GNSS_STATUS::FLOAT) // 非固定或者浮点解状态
            ;
        AINFO_EVERY(100) << "good_sat_num_mean_: " << good_state_sat_num_mean_ << " cur diff: " << good_state_sat_num_mean_ - gps_data_ptr->num_sats;
        AINFO_EVERY(100) << "gnss_dr_diff_index: " << std::fabs(cross_validate.get_gnss_dr_diff_index());
        if (bad_gnss_detected_) {
            AINFO_EVERY(30) << "bad gnss detected!";
        }

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
        {
            std::string spp_vali_str;
            spp_vali_str = fmt::format(                              //
                "{:>14.4f},{:>14.10f},{:>14.10f},{:>6.3f},{:>6.3f}," //
                "{:>6.3f},{:>6.3f},{:>6.3f},{:d},{:>6.3f}\n",        //
                state_ptr->measurement_timestamp,                    // 1
                gps_data_ptr->lla.x() * 180.0 / M_PI,                // 2
                gps_data_ptr->lla.y() * 180.0 / M_PI,                // 3
                gps_data_ptr->lla.z(),                               // 4
                state_ptr->rtk_fix_sat_num_mean_,                    // 5
                state_ptr->spp_good_sat_num_mean_,                   // 6
                pos_rfu_inno_.block<2, 1>(0, 0).norm(),              // 7
                std::fabs(cross_validate.get_gnss_dr_diff_index()),  // 8
                bad_gnss_detected_,                                  // 9
                (double)gps_data_ptr->num_sats                       // 10
            );
            debug::debug_sgt.spp_vali_state.line(spp_vali_str);
        }
#endif
    }

    // GNSS MSG 预处理
    MeasurementDataPreProcess(gps_data_ptr, state_ptr);

    // 滤波器相关
    Eigen::Matrix<double, 3, 21> H;
    Eigen::Vector3d              innovation;
    Eigen::Matrix3d              V;
    Eigen::Matrix<double, 8, 1>  inno_ = Eigen::Matrix<double, 8, 1>::Zero();

    Eigen::Matrix<bool, 21, 1> idx;

    state_ptr->rtk_status = gps_data_ptr->status;

    bool MSF_state_may_bad_ =
        std::fabs(state_ptr->mileage) < 300.0 ||                                  // 初始里程前300米
        process_control_sgt.msf_align_type == process_control_sgt.INITIALIZATION; // 初始化状态

    // 根据mileage进行渐进式阈值限制
    double consistency_threshold = 0.8;
    if (std::fabs(state_ptr->mileage) < 50.0) {
        consistency_threshold = 0.6; // 前50米，阈值宽松
    } else if (std::fabs(state_ptr->mileage) < 100.0) {
        consistency_threshold = 0.7; // 前100米，阈值中等
    }

    bool gnss_consistency_good_ =
        state_ptr->gnss_consistency_score_ > 0.0 && state_ptr->gnss_consistency_score_ > consistency_threshold;

    // 判断一下，刚开始初始化的时候，提高收敛速度
    bool fast_convergence_ =
        gnss_consistency_good_ &&      // 卫星一致性验证通过
        !bad_gnss_detected_ &&         // 未检测到卫星异常
        state_ptr->vel.norm() > 5.0 && // 有一定速度
        MSF_state_may_bad_             // 融合状态可能比较差
        ;

    if (parameters_sgt.get_enable_gnss_pos_fusion() && !bad_gnss_detected_) { // 配置是否融合GNSS的位置

        // Compute residual.

        innovation = state_ptr->lla - (gps_data_ptr->lla - state_ptr->MpvCnb * state_ptr->C_imu2vehicle * state_ptr->lever_imu2gnss);

        { // 记录位置新息
            Eigen::Vector3d pos_{innovation.y() * earth.clRNh, innovation.x() * earth.RMh, innovation.z()};

            inno_(0)                = gps_data_ptr->measurement_timestamp;
            inno_.block<3, 1>(1, 0) = pos_;
        }

        {
            H.setZero();
            H.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();

            const auto     &lla_cov_ = gps_data_ptr->lla_cov;
            Eigen::Vector3d lla_cov{lla_cov_(0) / RMh2_, lla_cov_(1) / clRNh2_, lla_cov_(2)};
            Eigen::Vector3d lla_cov_additional = innovation.cwiseAbs2();

            Eigen::Vector3d lla_std = lla_cov.cwiseSqrt() + 3.0 * kf_ptr->P.block<3, 3>(6, 6).diagonal().cwiseSqrt();

            Eigen::Vector3d lla_cov_scale{
                1.0 / igg3_pos(innovation.x() / lla_std.x()),
                1.0 / igg3_pos(innovation.y() / lla_std.y()),
                1.0 / igg3_pos(innovation.z() / lla_std.z())};

            double pos_norm_scale_ = 1.0 / igg3_pos_norm(innovation.block<2, 1>(0, 0).norm() / (lla_std.block<2, 1>(0, 0).norm() + 1e-10));
            if (cross_validate.is_msf_invalid()) {
                AINFO_EVERY(100) << "MSF not valid, disable IGG";
                V = (lla_cov + lla_cov_additional).asDiagonal();
            } else {
                V = (pos_norm_scale_ * lla_cov_scale.asDiagonal() * lla_cov + lla_cov_additional).asDiagonal();
            }

            bool msf_position_may_bad_ =
                std::fabs(process_control_sgt.state_nearby_info.pose_inno_info.pos_timestamp - state_ptr->measurement_timestamp) < 0.5 &&
                process_control_sgt.state_nearby_info.pose_inno_info.pos_count > 20 &&
                process_control_sgt.state_nearby_info.pose_inno_info.pos_mean_rfu.block<2, 1>(0, 0).norm() > 10.0;

            if (msf_position_may_bad_) {
                AINFO_EVERY(10) << "[Robustness Suppression] position, msf may bad";
                // 如果融合定位的位置新息比较大
                // 那么认为融合定位不是很准，这里降低抗差
                lla_cov_additional = lla_cov_additional * 0.5;

                V = (lla_cov + lla_cov_additional).asDiagonal();
            }

            kf_ptr->kf_update(H, V, innovation);

            switch (process_control_sgt.vehicle_motion_type) {

                case process_control_sgt.ZERO_VELOCITY: {
                    // 单点解在车辆静止情况下，会持续飘动。
                    // 这里考虑不做位置融合
                    // {
                    //     // 处于零速状态时，在GNSS量测更新周期内，只对位置观测进行量测更新
                    //     state_ptr->lla -= kf_ptr->dx.block<3, 1>(6, 0);
                    //     process_control_sgt.lla0_zupt = state_ptr->lla;
                    //     idx << 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
                    //     CovarianceUpdateByIdx(kf_ptr->P, kf_ptr->cov, idx, true);
                    // }
                    break;
                }

                case process_control_sgt.MOVING: {

                    Eigen::Vector3d lla_ = state_ptr->lla;

                    Eigen::Vector3d pos_update_linear_ = EnuUpdateLinearBalance(innovation, -kf_ptr->dx.block<3, 1>(6, 0), earth);

                    static constexpr double FRONT_SPEED_SCALE_ = 0.1 * 0.2;
                    static constexpr double DPOS_BOUND_MAX_    = 3e-1;

                    double dpos_bound_by_spd_ = std::fabs(state_ptr->vel_ego.y() * FRONT_SPEED_SCALE_);

                    // 这里依据前向速度，限制横向的位移修正量，并限制上限
                    // 此操作，抑制在车速低的时候，位置产生明显的横向漂移
                    double dpos_bound_ = 0.0;
                    if (fast_convergence_) {
                        // 如果需要加快收敛速度，则不对修正量进行额外限制
                        dpos_bound_ = dpos_bound_by_spd_;
                    } else {
                        // 如果不需要加快收敛速度，则限制修正量上限
                        dpos_bound_ = dpos_bound_by_spd_ > DPOS_BOUND_MAX_ ? DPOS_BOUND_MAX_ : dpos_bound_by_spd_;
                    }

                    lla_ = local_trans.LLAUpdateWithEgoLatConstrain(
                        state_ptr->lla,     //
                        pos_update_linear_, //
                        state_ptr->att,     //
                        1e14,               //
                        dpos_bound_         //
                    );

                    state_ptr->lla = lla_;

                    // 横向新息大的时候，会拉歪一些速度和航向
                    // 设置一个阈值，横向位置新息很大的时候，就不更新速度和航向了
                    if (std::abs(pos_rfu_inno_.x()) < 10.0) {

                        state_ptr->vel -= kf_ptr->dx.block<3, 1>(3, 0);

                        // 姿态更新
                        Eigen::Vector3d    datt = constrain(kf_ptr->dx.block<3, 1>(0, 0), MaxEulrAnglePerMeasurementUpdate);
                        Eigen::Quaterniond dq   = INS::rv2q(datt);

                        state_ptr->att = dq * state_ptr->att;
                        state_ptr->att.normalize();

                        state_ptr->gyro_bias += kf_ptr->dx.block<3, 1>(9, 0);
                        state_ptr->acc_bias += kf_ptr->dx.block<3, 1>(12, 0);
                    }
                    StateConstrain(state_ptr->gyro_bias, constrain_gyro_bias);
                    StateConstrain(state_ptr->acc_bias, constrain_acc_bias);

                    kf_ptr->P = kf_ptr->cov;

                    // // 姿态、速度、位置、陀螺零偏、加计零偏
                    // kf_ptr->es.block<15, 1>(0, 0) = 0.0 * kf_ptr->dx.block<15, 1>(0, 0);

                    break;
                }

                default:
                    break;
            }

            {
                // 通过GNSS的权重判断GNSS更新有效性，若更新有效，则重置IMU更新计数
                bool gnss_update_ =
                    kf_ptr->K.block<2, 2>(6, 0).diagonal().norm() > 1e-2;
                if (gnss_update_) {
                    process_control_sgt.fusion_status.imu_update_count_after_gnss_update = 0;
                    process_control_sgt.fusion_status.continous_gnss_fusion_count++;
                }
            }
        }
    }

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

            Eigen::Vector3d vel_cov = gps_data_ptr->vel_cov;

            double          hori_acc_                      = (state_ptr->acc - state_ptr->acc_bias).block<2, 1>(0, 0).norm();
            Eigen::Vector3d acc_related_vel_cov_additional = (hori_acc_ * Eigen::Vector3d::Ones()).cwiseAbs2();

            Eigen::Vector3d vel_cov_additional = innovation.cwiseAbs2() + acc_related_vel_cov_additional;

            auto vel_std = vel_cov.cwiseSqrt() + 9.0 * kf_ptr->P.block<3, 3>(3, 3).diagonal().cwiseSqrt();

            Eigen::Vector3d vel_cov_scale{
                1.0 / igg3_vel(innovation.x() / vel_std.x()),
                1.0 / igg3_vel(innovation.y() / vel_std.y()),
                1.0 / igg3_vel(innovation.z() / vel_std.z())};

            V = (vel_cov_scale.asDiagonal() * vel_cov + vel_cov_additional).asDiagonal();

            kf_ptr->kf_update(H, V, innovation);

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

    if (parameters_sgt.get_enable_gnss_heading_fusion() && !bad_gnss_detected_) { // 配置是否融合GNSS的航向

        // innovation.setZero();
        H.setZero();

        // 这里要考虑下，要不要补偿IMU的安装角偏差
        // 比较担心的一点是如果补偿的话，这个安装角偏差的约束性比较弱，可能会不受控制的增加
        // 但是理论上，肯定是要补偿，才可以得到最优的航向
        double dhdg_ = -gps_data_ptr->hdg - (INS::quaternion2euler(state_ptr->att).z() - state_ptr->vehicle_bias.z());

        // 侧滑角补偿
        {
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
            state_ptr->vel.norm() > 7.0; // 速度较快

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
                hdg_std_   = hdg_scale_ * (0.3 + 2.0 / (state_ptr->vel.norm() + 1.0)) / 180.0 * M_PI;
            } else {
                hdg_scale_ = 1.0 + std::abs(state_ptr->gyro.z()) / (2.0 / 180.0 * M_PI);
                hdg_std_   = hdg_scale_ * (1.0 + 4.0 / (state_ptr->vel.norm() + 1.0)) / 180.0 * M_PI;
            }
            double          att_cov_igg_scale = 1.0 / igg3_hdg(innovation.z() / hdg_std_);
            Eigen::Vector3d att_cov           = (att_cov_igg_scale * std::pow(hdg_std_, 2) + gps_data_ptr->hdg_cov) * Eigen::Vector3d::Ones();

            Eigen::Vector3d att_cov_additional = innovation.cwiseAbs2();

            Eigen::Vector3d dh_additional = Eigen::Vector3d::Zero();

            V = (att_cov + att_cov_additional).asDiagonal();

            bool msf_heading_may_bad_ =
                std::fabs(process_control_sgt.state_nearby_info.heading_inno_info.timestamp - state_ptr->measurement_timestamp) < 0.5 &&
                process_control_sgt.state_nearby_info.heading_inno_info.hdg_count > 20 &&
                std::fabs(process_control_sgt.state_nearby_info.heading_inno_info.mean_compensated) > 0.3 / 180.0 * M_PI;
            if ((msf_heading_may_bad_ && gnss_heading_may_good_) || fast_convergence_) {
                AINFO_EVERY(10) << "msf heading may bad, accelerate heading convergence";
                V = ((0.005 * 0.005 + gps_data_ptr->hdg_cov) * Eigen::Vector3d::Ones() + att_cov_additional).asDiagonal();
            }

            kf_ptr->kf_update(H, V, innovation);

            switch (process_control_sgt.vehicle_motion_type) {
                case process_control_sgt.ZERO_VELOCITY: {
                    break;
                }

                case process_control_sgt.MOVING: {

                    Eigen::Vector3d    datt = constrain(kf_ptr->dx.block<3, 1>(0, 0), MaxEulrAnglePerMeasurementUpdate);
                    Eigen::Quaterniond dq   = INS::rv2q(datt);
                    state_ptr->att          = dq * state_ptr->att;
                    state_ptr->att.normalize();

                    state_ptr->vel -= kf_ptr->dx.block<3, 1>(3, 0);
                    state_ptr->lla -= kf_ptr->dx.block<3, 1>(6, 0);

                    state_ptr->gyro_bias += kf_ptr->dx.block<3, 1>(9, 0);
                    state_ptr->acc_bias += kf_ptr->dx.block<3, 1>(12, 0);

                    // idx << 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1;
                    // CovarianceUpdateByIdx(kf_ptr->P, kf_ptr->cov, idx, true);

                    kf_ptr->P = kf_ptr->cov;
                    break;
                }

                default:
                    break;
            }
        } else {
        }
    }

    state_ptr->gnss_inno_ = inno_;

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
    {
        Eigen::Matrix3d ref2body     = state_ptr->C_imu2vehicle * state_ptr->C_b2n.transpose();
        Eigen::Vector3d inno_pos_ego = ref2body * inno_.block<3, 1>(1, 0);
        Eigen::Vector3d inno_vel_ego = ref2body * inno_.block<3, 1>(4, 0);
        Eigen::Vector3d lla_std_ego  = ref2body * gps_data_ptr->lla_cov.cwiseSqrt();
        Eigen::Vector3d vel_std_ego  = ref2body * gps_data_ptr->vel_cov.cwiseSqrt();
        double          lat_mars = 0.0, lon_mars = 0.0;
        wgtochina_lb(0, gps_data_ptr->lla.y() * 180.0 / M_PI, gps_data_ptr->lla.x() * 180.0 / M_PI, gps_data_ptr->lla.z(), 0, 0, &lon_mars, &lat_mars);
        auto gps_str_ = fmt::format("{:>14.4f},{:>14.10f},{:>14.10f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:d},{:d},{:>14.10f},{:>14.10f},{:>6.3f},"
                                    "{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:d},{:d},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},"
                                    "{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f}\n",
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
                                    vel_std_ego.z()                                   // 34
        );
        debug::debug_sgt.gps_state.line(gps_str_);
    }
#endif

    return true;
}

void GpsSingleProcessor::MeasurementDataPreProcess(GnssDataPtr gps_data_ptr, StatePtr state_ptr) {

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
            if (pos_sd < 0.15 + pos_sd_additional_) {
                pos_sd = 0.15 + pos_sd_additional_;
            }
            if (vel_sd < 0.15) {
                vel_sd = 0.15;
            }
        }
    }

    {
        // 位置协方差很大的时候，也不信任速度协方差
        if (gps_data_ptr->lla_cov.block<2, 1>(0, 0).norm() > 20.0) {
            if (gps_data_ptr->vel_cov.block<2, 1>(0, 0).norm() < 20.0) {
                gps_data_ptr->vel_cov = Eigen::Vector3d{20.0, 20.0, 20.0};
            }
        }
    }

    {
        // 弯道场景，速度不可靠
        double scale_         = 1.0 + std::abs(state_ptr->gyro.z()) / (2.0 / 180.0 * M_PI);
        gps_data_ptr->vel_cov = scale_ * gps_data_ptr->vel_cov;
    }

    {
        switch (process_control_sgt.vehicle_motion_type) {

            case process_control_sgt.ZERO_VELOCITY: {
                // 单点解的情况下，在静态条件下，位置容易出现明显的漂移
                // 这里做个预处理，降低静态条件下卫星权重
                gps_data_ptr->lla_cov = gps_data_ptr->lla_cov + 15.0 * Eigen::Vector3d::Ones();
            } break;

            case process_control_sgt.MOVING: {
            } break;

            default:
                break;
        }
    }

    {
        // 系统越稳定，则越可以降低GNSS的权重
        auto &ssir            = state_ptr->state_stability_index_ratio_;
        gps_data_ptr->lla_cov = (1.0 + ssir) * gps_data_ptr->lla_cov;
        gps_data_ptr->vel_cov = (1.0 + ssir) * gps_data_ptr->vel_cov;
        AINFO_EVERY(1000) << "SSI info, std additional ratio: " << ssir;

        // 源消息中带的是标准差，这里转换成方差
        gps_data_ptr->lla_cov = gps_data_ptr->lla_cov.cwiseAbs2();
        gps_data_ptr->vel_cov = gps_data_ptr->vel_cov.cwiseAbs2();
        gps_data_ptr->hdg_cov = (gps_data_ptr->hdg_cov / 180.0 * M_PI) * (gps_data_ptr->hdg_cov / 180.0 * M_PI);
    }
}

Eigen::Vector3d GpsSingleProcessor::EnuUpdateLinearBalance(const Eigen::Vector3d &pos_inno_, const Eigen::Vector3d &pos_update_, const INS::EARTH &earth_) {
    double pos_inno_distance_ = std::sqrt(std::pow(pos_inno_.x() * earth_.RMh, 2.0) + std::pow(pos_inno_.y() * earth_.clRNh, 2.0));

    static constexpr double DISTANCE_BOUND_     = 6.0;
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

void CrossValidation::insert(const GnssDataPtr gps_data_ptr_, const StatePtr state_ptr_gpst_, const KinematicDataPtr dr_ptr_gpst_) {

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

        bool gnss_valid = std::all_of(navi_info_buffer.begin() + 1, navi_info_buffer.end(), [](const NaviInfo &ele) {
            double spd_    = ele.DrVel.norm();
            bool   vel_ok  = spd_ > 2.0;
            bool   gnss_ok = std::fabs(ele.DrDeltaRfu.norm() - ele.GnssDeltaRfu.norm()) < 0.4 + spd_ * 0.02;
            return vel_ok && gnss_ok;
        });
        if (gnss_valid) {
            double pos_diff_mean = 0.0;
            for (int i = 1; i < BUFFER_SIZE; i++) {
                auto &cur     = navi_info_buffer.at(i);
                pos_diff_mean = pos_diff_mean + cur.GnssMsfDiffRfu.norm();
            }
            pos_diff_mean = pos_diff_mean / (BUFFER_SIZE - 1.0);
            if (pos_diff_mean > 4.0 && process_control_sgt.vehicle_motion_type == process_control_sgt.MOVING) {
                msf_invalid = true;
            } else {
                msf_invalid = false;
            }
        }

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
            state_stability_index_update(gps_data_ptr_, state_ptr_gpst_, gnss_, cross_);
            gnss_dr_diff_index = gnss_;
            // fmt::print("GNSS & MSF & cross: {:>6.3f},{:>6.3f},{:>6.3f}, {:>6.3f},{:>6.3f}\n", gnss_, msf_, cross_, get_state_stability_index_(), get_state_stability_index_ratio_());
        }

        navi_info_buffer.clear();
    }
}

void CrossValidation::state_stability_index_update(const GnssDataPtr gps_data_ptr, const StatePtr state_ptr, double gnss_diff_, double msf_diff_) {
    double delta_mileage_ = std::abs(ssi_pre_mileage_ - state_ptr->mileage);
    double delta_time_    = std::abs(ssi_pre_timestamp_ - gps_data_ptr->measurement_timestamp);

    double scale_ = (0.3 - gnss_diff_) + (1.0 - msf_diff_);
    scale_        = scale_ < -3.0 ? -3.0 : scale_;

    if (delta_time_ < BUFFER_SIZE * GNSS_MSG_SKIP * parameters_sgt.get_gnss_data_refresh_dt() * 2.0) {
        state_stability_index_ += delta_mileage_ * scale_;
    } else {
        state_stability_index_ -= delta_mileage_;
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

} // namespace MSF