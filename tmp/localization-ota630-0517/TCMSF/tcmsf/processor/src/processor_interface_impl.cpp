#include "processor_interface_impl.h"

#include "cyber/common/log.h"

#include "calc.h"

#include "processor_debug.h"
#include "rigid_transform.h"

#include <thread>

#include "Coord.h"
#include "replay_mode.h"

namespace MSF {

ProcessorImpl::ProcessorImpl() {
    kf_ptr_ = std::make_shared<KF<21, 3>>();

    initializer_       = std::make_unique<Initializer>(kf_ptr_);
    imu_processor_     = std::make_unique<ImuProcessor>();
    gps_processor_     = std::make_unique<GpsProcessor>();
    vehicle_processor_ = std::make_unique<VehicleProcessor>();
    zupt_processor_    = std::make_unique<ZuptProcessor>();
    att_referencer_    = std::make_unique<AttReference>();
    vision_processor_  = std::make_unique<VisionProcessor>();
    map_processor_     = std::make_unique<MapProcessor>();
    db_processor_      = std::make_unique<DbProcessor>();

    gps_single_processor_ = std::make_unique<GpsSingleProcessor>();

    pre_imu_data_ptr_  = std::make_shared<ImuData>();
    pre_veh_data_ptr_  = std::make_shared<VehicleData>();
    pre_gnss_data_ptr_ = std::make_shared<GnssData>();
}

bool ProcessorImpl::ProcessImuData(const ImuDataPtr imu_data_ptr, double dt_, StatePtr state_ptr) {

#if (defined __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES) && (defined __TCMSF_DEBUG__ENABLE_REPLAY_MODE)
    {
        // 这个额外判断是否启用保存到文件，以做一个保护
        byd::replay_mode::ReplayModeSGT::getInstance().sync_info_to_state(state_ptr, kf_ptr_, [imu_data_ptr, state_ptr, this] {
            process_control_sgt.msf_align_type                  = (ProcessControl::AlignmentType)state_ptr->align_type;
            process_control_sgt.initialization_count            = 1;
            process_control_sgt.reinitialization_state.heading  = false;
            process_control_sgt.reinitialization_state.velocity = false;
            process_control_sgt.reinitialization_state.position = false;
            init_traj_analysis_.setFirstTimeInitialization();
            {
                state_ptr->gyro = imu_data_ptr->gyro / 0.01;
                // 转换到IMU系
                auto state_imu_                  = state_ptr->FromVehicleToImu();
                state_ptr->lla                   = state_imu_.lla;
                state_ptr->att                   = state_imu_.att;
                state_ptr->vel                   = state_imu_.vel;
                state_ptr->q_imu2vehicle         = state_imu_.q_imu2vehicle;
                state_ptr->wheel_spd_scale_bias_ = state_imu_.wheel_spd_scale_bias_;
            }
            {
                process_control_sgt.fusion_status.imu_update_count_after_gnss_update = 0;
                gps_processor_->set_ssi_(3000.0);
            }
            // if (true) {
            //     // 是否需要额外做一个加偏
            //     double lat_mars = 0.0, lon_mars = 0.0;
            //     wgtochina_lb(0, state_ptr->lla.y() * 180.0 / M_PI, state_ptr->lla.x() * 180.0 / M_PI, state_ptr->lla.z(), 0, 0, &lon_mars, &lat_mars);
            //     state_ptr->lla.x() = lat_mars / 180.0 * M_PI;
            //     state_ptr->lla.y() = lon_mars / 180.0 * M_PI;
            // }
            imu_processor_->Feedback(state_ptr, true);
            AINFO << "[replay mode] init info: " << fmt::format("imu timestamp: {:14.4f}", imu_data_ptr->measurement_timestamp);
        });
    }
#endif

#ifndef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
    // 本地回灌的时候，如果开启了DEBUG模式，没必要记录轨迹LOG，屏蔽DEBUG模式下的输出
    if (state_ptr->tcmsf_1st_time_initialized_) {
        // 这里做一个轨迹的记录
        // 每10个记录一个
        static uint64_t sequence_num_ = 0;
        if (sequence_num_++ % 10 == 0) {
            Eigen::Vector3d lla_antenna_         = state_ptr->lla + (state_ptr->MpvCnb * state_ptr->C_imu2vehicle * state_ptr->lever_imu2gnss);
            Eigen::Vector3d lla_antenna_mapbias_ = lla_antenna_ + state_ptr->Mpv * state_ptr->sdmap_bias_enu;
            loc_logger.logLocData({
                //
                lla_antenna_.x() * 180.0 / M_PI,                 // 融合纬度
                lla_antenna_.y() * 180.0 / M_PI,                 // 融合经度
                lla_antenna_mapbias_.x() * 180.0 / M_PI,         // 融合纬度+mapbias
                lla_antenna_mapbias_.y() * 180.0 / M_PI,         // 融合经度+mapbias
                pre_gnss_data_ptr_->lla.x() * 180.0 / M_PI,      // 卫星纬度
                pre_gnss_data_ptr_->lla.y() * 180.0 / M_PI,      // 卫星经度
                state_ptr->sdmap_proj.x() * 180.0 / M_PI,        // SD投影纬度
                state_ptr->sdmap_proj.y() * 180.0 / M_PI,        // SD投影经度
                state_ptr->sdmap_proj_mid_db.x() * 180.0 / M_PI, // DB中心投影纬度
                state_ptr->sdmap_proj_mid_db.y() * 180.0 / M_PI, // DB中心投影经度
                (int)state_ptr->rtk_status,                      // 卫星状态
                (int)state_ptr->align_type,                      // 对准状态
                state_ptr->measurement_timestamp                 // 融合时间戳
            });
        }
    }
#endif

    {
        // 这里渐消sdmapbias
        map_processor_->MapBiasFadingSppMode(dt_, state_ptr);
        // 渐消db投影distance
        db_processor_->FadingUpdate(state_ptr);
    }

    {
        static uint64_t imu_frame_count_ = 0;
        if (imu_frame_count_ % 6000 == 0 && state_ptr->measurement_timestamp != 0.0) {
            // 每6000个IMU更新(约1min一次)，记录一次内部状态到LOG文件
            AINFO << " " << state_debug_info(state_ptr);
        }
        imu_frame_count_++;
    }

    {
        if (
            std::fabs(pre_imu_data_ptr_->measurement_timestamp) > 1.0 &&                                       //
            std::fabs(pre_imu_data_ptr_->measurement_timestamp - imu_data_ptr->measurement_timestamp) > 3.0 && //
            process_control_sgt.msf_align_type != process_control_sgt.INITIALIZATION                           //
        ) {
            AERROR << "imu large delay detected";
            process_control_sgt.sensor_dt_state.imu_ok = false;
            process_control_sgt.sensor_issue_detected  = true;
        }
        if (std::fabs(pre_imu_data_ptr_->measurement_timestamp - imu_data_ptr->measurement_timestamp) < 0.1) {
            process_control_sgt.sensor_dt_state.imu_ok = true;
        }
    }
    pre_imu_data_ptr_ = std::make_shared<ImuData>(*imu_data_ptr);

    // 将imu测量值持续放到一个队列中
    initializer_->AddImuData(imu_data_ptr, dt_);

    // 首次初始化阶段，返回false
    if (process_control_sgt.msf_align_type == process_control_sgt.INITIALIZATION && process_control_sgt.initialization_count == 0) {
        return false;
    }

    // 预测更新
    imu_processor_->Predict(imu_data_ptr, dt_, state_ptr, kf_ptr_);

    // // 通过IMU更新逐步反馈GNSS位置新息，以平滑轨迹
    // // TODO：呃，临时措施，后续可能不采用该方式
    // state_ptr->ctl_count++;
    // if (state_ptr->ctl_count >= std::round(parameters_sgt.get_gnss_data_refresh_dt() / parameters_sgt.get_imu_data_refresh_dt())) {
    //     state_ptr->ctl.setZero();
    // }

    // 如果是零速状态，则进行零速更新
    if (process_control_sgt.vehicle_motion_type == process_control_sgt.ZERO_VELOCITY) {
        zupt_processor_->UpdateStateByZupt(state_ptr, kf_ptr_);
        imu_processor_->Feedback(state_ptr, true);
    }

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
    static uint64_t sequence_num_ = 0;
    double gpst_imu = state_ptr->measurement_timestamp - process_control_sgt.last_valid_dt;
    u_int32_t gpst_imu_i = std::round(gpst_imu * 1000);
    double gpst_imu_100 = std::round((double(gpst_imu_i)) / 10) / 100;
    sequence_num_++;
    if (sequence_num_ % 20 == 0) {
        Eigen::Matrix<double, 21, 1> cov_std;
        cov_std = state_ptr->error_state_std;

        auto            vehicle_state_ptr = std::make_shared<MSF::State>(state_ptr->ToVehicle().WithSdmapBias());
        Eigen::Vector3d eulr_             = vehicle_state_ptr->eulr_ * 180.0 / M_PI;
        Eigen::Vector3d ego_vel_          = vehicle_state_ptr->att.toRotationMatrix().transpose() * vehicle_state_ptr->vel;

        double lat_mars = 0.0, lon_mars = 0.0;
        // wgtochina_lb(0, vehicle_state_ptr->lla.y() * 180.0 / M_PI, vehicle_state_ptr->lla.x() * 180.0 / M_PI, vehicle_state_ptr->lla.z(), 0, 0, &lon_mars, &lat_mars);
        lon_mars = vehicle_state_ptr->lla.y() * 180.0 / M_PI;
        lat_mars = vehicle_state_ptr->lla.x() * 180.0 / M_PI;

        Eigen::Matrix3d C_veh2ref    = state_ptr->C_b2n * state_ptr->C_imu2vehicle.transpose();
        Eigen::Vector3d lla_imu_     = state_ptr->lla;
        Eigen::Vector3d lla_antenna_ = state_ptr->lla + (state_ptr->MpvCnb * state_ptr->C_imu2vehicle * state_ptr->lever_imu2gnss);
        Eigen::Vector3d lla_vehicle_ = state_ptr->lla + (state_ptr->MpvCnb * state_ptr->C_imu2vehicle * state_ptr->lever_imu2vehicle);

        double gnss_antenna_lat_mars = 0.0, gnss_antenna_lon_mars = 0.0;
        double vehicle_lat_mars = 0.0, vehicle_lon_mars = 0.0;
        double imu_lat_mars = 0.0, imu_lon_mars = 0.0;
        // wgtochina_lb(0, lla_antenna_.y() * 180.0 / M_PI, lla_antenna_.x() * 180.0 / M_PI, lla_antenna_.z(), 0, 0, &gnss_antenna_lon_mars, &gnss_antenna_lat_mars);
        // wgtochina_lb(0, lla_vehicle_.y() * 180.0 / M_PI, lla_vehicle_.x() * 180.0 / M_PI, lla_vehicle_.z(), 0, 0, &vehicle_lon_mars, &vehicle_lat_mars);
        // wgtochina_lb(0, lla_imu_.y() * 180.0 / M_PI, lla_imu_.x() * 180.0 / M_PI, lla_imu_.z(), 0, 0, &imu_lon_mars, &imu_lat_mars);
        gnss_antenna_lon_mars = lla_antenna_.y() * 180.0 / M_PI;
        gnss_antenna_lat_mars = lla_antenna_.x() * 180.0 / M_PI;
        vehicle_lon_mars      = lla_vehicle_.y() * 180.0 / M_PI;
        vehicle_lat_mars      = lla_vehicle_.x() * 180.0 / M_PI;
        imu_lon_mars          = lla_imu_.y() * 180.0 / M_PI;
        imu_lat_mars          = lla_imu_.x() * 180.0 / M_PI;

        Eigen::Vector3d vis_lla_antenna_          = -state_ptr->Mpv * C_veh2ref * state_ptr->vis_inno_ + lla_antenna_;
        double          vis_lla_antenna_lat_mars_ = 0.0, vis_lla_antenna_lon_mars_ = 0.0;
        // wgtochina_lb(0, vis_lla_antenna_.y() * 180.0 / M_PI, vis_lla_antenna_.x() * 180.0 / M_PI, vis_lla_antenna_.z(), 0, 0, &vis_lla_antenna_lon_mars_, &vis_lla_antenna_lat_mars_);
        vis_lla_antenna_lon_mars_ = vis_lla_antenna_.y() * 180.0 / M_PI;
        vis_lla_antenna_lat_mars_ = vis_lla_antenna_.x() * 180.0 / M_PI;

        std::string state_str = fmt::format(
            "{:>14.3f},{:>14.10f},{:>14.10f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f}",
            vehicle_state_ptr->measurement_timestamp,  // 1  量测时间
            vehicle_state_ptr->lla.x() * 180.0 / M_PI, // 2  纬度
            vehicle_state_ptr->lla.y() * 180.0 / M_PI, // 3  经度
            vehicle_state_ptr->lla.z(),                // 4  高度
            eulr_.x(),                                 // 5  俯仰
            eulr_.y(),                                 // 6  横滚
            eulr_.z(),                                 // 7  航向
            vehicle_state_ptr->vel.x(),                // 8  东向速度
            vehicle_state_ptr->vel.y(),                // 9  北向速度
            vehicle_state_ptr->vel.z(),                // 10 天向速度
            vehicle_state_ptr->gyro_bias.x(),          // 11 陀螺零偏
            vehicle_state_ptr->gyro_bias.y(),          // 12
            vehicle_state_ptr->gyro_bias.z(),          // 13
            vehicle_state_ptr->acc_bias.x(),           // 14 加计零偏
            vehicle_state_ptr->acc_bias.y(),           // 15
            vehicle_state_ptr->acc_bias.z(),           // 16
            vehicle_state_ptr->vehicle_bias.x(),       // 17 俯仰安装角误差
            vehicle_state_ptr->vehicle_bias.y(),       // 18 轮速系数误差
            vehicle_state_ptr->vehicle_bias.z()        // 19 航向安装角误差
        );
        std::string cov_std_str = fmt::format(
            "{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>14.14f},{:>14.14f},{:>14.14f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f}",
            cov_std[0],  // 20 姿态
            cov_std[1],  // 21
            cov_std[2],  // 22
            cov_std[3],  // 23 速度
            cov_std[4],  // 24
            cov_std[5],  // 25
            cov_std[6],  // 26 位置
            cov_std[7],  // 27
            cov_std[8],  // 28
            cov_std[9],  // 29 陀螺零偏
            cov_std[10], // 30
            cov_std[11], // 31
            cov_std[12], // 32 加计零偏
            cov_std[13], // 33
            cov_std[14], // 34
            cov_std[15], // 35 俯仰安装角误差
            cov_std[16], // 36 轮速系数误差
            cov_std[17], // 37 航向安装角误差
            cov_std[18], // 38
            cov_std[19], // 39
            cov_std[20]  // 40
        );
        std::string additional_info = fmt::format(
            "{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>14.10f},{:>14.10f},{:>6.4f},{:>6.4f},"
            "{:>6.4f},{:>6.4f},{:d},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},"
            "{:>14.10f},{:>14.10f},{:>14.10f},{:>14.10f},{:>14.10f},{:>14.10f},"
            "{:d},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:d},"
            "{:>14.10f},{:>14.10f},{:>6.4f},{:>14.10f},{:>14.10f},{:>6.4f},{:>6.4f},{:d},{:>6.4f},{:>9.2f}",
            vehicle_state_ptr->map_bias.x(),                               // 41
            vehicle_state_ptr->map_bias.y(),                               // 42
            vehicle_state_ptr->map_bias.z() * 180 / M_PI,                  // 43
            ego_vel_.x(),                                                  // 44
            ego_vel_.y(),                                                  // 45
            ego_vel_.z(),                                                  // 46
            lat_mars,                                                      // 47 融合输出加偏纬度（后轴,带制图误差）
            lon_mars,                                                      // 48 融合输出加偏经度
            vehicle_state_ptr->vel_ego.x(),                                // 49
            vehicle_state_ptr->vel_ego.y(),                                // 50
            vehicle_state_ptr->acc.x(),                                    // 51
            vehicle_state_ptr->acc.y(),                                    // 52
            (int)MSF::process_control_sgt.msf_align_type,                  // 53
            vehicle_state_ptr->vel.norm(),                                 // 54
            vehicle_state_ptr->gyro.x(),                                   // 55
            vehicle_state_ptr->gyro.y(),                                   // 56
            vehicle_state_ptr->acc.block<2, 1>(0, 0).norm(),               // 57
            vehicle_state_ptr->gyro.norm(),                                // 58
            vehicle_state_ptr->acc_lp_.block<2, 1>(0, 0).norm(),           // 59
            vehicle_state_ptr->gyro_lp_.norm(),                            // 60
            vehicle_state_ptr->gyro.z(),                                   // 61
            gnss_antenna_lat_mars,                                         // 62 天线加偏纬度（不带制图误差）
            gnss_antenna_lon_mars,                                         // 63 天线加偏经度
            vehicle_lat_mars,                                              // 64 后轴加偏纬度（不带制图误差）
            vehicle_lon_mars,                                              // 65 后轴加偏经度
            imu_lat_mars,                                                  // 66 IMU加偏纬度（不带制图误差）
            imu_lon_mars,                                                  // 67 IMU加偏经度
            (int)MSF::process_control_sgt.vehicle_motion_type,             // 68 运动状态
            state_ptr->gnss_pos_inno_statistics_(0),                       // 69 pos
            state_ptr->gnss_pos_inno_statistics_(1),                       // 70
            state_ptr->gnss_pos_inno_statistics_.block<2, 1>(0, 0).norm(), // 71
            state_ptr->gnss_pos_inno_statistics_(3),                       // 72 pos mean
            state_ptr->gnss_pos_inno_statistics_(4),                       // 73
            state_ptr->gnss_pos_inno_statistics_.block<2, 1>(3, 0).norm(), // 74
            state_ptr->gnss_pos_inno_statistics_(6),                       // 75 pos std
            state_ptr->gnss_pos_inno_statistics_(7),                       // 76
            state_ptr->gnss_pos_inno_statistics_.block<2, 1>(6, 0).norm(), // 77
            state_ptr->rtk_status,                                         // 78
            vis_lla_antenna_lat_mars_,                                     // 79 视觉修正量投影到卫星天线
            vis_lla_antenna_lon_mars_,                                     // 80 视觉修正量投影到卫星天线
            vis_lla_antenna_.z(),                                          // 81
            lla_antenna_.x() * 180 / M_PI,                                 // 82 天线位置，不加偏
            lla_antenna_.y() * 180 / M_PI,                                 // 83
            lla_antenna_.z(),                                              // 84
            vehicle_state_ptr->acc.z(),                                    // 85
            vehicle_state_ptr->tire_slip,                                  // 86
            state_ptr->state_stability_index_ratio_,                        // 87
            gpst_imu_100

        );
        debug::debug_sgt.msf_state.line(state_str + "," + cov_std_str + "," + additional_info + "\n");
    }
#endif

    // // 达到过对准状态，输出True，否则维持False
    // if (state_ptr->tcmsf_aligned_count > 0) {
    //     return true;
    // } else {
    //     return false;
    // }

    bool is_tcmsf_initialized_ = init_traj_analysis_.isFirstTimeInitializationReady();

    state_ptr->tcmsf_1st_time_initialized_ = is_tcmsf_initialized_;

    bool is_self_test_passed_ = isStateReasonable(state_ptr);

    // 融合定位ready能够输出的条件
    bool is_tcmsf_ready = is_tcmsf_initialized_    // 融合定位完成初始化
                          && is_self_test_passed_; // 自检通过，检测NaN值和合理性

    return is_tcmsf_ready;
}

void ProcessorImpl::BacktrackState(StatePtr msf_state_ptr, const KinematicDataPtr delta_ptr, bool enable_vel_compesation) {
    // DR序列坐标系为前左上
    // 将状态拉到GNSS量测时间
    msf_state_ptr->measurement_timestamp = msf_state_ptr->measurement_timestamp - delta_ptr->measurement_timestamp;
    Eigen::Quaterniond dq{INS::frame_trans.FLU2RFU * delta_ptr->att.toRotationMatrix() * INS::frame_trans.FLU2RFU.transpose()};
    msf_state_ptr->att        = dq.conjugate() * msf_state_ptr->att;
    Eigen::Matrix3d R_rfu2enu = msf_state_ptr->att.toRotationMatrix();
    Eigen::Vector3d dpos      = -R_rfu2enu * INS::frame_trans.FLU2RFU * delta_ptr->pos;
    if (enable_vel_compesation) { // 速度补偿容易耦合航向误差，加个是否补偿速度的控制
        msf_state_ptr->vel = msf_state_ptr->vel - R_rfu2enu * INS::frame_trans.FLU2RFU * delta_ptr->vel;
    }
    auto lla           = geotrans.enu2blh({msf_state_ptr->lla.x() * 180.0 / M_PI, msf_state_ptr->lla.y() * 180.0 / M_PI, msf_state_ptr->lla.z()}, {dpos.x(), dpos.y(), dpos.z()});
    msf_state_ptr->lla = Eigen::Vector3d{lla.b / 180.0 * M_PI, lla.l / 180.0 * M_PI, lla.h};
}

void ProcessorImpl::AdvanceState(StatePtr msf_state_ptr, const KinematicDataPtr delta_ptr, bool enable_vel_compesation) {
    // 将状态拉回IMU量测时间
    msf_state_ptr->measurement_timestamp = msf_state_ptr->measurement_timestamp + delta_ptr->measurement_timestamp;
    Eigen::Matrix3d R_rfu2enu            = msf_state_ptr->att.toRotationMatrix();
    Eigen::Vector3d dpos                 = R_rfu2enu * INS::frame_trans.FLU2RFU * delta_ptr->pos;
    auto            lla                  = geotrans.enu2blh({msf_state_ptr->lla.x() * 180.0 / M_PI, msf_state_ptr->lla.y() * 180.0 / M_PI, msf_state_ptr->lla.z()}, {dpos.x(), dpos.y(), dpos.z()});
    msf_state_ptr->lla                   = Eigen::Vector3d{lla.b / 180.0 * M_PI, lla.l / 180.0 * M_PI, lla.h};
    Eigen::Quaterniond dq{INS::frame_trans.FLU2RFU * delta_ptr->att.toRotationMatrix() * INS::frame_trans.FLU2RFU.transpose()};
    msf_state_ptr->att = dq * msf_state_ptr->att;
    if (enable_vel_compesation) { // 速度补偿容易耦合航向误差，加个是否补偿速度的控制
        msf_state_ptr->vel = msf_state_ptr->vel + R_rfu2enu * INS::frame_trans.FLU2RFU * delta_ptr->vel;
    }
}

bool ProcessorImpl::ProcessGnssData(const GnssDataPtr gps_data_ptr, StatePtr state_ptr, const KinematicDataPtr delta_ptr, const KinematicDataPtr dr_ptr) {

    // 执行卫星信息自洽性验证，使用10帧间隔进行逐帧连续判断
    auto consistency_result = gnss_consistency_validator_.insert_and_validate(gps_data_ptr);
    // 将一致性系数写入state_ptr，默认-1表示不可用
    if (consistency_result.is_valid) {
        state_ptr->gnss_consistency_score_ = consistency_result.consistency_score;
    } else {
        state_ptr->gnss_consistency_score_ = -1.0;
    }

    Eigen::Vector3d raw_gnss_pos_std_ = gps_data_ptr->lla_cov;

    // RTK LC 和 GNSS LC 两种模式，进行自适应切换
    if (parameters_sgt.get_gnss_fusion_mode() == parameters_sgt.GNSS_LOOSE_COUPLE ||
        parameters_sgt.get_gnss_fusion_mode() == parameters_sgt.RTK_LOOSE_COUPLE) {
        gnss_fusion_mode_adaption.update(gps_data_ptr, state_ptr);
        auto recommend_gnss_fusion_mode_ = gnss_fusion_mode_adaption.recommend_mode();
        if (parameters_sgt.get_gnss_fusion_mode_adaption()) {
            if (recommend_gnss_fusion_mode_ != parameters_sgt.get_gnss_fusion_mode()) {
                AWARN << "[gnss fusion mode adaption] switch: " << (uint64_t)parameters_sgt.get_gnss_fusion_mode() << " to " << (uint64_t)recommend_gnss_fusion_mode_;
                parameters_sgt.set_gnss_fusion_mode(recommend_gnss_fusion_mode_);
            }
        }
    }
    pre_gnss_data_ptr_ = std::make_shared<GnssData>(*gps_data_ptr);
    {
        if (process_control_sgt.sensor_issue_detected && process_control_sgt.sensor_dt_state.imu_ok && process_control_sgt.sensor_dt_state.wheel_ok && gps_data_ptr->vel.norm() > parameters_sgt.get_gnss_minimum_speed_required_for_initialization()) {
            AERROR << "reinit for sensor delay issue, clear issue flag";
            process_control_sgt.msf_align_type                  = process_control_sgt.INITIALIZATION;
            process_control_sgt.reinitialization_state.position = true;
            process_control_sgt.reinitialization_state.velocity = true;
            process_control_sgt.reinitialization_state.heading  = true;
            process_control_sgt.sensor_issue_detected           = false;
        }
    }

    if (process_control_sgt.msf_align_type == process_control_sgt.INITIALIZATION || !parameters_sgt.get_enable_gnss_fusion()) {
        AINFO_EVERY(100)
            << "Not fusing gnss, reason: "
            << ((process_control_sgt.msf_align_type == process_control_sgt.INITIALIZATION) ? "<init> " : "")
            << (!parameters_sgt.get_enable_gnss_fusion() ? "<gnss fusion is disabled> " : "");

        // 初始化阶段，不进行量测更新
        // 或者不启用GNSS融合，不进行量测更新（实际上必须启用GNSS融合，要不然无法输出有效信息）
        // 进行初始化逻辑
        if (!initializer_->AddGnssData(gps_data_ptr, state_ptr, kf_ptr_, delta_ptr)) {
            return false;
        }

        imu_processor_->Feedback(state_ptr, true);

    } else {
        // 进行GNSS量测更新

        bool enable_vel_compesation = std::abs(state_ptr->acc.y()) > 0.1 && state_ptr->vel.norm() > 1.0;

        // 如果存在补偿（不为nullptr），进行补偿
        if (delta_ptr) {
            // 状态回溯
            BacktrackState(state_ptr, delta_ptr, enable_vel_compesation);
        }

        {
            // 这里检测是否完成初始化
            double   m_time_          = gps_data_ptr->measurement_timestamp;
            auto     ins_antenna_blh_ = state_ptr->GetAntennaBLH();
            auto     rtk_antenna_blh_ = gps_data_ptr->lla;
            double   ins_heading_     = -state_ptr->eulr_.z() * 180.0 / M_PI;
            uint64_t rtk_status_      = gps_data_ptr->status;
            if (ins_heading_ < 0) {
                ins_heading_ = ins_heading_ + 360;
            }
            Eigen::Quaterniond att_ = state_ptr->att * state_ptr->q_imu2vehicle.conjugate();
            init_traj_analysis_.Analysis(m_time_, ins_antenna_blh_, rtk_antenna_blh_, att_, rtk_status_, ins_heading_, std::fabs(state_ptr->acc.y()));
        }

        switch (parameters_sgt.get_gnss_fusion_mode()) {

            case parameters_sgt.GNSS_LOOSE_COUPLE: {
                gps_single_processor_->UpdateStateByGpsSingle(gps_data_ptr, state_ptr, kf_ptr_, dr_ptr);
            } break;

            case parameters_sgt.GNSS_TIGHT_COUPLE: {
            } break;

            case parameters_sgt.RTK_LOOSE_COUPLE: {
                gps_processor_->UpdateStateByGps(gps_data_ptr, state_ptr, kf_ptr_, dr_ptr);
            } break;

            case parameters_sgt.RTK_TIGHT_COUPLE: {
            } break;

            default:
                break;
        }

        calcPosInnoRangeNorm(state_ptr, raw_gnss_pos_std_);

        if (delta_ptr) {
            // 状态对齐到最新时间
            AdvanceState(state_ptr, delta_ptr, enable_vel_compesation);
        }

        imu_processor_->Feedback(state_ptr, true);
    }
    return true;
}

bool ProcessorImpl::ProcessVehicleData(const VehicleDataPtr vehicle_data_ptr, StatePtr state_ptr, const KinematicDataPtr delta_ptr) {

    {
        if (
            std::fabs(pre_veh_data_ptr_->measurement_timestamp) > 1.0 &&                                            //
            std::fabs(pre_veh_data_ptr_->measurement_timestamp - vehicle_data_ptr->measurement_timestamp) > 20.0 && //
            process_control_sgt.msf_align_type != process_control_sgt.INITIALIZATION                                //
        ) {
            AERROR << "wheel large delay detected";
            process_control_sgt.sensor_dt_state.wheel_ok = false;
            process_control_sgt.sensor_issue_detected    = true;
        }
        if (std::fabs(pre_veh_data_ptr_->measurement_timestamp - vehicle_data_ptr->measurement_timestamp) < 0.1) {
            process_control_sgt.sensor_dt_state.wheel_ok = true;
        }
    }

    // auto &motion_status_ref = MSF::MotionStatus::GetInstance();
    // auto  motion_status     = motion_status_ref.GetMotionStatus();

    state_ptr->tire_slip = vehicle_data_ptr->tire_slip;

    // state_ptr 里面的gyro是未补偿零偏的，这里计算的时候做个补偿
    bool gyro_steady_ = (state_ptr->gyro - state_ptr->gyro_bias).norm() < 0.3 / 180.0 * M_PI;

    bool is_pls_count_unchanged = judge_zupt_by_pls_count(pre_veh_data_ptr_, vehicle_data_ptr);

    // 如果已经处于稳态的零速。
    // 比如4秒（20次计数对应1秒）。
    // 这个时候，为了准确地检测到载体的运动。
    // 速度的判断依据整严格些。
    // 这个严格判断最少持续200个周期（2秒）

    bool no_moving_after_steady_ = true;
    {
        constexpr uint64_t MAX_CIRCLE_ = 200;

        static uint64_t steady_count_      = 0;
        static bool     start_steay_count_ = false;
        if (process_control_sgt.vehicle_motion_type == process_control_sgt.ZERO_VELOCITY && process_control_sgt.fusion_status.continous_zupt_imu_bias_estimate_count > 80) {
            start_steay_count_ = true;
        }
        if (steady_count_ >= MAX_CIRCLE_) {
            start_steay_count_ = false;
            steady_count_      = 0;
        }
        if (start_steay_count_) {
            steady_count_++;
        } else {
            steady_count_ = 0;
        }
        no_moving_after_steady_ = steady_count_ > 0 ? state_ptr->vel.norm() < 0.01 : true;
    }

    {

        // 判断方向盘转角角速度是否小量
        bool is_steer_steady_ = std::fabs(vehicle_data_ptr->steer_rate) < 1e-6;

        // 这里做个零速更新的保护逻辑
        // 如果轮速进入零速状态，持续1.0秒之后，融合的速度还没进入小量状态，则直接给速度置零

        // 使用轮速和陀螺的状态判断，车辆是否进入零速状态
        bool zero_spd_ =
            std::abs(vehicle_data_ptr->speed_rl + vehicle_data_ptr->speed_rr) < 1e-3 &&
            is_pls_count_unchanged &&
            gyro_steady_ &&
            (!vehicle_data_ptr->tire_slip) &&
            is_steer_steady_;

        zspd_switch_info.insert_state(zero_spd_, vehicle_data_ptr->measurement_timestamp);

        if (
            zero_spd_ &&                                                                                        //
            std::fabs(vehicle_data_ptr->measurement_timestamp - zspd_switch_info.get_into_timestamp()) > 1.0 && //
            state_ptr->vel.norm() >= 0.09                                                                       //
        ) {
            // 如果零速状态，并持续了1.0秒以上，并且融合速度还没进入小量状态，则直接把融合速度置零
            AWARN << "wheel stop for 1.0 sec, set vel to zero";
            state_ptr->vel.setZero();
        }
    }

    // 通过轮速判断是否进入零速状态

    // 大于100米认为里程超出附近范围
    // 稍微做点防护，额外要求最后一次零速更新里程大于10米
    bool out_zupt_nearby_ =
        (std::fabs(process_control_sgt.state_nearby_info.latest_zupt_info.mileage_nearby - state_ptr->mileage) > process_control_sgt.state_nearby_info.latest_zupt_info.NEARBY_RANGE) &&
        (std::fabs(process_control_sgt.state_nearby_info.latest_zupt_info.mileage_out_zupt - state_ptr->mileage) > 10.0);

    // ZUPT状态判断条件

    bool zspd_for_a_while_ =
        zspd_switch_info.get_current_state() &&                                                          //
        std::fabs(vehicle_data_ptr->measurement_timestamp - zspd_switch_info.get_into_timestamp()) > 0.1 //
        ;

    bool zupt_state_ =
        (state_ptr->vel_ego.block<2, 1>(0, 0).norm() < 0.1 || zspd_for_a_while_) && // 融合速度小量 或 轮速零状态一小会
        std::abs(vehicle_data_ptr->speed_rl + vehicle_data_ptr->speed_rr) < 1e-3 && // 轮速小量
        gyro_steady_ &&                                                             // 陀螺小量
        is_pls_count_unchanged &&                                                   // 脉冲计数不变
        (!vehicle_data_ptr->tire_slip) &&                                           // 非打滑
        no_moving_after_steady_                                                     // 静态稳定之后，速度极小量未运动
        ;

    if (zupt_state_) {
        // 设置零速标志
        AINFO_IF(process_control_sgt.vehicle_motion_type != process_control_sgt.ZERO_VELOCITY) << "enter zero speed state.";

        if (process_control_sgt.vehicle_motion_type != process_control_sgt.ZERO_VELOCITY && out_zupt_nearby_) {
            // 如果从动态进入静态且超出附近范围，则重置
            // 这里做这个逻辑，主要是为了应对长时间零速更新的情况
            process_control_sgt.state_nearby_info.latest_zupt_info.mileage_nearby   = state_ptr->mileage;
            process_control_sgt.state_nearby_info.latest_zupt_info.duration_nearby  = 0.0;
            process_control_sgt.state_nearby_info.latest_zupt_info.timestamp_nearby = state_ptr->measurement_timestamp;
        }

        process_control_sgt.vehicle_motion_type = process_control_sgt.ZERO_VELOCITY;

    } else {
        // 设置运动标志
        AINFO_IF(process_control_sgt.vehicle_motion_type != process_control_sgt.MOVING) << "quit zero speed state.";

        if (process_control_sgt.vehicle_motion_type != process_control_sgt.MOVING && !out_zupt_nearby_) {
            // 如果状态从静态进入动态且在附近距离内，触发更新
            process_control_sgt.state_nearby_info.latest_zupt_info.mileage_out_zupt   = state_ptr->mileage;
            process_control_sgt.state_nearby_info.latest_zupt_info.timestamp_out_zupt = state_ptr->measurement_timestamp;
            process_control_sgt.state_nearby_info.latest_zupt_info.duration_nearby    = state_ptr->measurement_timestamp - process_control_sgt.state_nearby_info.latest_zupt_info.timestamp_nearby;
        }

        process_control_sgt.vehicle_motion_type = process_control_sgt.MOVING;
        // 记录自车位、姿状态，以备零速更新时使用
        process_control_sgt.att0_zupt = state_ptr->att;
        process_control_sgt.lla0_zupt = state_ptr->lla;
    }

    pre_veh_data_ptr_ = std::make_shared<VehicleData>(*vehicle_data_ptr);

    if (vehicle_data_ptr->tire_slip) {
        // 如果轮胎打滑，轮速变得不可信，不进一步融合轮速
        return false;
    }

    // 首次初始化阶段，不进行轮速量测更新
    // 如果配置不融合轮速，不进行量测更新
    if ((process_control_sgt.msf_align_type == process_control_sgt.INITIALIZATION && process_control_sgt.initialization_count == 0) || !parameters_sgt.get_enable_wheel_vel_fusion()) {
        AINFO_EVERY(1000)
            << "Not fusing wheel, reason: "
            << ((process_control_sgt.msf_align_type == process_control_sgt.INITIALIZATION && process_control_sgt.initialization_count == 0) ? "<first init> " : "")
            << (!parameters_sgt.get_enable_wheel_vel_fusion() ? "<wheel fusion is disabled> " : "");
        return false;
    }

    // 量测更新
    if (process_control_sgt.vehicle_motion_type == process_control_sgt.ZERO_VELOCITY) {
        // 零速状态下，不进行轮速的零速更新
    } else {
        // 进行轮速的量测更新
        vehicle_processor_->UpdateStateByVehicle(vehicle_data_ptr, state_ptr, kf_ptr_, delta_ptr);
    }

    imu_processor_->Feedback(state_ptr, true);
    return true;
}

bool ProcessorImpl::ProcessVisionData(const VisionFusionDataPtr vis_data_ptr, StatePtr state_ptr, const KinematicDataPtr delta_ptr) {
    // 初始化阶段，不进行量测更新
    // 如果配置不融合视觉。不进行量测更新
    if ((process_control_sgt.msf_align_type == process_control_sgt.INITIALIZATION && process_control_sgt.initialization_count == 0) || !parameters_sgt.get_enable_vision_fusion()) {
        AINFO_EVERY(100)
            << "Not fusing vision, reason: "
            << ((process_control_sgt.msf_align_type == process_control_sgt.INITIALIZATION && process_control_sgt.initialization_count == 0) ? "<first init> " : "")
            << (!parameters_sgt.get_enable_vision_fusion() ? "<vision fusion is disabled> " : "");
        return false;
    }

    // if (delta_ptr) {
    //     Eigen::Quaterniond dq{INS::frame_trans.FLU2RFU * delta_ptr->att.toRotationMatrix() * INS::frame_trans.FLU2RFU.transpose()};
    //     Eigen::Matrix3d    dm    = dq.toRotationMatrix();
    //     vis_data_ptr->pos_offset = dm * vis_data_ptr->pos_offset;
    // }

    // 如果存在补偿（不为nullptr），进行补偿
    if (delta_ptr) {
        // 状态回溯
        BacktrackState(state_ptr, delta_ptr, true);
    }

    // 量测更新
    if (process_control_sgt.vehicle_motion_type == process_control_sgt.ZERO_VELOCITY) {
        // 零速状态下，不进行视觉的零速更新
    } else {
        // 进行视觉的量测更新
        vision_processor_->UpdateStateByVision(vis_data_ptr, state_ptr, kf_ptr_);
    }

    if (delta_ptr) {
        // 状态对齐到最新时间
        AdvanceState(state_ptr, delta_ptr, true);
    }

    imu_processor_->Feedback(state_ptr, true);
    return true;
}

bool ProcessorImpl::ProcessMapData(const MapPosDataPtr map_data_ptr, StatePtr state_ptr, const KinematicDataPtr delta_ptr) {
    // 更新状态里面的投影点坐标（无论是否融合都需要更新，用于日志记录）
    state_ptr->sdmap_proj = map_data_ptr->lla;

    // 初始化阶段，不进行量测更新
    // 如果配置不融合地图，不进行量测更新
    if ((process_control_sgt.msf_align_type == process_control_sgt.INITIALIZATION && process_control_sgt.initialization_count == 0) || !parameters_sgt.get_enable_map_fusion()) {
        AINFO_EVERY(100)
            << "Not fusing map, reason: "
            << ((process_control_sgt.msf_align_type == process_control_sgt.INITIALIZATION && process_control_sgt.initialization_count == 0) ? "<first init> " : "")
            << (!parameters_sgt.get_enable_map_fusion() ? "<map fusion is disabled> " : "");
        return false;
    }
    // 如果存在补偿（不为nullptr），进行补偿
    if (delta_ptr) {
        // 状态回溯
        BacktrackState(state_ptr, delta_ptr, true);
    }

    // 量测更新
    if (process_control_sgt.vehicle_motion_type == process_control_sgt.ZERO_VELOCITY) {
        // 零速状态下，不进行地图的零速更新
    } else {
        // 进行地图的量测更新
        map_processor_->UpdateStateByMap(map_data_ptr, state_ptr, kf_ptr_);
    }

    if (delta_ptr) {
        // 状态对齐到最新时间
        AdvanceState(state_ptr, delta_ptr, true);
    }

    imu_processor_->Feedback(state_ptr, true);
    return true;
}

bool ProcessorImpl::ProcessDbData(const DbDataPtr db_data_ptr, StatePtr state_ptr) {
    // 初始化阶段，不处理
    // 如果配置不融合DB，不进行量测更新
    if ((process_control_sgt.msf_align_type == process_control_sgt.INITIALIZATION && process_control_sgt.initialization_count == 0) || !parameters_sgt.get_enable_db_fusion()) {
        AINFO_EVERY(100)
            << "Not fusing db, reason: "
            << ((process_control_sgt.msf_align_type == process_control_sgt.INITIALIZATION && process_control_sgt.initialization_count == 0) ? "<first init> " : "")
            << (!parameters_sgt.get_enable_db_fusion() ? "<db fusion is disabled> " : "");
        return false;
    }
    db_processor_->ProcessDbData(db_data_ptr, state_ptr);
    return true;
}

// 根据加计信息，为姿态提供参考。保证滤波算法即使是在无GNSS条件下也能够长期不发散。
void ProcessorImpl::AttitudeReference(const ImuData &imu_data, const VehicleData &vehicle_data, double dt_, StatePtr state_ptr, const KinematicDataPtr inner_motion_state_ptr) { //
    att_referencer_->AttRef(imu_data, vehicle_data, dt_, state_ptr, inner_motion_state_ptr);
}

bool ProcessorImpl::judge_zupt_by_pls_count(const VehicleDataPtr pre_msg_, const VehicleDataPtr cur_msg_) {
    if (pre_msg_->rl_pls_cnt == cur_msg_->rl_pls_cnt && pre_msg_->rr_pls_cnt == cur_msg_->rr_pls_cnt) {
        zupt_count_by_pls++;
    } else {
        zupt_count_by_pls = 0;
    }
    if (zupt_count_by_pls > 5) {
        return true;
    } else {
        return false;
    }
}

bool ProcessorImpl::isStateReasonable(const StatePtr state_ptr_) {
    bool NaN_detected_            = state_ptr_->isNaN();
    bool velocity_not_reasonable_ = state_ptr_->vel.norm() > 100.0;
    bool pitch_not_reasonable_    = std::fabs(state_ptr_->eulr_.x()) > 60.0 / 180.0 * M_PI;
    bool roll_not_reasonable_     = std::fabs(state_ptr_->eulr_.y()) > 60.0 / 180.0 * M_PI;

    bool state_not_reasonable_ = NaN_detected_ ||
                                 velocity_not_reasonable_ ||
                                 pitch_not_reasonable_ ||
                                 roll_not_reasonable_;

    AWARN_IF(state_not_reasonable_) << "TCMSFS's state is not reasonable. time: " << fmt::format("{:14.4f}", state_ptr_->measurement_timestamp);

    return !state_not_reasonable_;
}

std::string ProcessorImpl::state_debug_info(const StatePtr state_ptr) {
    std::string info_str_ = "";

    info_str_ =                                                                   //
        fmt::format(                                                              //
            "{:>14.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},"                      //
            "{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},"                       //
            "{:d},{:>3.2f},{:d},{:d}",                                            //
            state_ptr->measurement_timestamp,                                     //  1 量测时间
            state_ptr->gyro_bias.x() * 180.0 / M_PI,                              //  2 陀螺零偏 R
            state_ptr->gyro_bias.y() * 180.0 / M_PI,                              //  3 陀螺零偏 F
            state_ptr->gyro_bias.z() * 180.0 / M_PI,                              //  4 陀螺零偏 U
            state_ptr->acc_bias.x(),                                              //  5 加计零偏 R
            state_ptr->acc_bias.y(),                                              //  6 加计零偏 F
            state_ptr->acc_bias.z(),                                              //  7 加计零偏 U
            state_ptr->vehicle_bias.x() * 180.0 / M_PI,                           //  8 IMU安装角误差 Pitch
            state_ptr->vehicle_bias.y(),                                          //  9 轮速系数误差
            state_ptr->vehicle_bias.z() * 180.0 / M_PI,                           // 10 IMU安装角误差 Yaw
            (int)MSF::process_control_sgt.msf_align_type,                         // 11 对准状态
            state_ptr->state_stability_index_ratio_,                              // 12 稳定性指数
            (int)MSF::process_control_sgt.rtk_overall_status,                     // 13 RTK整体状态评估
            MSF::process_control_sgt.fusion_status.imu_update_count_after_rtk_fix // 14 距RTK最后一次FIX，预测更新次数
        );

    return info_str_;
}

ProcessorImpl::~ProcessorImpl() { //
    AINFO << "exit ProcessorImpl";
    // imu_processor_ = nullptr;
}

void ProcessorImpl::calcPosInnoRangeNorm(StatePtr state_ptr, const Eigen::Vector3d &pos_std_) {

    double h_pos_std_       = pos_std_.block<2, 1>(0, 0).norm();
    double h_pos_std_scale_ = 1.0 / (0.5 + h_pos_std_);

    double h_spd_ = state_ptr->vel.block<2, 1>(0, 0).norm();

    // 这里设计一下：100m 收敛差值的 0.6
    // 1e-2对应100米，0.1对应GNSS的更新周期，h_spd_对应速度权重
    // 另外考虑卫星量测的std权重
    double scale_ = h_spd_ * 1.0e-2 * 0.1 * h_pos_std_scale_;
    scale_        = scale_ > 1e-1 ? 1e-1 : scale_;

    double pos_inno_norm_ = state_ptr->pos_rfu_inno_.block<2, 1>(0, 0).norm();

    // 对位置新息做一个限制，设置上限为 1e3
    pos_inno_norm_ = pos_inno_norm_ > 1e3 ? 1e3 : pos_inno_norm_;

    state_ptr->pos_rfu_inno_range_norm_ = state_ptr->pos_rfu_inno_range_norm_ * (1.0 - scale_) + pos_inno_norm_ * scale_;

    AINFO_EVERY(100) << "pos inno range norm: " << state_ptr->pos_rfu_inno_range_norm_;

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
    {
        std::string str_ = "";

        str_ = fmt::format( //
            "{:>14.3f},{:>14.10f},{:>14.10f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f}\n",
            state_ptr->measurement_timestamp,   // 1 量测时间
            state_ptr->lla.x() * 180.0 / M_PI,  // 2 纬度
            state_ptr->lla.y() * 180.0 / M_PI,  // 3 经度
            state_ptr->lla.z(),                 // 4 高度
            h_pos_std_,                         // 5 平面位置std
            h_spd_,                             // 6 平面速度
            pos_inno_norm_,                     // 7 平面位置新息
            state_ptr->pos_rfu_inno_range_norm_ // 8 平面位置新息距离加权
        );
        debug::debug_sgt.pos_inno_range_norm.line(str_);
    }
#endif
}

GnssFusionModeAdaption::GnssFusionModeAdaption() {

    // 默认初值为零
    pre_gnss_timestamp      = 0.0;
    pre_gnss_status         = 0;
    continuous_gnss_mileage = 0.0;
    continuous_rtk_mileage  = 0.0;

    auto &parameters_sgt  = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);
    recommend_fusion_mode = parameters_sgt.get_gnss_fusion_mode();
}

void GnssFusionModeAdaption::update(const GnssDataPtr gps_data_ptr_, const StatePtr state_ptr_) {

    // 更新逻辑：
    // 单点解持续里程超过一定值(eg. 1000m) -> 推荐单点解融合模式
    // 固定解持续里程超过一定值(eg. 50m) -> 推荐RTK融合模式
    // 状态跳变、消息不连续、更新推荐模式之后，清空里程计算
    constexpr static double GNSS_CONTINUOUS_DT_BOUND = 1.0;

    double   cur_gnss_timestamp = gps_data_ptr_->measurement_timestamp;
    uint64_t cur_gnss_status    = gps_data_ptr_->raw_status;

    double gnss_dt = std::fabs(cur_gnss_timestamp - pre_gnss_timestamp);
    double cur_spd = state_ptr_->vel.norm();

    if (gnss_dt < GNSS_CONTINUOUS_DT_BOUND && pre_gnss_status == cur_gnss_status) {
        // 只有卫星前后帧状态一致、时间间隔满足条件，才计算累积里程
        // 卫星定位为FIX，表明RTK状态良好，累积RTK模式里程
        if (cur_gnss_status == 6) {
            continuous_rtk_mileage = continuous_rtk_mileage + cur_spd * gnss_dt;
        }
        // 卫星定位为SINGLE或者DGPS状态均累积SPP模式里程
        if (cur_gnss_status == 2 || cur_gnss_status == 3) {
            continuous_gnss_mileage = continuous_gnss_mileage + cur_spd * gnss_dt;
        }
    } else {
        // 如果不满足，则认为卫星状态不好，里程清零
        clear_mileage();
    }

    pre_gnss_timestamp = gps_data_ptr_->measurement_timestamp;
    pre_gnss_status    = gps_data_ptr_->raw_status;

    if (continuous_gnss_mileage > MILEAGE_SWITCH_TO_GNSS_LC_MODE) {
        // 连续里程超过单点解条件，更新推荐融合模式为单点解融合，并清零里程
        recommend_fusion_mode = Parameters::GnssFusionMode::GNSS_LOOSE_COUPLE;
        clear_mileage();
    }

    if (continuous_rtk_mileage > MILEAGE_SWITCH_TO_RTK_LC_MODE) {
        // 连续里程超过RTK条件，更新推荐融合模式为RTK融合，并清零里程
        recommend_fusion_mode = Parameters::GnssFusionMode::RTK_LOOSE_COUPLE;
        clear_mileage();
    }
}

Parameters::GnssFusionMode GnssFusionModeAdaption::recommend_mode() {
    return recommend_fusion_mode;
}

void GnssFusionModeAdaption::clear_mileage() {
    continuous_gnss_mileage = 0.0;
    continuous_rtk_mileage  = 0.0;
}

TrajAnalysis::TrajAnalysis() {
    FirstTimeInitializationReady = false;
}

bool TrajAnalysis::isFirstTimeInitializationReady() {
    return FirstTimeInitializationReady;
}

void TrajAnalysis::resetFirstTimeInitialization() {
    FirstTimeInitializationReady = false;
}

void TrajAnalysis::setFirstTimeInitialization() {
    FirstTimeInitializationReady = true;
}

void TrajAnalysis::Analysis(double m_time_, const Eigen::Vector3d &ins_blh_antenna_, const Eigen::Vector3d &rtk_blh_, const Eigen::Quaterniond &att_, uint64_t rtk_status_, double ins_heading_, double long_acc_) {

    // 如果已经完成初始化，则直接返回
    if (FirstTimeInitializationReady) {
        return;
    }

    double rtk_traj_heading = local_trans.HeadingFromBLH(pre_pos.rtk_blh, rtk_blh_);
    double ins_traj_heading = local_trans.HeadingFromBLH(pre_pos.ins_blh, ins_blh_antenna_);

    double dp_ins = local_trans.BLH2ENU(pre_pos.ins_blh, ins_blh_antenna_).block<2, 1>(0, 0).norm();

    Eigen::Vector3d delta_blh_in_meters_ = local_trans.BLH2ENU(rtk_blh_, ins_blh_antenna_);
    Eigen::Vector3d delta_rfu_in_meters_ = att_.toRotationMatrix().transpose() * delta_blh_in_meters_;

    auto cur_pos = POS_INFO{m_time_, ins_blh_antenna_, rtk_blh_, delta_rfu_in_meters_, ins_heading_, ins_traj_heading, rtk_traj_heading, dp_ins, rtk_status_};

    traj_queue.emplace_back(cur_pos);
    pre_pos = cur_pos;

    while (traj_queue.size() > MAX_QUEUE_SIZE) {
        traj_queue.pop_front();
    }

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
    {
        double   measure_time_ = cur_pos.measurement_timestamp;
        double   dp_           = cur_pos.dp_ins;
        double   dpinno_       = 0.0;
        double   dblhinno_     = cur_pos.delta_blh_in_meters.block<2, 1>(0, 0).norm();
        double   ins_heading_  = cur_pos.ins_traj_heading;
        double   rtk_heading_  = cur_pos.rtk_traj_heading;
        double   dhgd_         = ins_heading_ - rtk_heading_;
        uint64_t rtk_status_   = cur_pos.rtk_status;
        double   dp_r_         = cur_pos.delta_blh_in_meters.x();
        double   dp_f_         = cur_pos.delta_blh_in_meters.y();

        std::string init_info_str =
            fmt::format(
                "{:14.4f},{:4.3f},{:4.3f},{:4.3f},{:4.3f},{:4.3f},{:4.3f},{:d},{:d},{:4.3f},{:4.3f},{:4.3f}\n ",
                measure_time_,                // 1
                dp_,                          // 2
                dpinno_,                      // 3
                dblhinno_,                    // 4
                ins_heading_,                 // 5
                rtk_heading_,                 // 6
                dhgd_,                        // 7
                rtk_status_,                  // 8
                FirstTimeInitializationReady, // 9
                long_acc_,                    // 10
                dp_r_,                        // 11
                dp_f_                         // 12
            );

        debug::debug_sgt.init_state.line(init_info_str);
    }
#endif

    if (traj_queue.size() < MAX_QUEUE_SIZE) {
        return;
    } else {

        FirstTimeInitializationReady = std::all_of(traj_queue.begin(), traj_queue.end(), [this](const POS_INFO &ele) {
            bool ready_ = false;

            switch (parameters_sgt.get_gnss_fusion_mode()) {
                case Parameters::GnssFusionMode::GNSS_LOOSE_COUPLE: {
                    bool is_dynamic_     = ele.dp_ins > 0.2;                                        // 帧间运动距离超过20cm [10Hz，2m/s]
                    bool is_blh_precise_ = ele.delta_blh_in_meters.block<2, 1>(0, 0).norm() < 0.30; // INS递推与RTK误差小于30cm
                    ready_               = is_dynamic_ && is_blh_precise_;
                } break;

                case Parameters::GnssFusionMode::GNSS_TIGHT_COUPLE: {
                } break;

                case Parameters::GnssFusionMode::RTK_LOOSE_COUPLE: {
                    bool fast_init_ = true;
                    // 快速初始化要求，这里做一个配置。对于精度要求没那么高的场景，可以放宽初始化准出条件，让融合定位快速的完成初始化。
                    // 但是对于对定位要求高的场景，还是需要较为严格的限制初始化完成条件。
                    if (fast_init_) {
                        // fast init setting
                        bool is_dynamic_     = ele.dp_ins > 0.2;                                        // 帧间运动距离超过20cm [10Hz，2m/s]
                        bool is_blh_precise_ = ele.delta_blh_in_meters.block<2, 1>(0, 0).norm() < 0.30; // INS递推与RTK误差小于30cm
                        ready_               = is_dynamic_ && is_blh_precise_;
                    } else {
                        bool is_fix_         = ele.rtk_status == 6;                                     // 固定解
                        bool is_dynamic_     = ele.dp_ins > 0.2;                                        // 帧间运动距离超过20cm [10Hz，2m/s]
                        bool is_blh_precise_ = ele.delta_blh_in_meters.block<2, 1>(0, 0).norm() < 0.15; // INS递推与RTK误差小于15cm
                        ready_               = is_fix_ && is_dynamic_ && is_blh_precise_;
                    }
                } break;

                case Parameters::GnssFusionMode::RTK_TIGHT_COUPLE: {
                } break;

                default:
                    break;
            }
            return ready_;
        }); //
    }
}

/**
 * ==================== GnssConsistencyValidator 实现 ====================
 *
 * 功能说明：
 * 使用10帧间隔的两帧进行逐帧连续判断，验证卫星航向、速度、轨迹追踪的自洽性。
 *
 * 验证条件（必须全部满足才能进行自洽性验证）：
 * 1. 缓冲区足够（至少 FRAME_INTERVAL + 1 帧）
 * 2. 帧间隔合理（不超过 GNSS_MAX_TWO_FRAME_INTERVAL_S = 2.0秒）
 * 3. 位移 > 5米
 * 4. 平均速度 > 5米/秒
 *
 * 三对物理量验证：
 * 1. 卫星航向 vs 轨迹追踪航向：验证卫星报告的航向与实际轨迹方向是否一致
 * 2. 位移矢量方向 vs 速度积分矢量方向：验证位置变化与速度积分的一致性
 * 3. 速度方向 vs 卫星航向：验证速度方向与卫星航向的一致性
 *
 * 自洽性输出：
 * - 综合评分 consistency_score：范围[0,1]，1表示完全自洽
 * - 各项差异以角度(度)表示
 */

GnssConsistencyValidator::ConsistencyResult GnssConsistencyValidator::insert_and_validate(const GnssDataPtr gps_data_ptr) {
    // 将当前帧数据存入缓冲区
    gnss_buffer_.push_back(*gps_data_ptr);

    // 控制缓冲区大小，保持足够的历史数据用于10帧间隔的判断
    while (gnss_buffer_.size() > static_cast<size_t>(FRAME_INTERVAL + 1)) {
        gnss_buffer_.pop_front();
    }

    ConsistencyResult result;

    // ==================== 验证条件1：缓冲区是否足够 ====================
    if (gnss_buffer_.size() < static_cast<size_t>(FRAME_INTERVAL + 1)) {
        latest_result_ = result;
        return result;
    }

    // 获取参考帧（前第10帧）和当前帧
    const GnssData &ref_frame = gnss_buffer_.front();
    const GnssData &cur_frame = gnss_buffer_.back();

    // ==================== 验证条件2：帧间隔是否合理 ====================
    result.frame_interval_s   = std::abs(cur_frame.measurement_timestamp - ref_frame.measurement_timestamp);
    bool frame_interval_valid = (result.frame_interval_s <= GNSS_MAX_TWO_FRAME_INTERVAL_S);

    // ==================== 计算位移距离 ====================
    Eigen::Vector3d disp_vec = local_trans_.LLAtoEgoRfu(
        ref_frame.lla, cur_frame.lla, Eigen::Quaterniond::Identity(), true);
    result.displacement_m = disp_vec.norm();

    // ==================== 计算平均速度 ====================
    result.avg_speed_mps = (ref_frame.vel.norm() + cur_frame.vel.norm()) / 2.0;

    // ==================== 验证条件3&4：位移和速度是否满足阈值 ====================
    bool displacement_valid = (result.displacement_m > MIN_DISPLACEMENT_METERS);
    bool speed_valid        = (result.avg_speed_mps > MIN_SPEED_MPS);

    // ==================== 综合判断是否有效 ====================
    result.is_valid = frame_interval_valid && displacement_valid && speed_valid;

    // 如果不满足验证条件，直接返回（不进行自洽性计算）
    if (!result.is_valid) {
        result.consistency_score = 0.0;
        latest_result_           = result;
        return result;
    }

    // ==================== 预计算轨迹航向 ====================
    double traj_hdg_deg = local_trans_.HeadingFromBLH(ref_frame.lla, cur_frame.lla);
    double traj_hdg_rad = traj_hdg_deg / 180.0 * M_PI;
    if (traj_hdg_rad > M_PI) {
        traj_hdg_rad -= 2.0 * M_PI;
    }

    // ==================== 第一对：卫星航向 vs 轨迹航向 ====================
    double gnss_hdg_rad      = cur_frame.hdg;
    double hdg_traj_diff_rad = normalize_angle_diff(gnss_hdg_rad - traj_hdg_rad);
    result.hdg_traj_diff_deg = hdg_traj_diff_rad * 180.0 / M_PI;

    // ==================== 第二对：位移方向 vs 速度积分方向 ====================
    Eigen::Vector2d avg_vel_enu       = (ref_frame.vel.block<2, 1>(0, 0) + cur_frame.vel.block<2, 1>(0, 0)) / 2.0;
    double          vel_integ_hdg_rad = compute_vector_heading(avg_vel_enu);
    double          disp_vel_diff_rad = normalize_angle_diff(traj_hdg_rad - vel_integ_hdg_rad);
    result.disp_vel_diff_deg          = disp_vel_diff_rad * 180.0 / M_PI;

    // ==================== 第三对：速度方向 vs 卫星航向 ====================
    Eigen::Vector2d cur_vel_enu      = cur_frame.vel.block<2, 1>(0, 0);
    double          vel_hdg_rad      = compute_vector_heading(cur_vel_enu);
    double          vel_hdg_diff_rad = normalize_angle_diff(vel_hdg_rad - gnss_hdg_rad);
    result.vel_hdg_diff_deg          = vel_hdg_diff_rad * 180.0 / M_PI;

    // ==================== 综合自洽性评分计算 ====================
    // 自洽阈值：当角度差异超过3度时，评分降为0
    // 对于高精度GNSS观测，航向和速度方向差异应控制在较小范围内
    static constexpr double HDG_TRAJ_THRESHOLD_DEG = 3.0;
    static constexpr double DISP_VEL_THRESHOLD_DEG = 3.0;
    static constexpr double VEL_HDG_THRESHOLD_DEG  = 3.0;

    double score_hdg_traj = std::max(0.0, 1.0 - std::abs(result.hdg_traj_diff_deg) / HDG_TRAJ_THRESHOLD_DEG);
    double score_disp_vel = std::max(0.0, 1.0 - std::abs(result.disp_vel_diff_deg) / DISP_VEL_THRESHOLD_DEG);
    double score_vel_hdg  = std::max(0.0, 1.0 - std::abs(result.vel_hdg_diff_deg) / VEL_HDG_THRESHOLD_DEG);

    static constexpr double WEIGHT_HDG_TRAJ = 0.4;
    static constexpr double WEIGHT_DISP_VEL = 0.2;
    static constexpr double WEIGHT_VEL_HDG  = 0.4;

    result.consistency_score = WEIGHT_HDG_TRAJ * score_hdg_traj + WEIGHT_DISP_VEL * score_disp_vel + WEIGHT_VEL_HDG * score_vel_hdg;

    // 更新最新结果
    latest_result_ = result;

    // ==================== 输出调试信息到CSV文件 ====================
#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
    {
        std::string consistency_str = fmt::format(
            "{:>14.4f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:d}\n",
            cur_frame.measurement_timestamp, // 1: 时间戳
            result.consistency_score,        // 2: 自洽性评分
            result.hdg_traj_diff_deg,        // 3: 航向与轨迹差异(度)
            result.disp_vel_diff_deg,        // 4: 位移与速度差异(度)
            result.vel_hdg_diff_deg,         // 5: 速度与航向差异(度)
            result.frame_interval_s,         // 6: 帧间隔(秒)
            result.displacement_m,           // 7: 位移距离(米)
            result.avg_speed_mps,            // 8: 平均速度(m/s)
            (int)result.is_valid             // 9: 是否符合验证条件
        );
        debug::debug_sgt.gnss_consistency_state.line(consistency_str);
    }
#endif

    return result;
}

/**
 * 归一化角度差异到 [-PI, PI] 范围
 */
double GnssConsistencyValidator::normalize_angle_diff(double angle_diff) const {
    while (angle_diff > M_PI) {
        angle_diff -= 2.0 * M_PI;
    }
    while (angle_diff < -M_PI) {
        angle_diff += 2.0 * M_PI;
    }
    return angle_diff;
}

/**
 * 计算向量方向角（ENU坐标系）
 * heading = atan2(东向分量, 北向分量)
 */
double GnssConsistencyValidator::compute_vector_heading(const Eigen::Vector2d &vec) const {
    if (vec.norm() < 1e-6) {
        return 0.0;
    }
    return std::atan2(vec.x(), vec.y());
}

} // namespace MSF