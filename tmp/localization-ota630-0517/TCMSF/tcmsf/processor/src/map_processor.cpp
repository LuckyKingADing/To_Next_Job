#include "map_processor.h"
#include "calc.h"
#include "cyber/common/log.h"
#include "fmt/format.h"
#include "processor_debug.h"

namespace MSF {

MapProcessor::MapProcessor() {
    // 从配置文件获取 sd_map 相关参数
    max_bias_propor_dist_    = parameters_sgt.get_sd_map_max_bias_propor_dist();
    map_bias_bound_          = parameters_sgt.get_sd_map_bias_bound();
    min_thres_trig_corr_     = parameters_sgt.get_sd_map_min_thres_trig_corr();
    bias_fading_propor_dist_ = parameters_sgt.get_sd_map_bias_fading_propor_dist();
}

bool MapProcessor::UpdateStateByMap(const MapPosDataPtr map_data_ptr, StatePtr state_ptr, KfPtr<21, 3> kf_ptr) {

    // 地图没有高度信息，这里只使用经纬度
    Eigen::Vector3d state_ll0_   = Eigen::Vector3d::Zero();
    Eigen::Vector3d map_ll0_     = Eigen::Vector3d::Zero();
    state_ll0_.block<2, 1>(0, 0) = state_ptr->lla.block<2, 1>(0, 0);
    map_ll0_.block<2, 1>(0, 0)   = map_data_ptr->lla.block<2, 1>(0, 0);

    // 这里将地图投影点，以DB中心线为参考，补偿到道路中心。
    double DbVeh_lat_ = -state_ptr->db_projection.distance_smoothed;

    // 做横向补偿，并将高度置零
    Eigen::Vector3d map_ll0_mid_db_ = local_trans.DPos_Ego2LLA(map_ll0_, {DbVeh_lat_, 0.0, 0.0}, state_ptr->att);
    map_ll0_mid_db_.z()             = 0.0;
    state_ptr->sdmap_proj_mid_db    = map_ll0_mid_db_;

    if (process_control_sgt.fusion_status.imu_update_count_after_vision_fusion_update < 200) {
        // 如果两秒内有视觉融合，则不再进行地图融合
        return false;
    }

    // 以下融合地图信息的时候，使用补偿到道路中心的投影点
    Eigen::Vector3d dpos_rfu_          = local_trans.LLAtoEgoRfu(state_ll0_, map_ll0_mid_db_, state_ptr->att);
    double          dpos_distance_     = dpos_rfu_.norm();
    double          dpos_lat_distance_ = std::fabs(dpos_rfu_.x());

    {
        // 这里依据自车位置和地图投影点，计算地图偏置的修正量
        // 自车系下的偏差量 -> 参考坐标（自车位置 + 偏置量），目标坐标（投影点），单位米，轴向右前上
        Eigen::Vector3d veh_ll0_with_mb_      = state_ptr->lla + state_ptr->MpvCnb * state_ptr->lever_imu2vehicle + state_ptr->Mpv * state_ptr->sdmap_bias_enu;
        veh_ll0_with_mb_.z()                  = 0.0;
        Eigen::Vector3d dpos_rfu_mb_          = local_trans.LLAtoEgoRfu(veh_ll0_with_mb_, map_ll0_mid_db_, state_ptr->att);
        double          dpos_mb_lat_distance_ = std::fabs(dpos_rfu_mb_.x());
        // 地图到自车的偏差量 ENU
        // 这里额外做一步，只考虑自车横向的修正量
        // 转换到ENU坐标系
        auto            map_ll_lat_  = local_trans.DPos_Ego2LLA(veh_ll0_with_mb_, {dpos_rfu_mb_.x(), 0.0, dpos_rfu_mb_.z()}, state_ptr->att);
        Eigen::Vector3d dpos_enu_mb_ = local_trans.BLH2ENU(veh_ll0_with_mb_, map_ll_lat_);

        // 横向偏差量在一定值范围内，认为是有效的
        if (pre_map_measure_time_ != 0.0 && dpos_mb_lat_distance_ < 50.0 && dpos_lat_distance_ < 50.0) {
            // 距离小于100米，做地图偏置量更新
            double map_dt_ = map_data_ptr->measurement_timestamp - pre_map_measure_time_;
            UpdateMapbiasSppMode(dpos_enu_mb_, map_dt_, state_ptr);
        }
        pre_map_measure_time_ = map_data_ptr->measurement_timestamp;

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
        std::string sdmap_str = "";

        sdmap_str = fmt::format(                                     //
            "{:>14.4f},{:>14.10f},{:>14.10f},{:>14.10f},{:>14.10f}," //
            "{:>14.10f},{:>14.10f},"                                 //
            "{:>8.4f},{:>8.4f},{:>8.4f},{:>8.4f},"                   //
            "{:>8.4f}\n",                                            //
            map_data_ptr->measurement_timestamp,                     // time
            state_ptr->lla.x() * 180.0 / M_PI,                       // ins lat
            state_ptr->lla.y() * 180.0 / M_PI,                       // ins lon
            map_data_ptr->lla.x() * 180.0 / M_PI,                    // sdmap_proj lat
            map_data_ptr->lla.y() * 180.0 / M_PI,                    // sdmap_proj lon
            map_ll0_mid_db_.x() * 180.0 / M_PI,                      // sdmap_proj_db_mid lat
            map_ll0_mid_db_.y() * 180.0 / M_PI,                      // sdmap_proj_db_mid lon
            dpos_enu_mb_.x(),                                        // enu_e
            dpos_enu_mb_.y(),                                        // enu_n
            dpos_rfu_.x(),                                           // rfu_r
            dpos_rfu_.y(),                                           // rfu_f
            DbVeh_lat_                                               // DbVeh_lat_
        );
        debug::debug_sgt.sdmap_state.line(sdmap_str);
#endif
    }

    if (dpos_lat_distance_ < 2.0 || dpos_lat_distance_ > 50.0 || dpos_distance_ > 50.0) {
        // 如果横向投影点距离小于一定值则不融合
        // 如果横向投影点距离大于一定值则不融合
        // 如果投影点距离大于一定值则不融合
        return false;
    }

    constexpr static double MAP_INFO_DT_               = 0.05; // 消息时间间隔
    constexpr static double MAX_PROPORTIONAL_DISTANCE_ = 0.01; // 最大修正量距离比例
    constexpr static double MAP_POS_ERR_STD_           = 1e2;  // 地图误差均值
    constexpr static double LARGE                      = 1e7;  // 定义一个较大的值

    // 滤波器相关
    Eigen::Matrix<double, 3, 21> H          = Eigen::Matrix<double, 3, 21>::Zero();
    Eigen::Vector3d              innovation = Eigen::Vector3d::Zero();

    H.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();

    // 每200个打印一个心跳信息
    AINFO_EVERY(200) << "[HeartBeat] Map Fusion, TS: " << fmt::format("{:14.4f}", map_data_ptr->measurement_timestamp);

    innovation     = state_ptr->lla - local_trans.DPos_Ego2LLA(state_ptr->lla, Eigen::Vector3d{dpos_rfu_.x(), 0.0, 0.0}, state_ptr->att);
    innovation.z() = 0.0;

    Eigen::Matrix3d map_cov_ego_ = Eigen::Vector3d(MAP_POS_ERR_STD_, LARGE, LARGE).cwiseAbs2().asDiagonal();
    Eigen::Matrix3d map_cov_lla_ = state_ptr->MpvCnb * map_cov_ego_ * state_ptr->MpvCnb.transpose();

    Eigen::Matrix3d V = Eigen::Matrix3d::Identity();

    // 长隧道场景，P矩阵发散的比较快，这里控制一下
    V = map_cov_lla_ + 100.0 * kf_ptr->cov.block<3, 3>(6, 6);

    kf_ptr->kf_update(H, V, innovation);

    // 控制修正量在前进距离的 0.01 内
    double forward_spd_        = std::fabs(state_ptr->vel_ego.y());
    double lat_pos_comp_bound_ = forward_spd_ * MAP_INFO_DT_ * MAX_PROPORTIONAL_DISTANCE_;

    Eigen::Vector3d dlla_ = -kf_ptr->dx.block<3, 1>(6, 0);

    Eigen::Vector3d pos_updata_rfu_ = local_trans.DPos_LLAtoEgo(state_ptr->lla, dlla_, state_ptr->att);

    Eigen::Vector3d lla_ = local_trans.LLAUpdateWithEgoLatScaleConstrain(
        state_ptr->lla,     //
        dlla_,              //
        state_ptr->att,     //
        lat_pos_comp_bound_ //
    );

    if (lla_.array().isNaN().any()) {
        // 防一手滤波出现NAN值
        AWARN << "map fusion result NAN detected!";
    } else {
        state_ptr->lla = lla_;
    }

    // 这里把更新协方差注释掉
    // 相当于仅作为一个补偿量来处理
    // 不进行真正的滤波

    // kf_ptr->P = kf_ptr->cov;

    // Eigen::Matrix<bool, 21, 1> idx = Eigen::Matrix<bool, 21, 1>::Ones();
    // idx.block<3, 1>(6, 0) << 0, 0, 0;

    // // 更新协方差
    // CovarianceUpdateByIdx(kf_ptr->P, kf_ptr->cov, idx, true);

    return true;
}

void MapProcessor::UpdateMapbiasSppMode(const Eigen::Vector3d &map_inno_enu, double dt, StatePtr state_ptr) {
    // map_inno_enu 需保证只有前进方向的横向修正量
    // 这里根据投影点与自车的位置更新偏置量

    if (std::fabs(dt) > 10.0 * MAP_INFO_DT_BY_DESIGN_ || dt < 0.0) {
        // 如果前后帧时间差过大的话，或者时间回退，则不使用。
        return;
    }
    if (map_inno_enu.array().isNaN().any()) {
        // 防一下NaN值
        return;
    }
    // 如果启用了db_fusion，min_thres_trig_corr_降为一半
    double effective_min_thres = parameters_sgt.get_enable_db_fusion() ? min_thres_trig_corr_ * 0.5 : min_thres_trig_corr_;
    if (map_inno_enu.block<2, 1>(0, 0).norm() < effective_min_thres) {
        // 如果地图距离自车小于一定值，则不做处理
        return;
    }

    Eigen::Vector2d dmileage_ = state_ptr->vel.block<2, 1>(0, 0).cwiseAbs() * dt;

    // 这里reverse目的是限制修正量为速度的正交方向
    Eigen::Vector2d delta_bias_bound_ = dmileage_.reverse() * max_bias_propor_dist_;

    // 限制map_inno_enu的e、n轴向绝对值范围在delta_bias_bound_范围内
    // u设置为零
    Eigen::Vector3d map_inno_enu_clamped = map_inno_enu;
    map_inno_enu_clamped.head<2>() =
        map_inno_enu.head<2>().array().abs().min(delta_bias_bound_.array().abs()) *
        map_inno_enu.head<2>().array().sign();
    map_inno_enu_clamped.z() = 0.0;

    state_ptr->sdmap_bias_enu = state_ptr->sdmap_bias_enu + map_inno_enu_clamped;

    state_ptr->sdmap_bias_enu =
        state_ptr->sdmap_bias_enu.array().abs().min(map_bias_bound_) * state_ptr->sdmap_bias_enu.array().sign();

    // double delta_bias_ = map_inno_enu;
}

void MapProcessor::MapBiasFadingSppMode(double dt, StatePtr state_ptr) {
    // 根据位移逐渐消除地图偏置量

    if (std::fabs(dt) > 1.0 || dt < 0.0) {
        // dt过大或者回退，认为有问题
        return;
    }
    if (state_ptr->sdmap_bias_enu.array().isNaN().any()) {
        state_ptr->sdmap_bias_enu.setZero();
        return;
    }
    if (state_ptr->sdmap_bias_enu.head<2>().norm() < 1e-6) {
        // 偏置量已接近零
        return;
    }

    // 计算位移量（里程模长）
    double dmileage = state_ptr->vel.head<2>().norm() * dt;

    // 计算单次最大渐消模长
    double max_fading = dmileage * bias_fading_propor_dist_;

    // 当前偏置量模长
    double bias_norm = state_ptr->sdmap_bias_enu.head<2>().norm();

    // 渐消方向（朝零方向）
    Eigen::Vector2d fading_dir = -state_ptr->sdmap_bias_enu.head<2>().normalized();

    // 实际渐消模长：不超过最大渐消量，也不超过当前偏置量
    double fading_dist = std::min(max_fading, bias_norm);

    // 更新偏置量
    state_ptr->sdmap_bias_enu.head<2>() += fading_dir * fading_dist;
}

} // namespace MSF
