#pragma once

#include "Eigen/Eigen"
#include "base_type.h"
#include "kalman_filter.h"
#include "local_trans.h"

namespace MSF {
class MapProcessor {

private:
    byd::geo::LocalTrans local_trans;

private:
    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

public:
    MapProcessor();

public:
    bool UpdateStateByMap(const MapPosDataPtr map_data_ptr, StatePtr state_ptr, KfPtr<21, 3> kf_ptr);

private:
    double pre_map_measure_time_ = 0.0;
    void   UpdateMapbiasSppMode(const Eigen::Vector3d &map_inno_enu, double dt, StatePtr state_ptr);

public:
    void MapBiasFadingSppMode(double dt, StatePtr state_ptr);

private:
    constexpr static double MAP_INFO_DT_BY_DESIGN_ = 0.05; // 消息设计时间间隔

    // sd_map 配置参数（从配置文件获取）
    double max_bias_propor_dist_;
    double map_bias_bound_;
    double min_thres_trig_corr_;
    double bias_fading_propor_dist_;
};
} // namespace MSF