#pragma once

#include "Eigen/Eigen"
#include "base_type.h"
#include "geo_trans.h"
#include "igg3.h"
#include "kalman_filter.h"

#include "local_trans.h"
#include "state_analysis.h"

#include "calc.h"
#include "signal_filter.h"

namespace MSF {

class VisionProcessor {

private:
    // 这里认为频率浮动范围为 ±2.5
    enum FREQUENCY {
        F_LOW  = 0,
        F_5Hz  = 1,
        F_10Hz = 2,
        F_15Hz = 3,
        F_20Hz = 4,
        F_HIGH = 6,
    };

private:
    // 设计上是10Hz
    FREQUENCY cur_freq = F_10Hz;
    void      update_frequency(double pre_timestamp, double cur_timestamp);

private:
    const double dt_bound = 1e-10;

public:
    VisionProcessor() {

        double EPS = std::abs(std::numeric_limits<double>::epsilon());

        igg3_pos_lat            = IGG3(parameters_sgt.get_vision_pos_lat_igg3_k0(), parameters_sgt.get_vision_pos_lat_igg3_k1());
        igg3_pos_lon            = IGG3(parameters_sgt.get_vision_pos_lon_igg3_k0(), parameters_sgt.get_vision_pos_lon_igg3_k1());
        igg3_hdg                = IGG3(parameters_sgt.get_vision_hdg_igg3_k0(), parameters_sgt.get_vision_hdg_igg3_k1());
        vision_pos_lat_bias_std = std::abs(parameters_sgt.get_vision_pos_lat_bias_std()) + EPS;
        vision_pos_lon_bias_std = std::abs(parameters_sgt.get_vision_pos_lon_bias_std()) + EPS;
        vision_heading_bias_std = std::abs(parameters_sgt.get_vision_heading_bias_std()) + EPS;

        vision_pos_lat_additional_std_scale = parameters_sgt.get_vision_pos_lat_additional_std_scale();
        vision_pos_lon_additional_std_scale = parameters_sgt.get_vision_pos_lon_additional_std_scale();
        vision_heading_additional_std_scale = parameters_sgt.get_vision_heading_additional_std_scale();

        constrain_map_bias = parameters_sgt.get_constrain_map_bias();
        pre_timestamp      = 0.0;
        vf_mean_std_.set_max_size(10);
        vf_mode_switch_ = false; // false for maintain, true for map bias estimate
    }

private:
    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

public:
    bool UpdateStateByVision(const VisionFusionDataPtr vis_data_ptr, StatePtr state_ptr, KfPtr<21, 3> kf_ptr);

private:
    byd::geo::Trans      geotrans;
    byd::geo::LocalTrans localtrans;

private:
    IGG3 igg3_pos_lat;
    IGG3 igg3_pos_lon;
    IGG3 igg3_hdg;

private:
    double vision_pos_lat_bias_std;
    double vision_pos_lon_bias_std;
    double vision_heading_bias_std;

    double vision_pos_lat_additional_std_scale;
    double vision_pos_lon_additional_std_scale;
    double vision_heading_additional_std_scale;

    Eigen::Vector3d constrain_map_bias;

private:
    double pre_timestamp;

private:
    statistics::StateMeanStd vf_mean_std_;

private:
    VLPF<3> vf_lpf_ = VLPF<3>(10.0, 1.0);

private:
    double vf_heading_std_estimate();

private:
    bool vf_mode_switch_;

private:
    void vis_weight_adaptation(double lat_, double lat_mean_, double lat_std_, double &scale_);

private:
    double latest_gnss_measurement_mileage_ = 0.0;
};

} // namespace MSF