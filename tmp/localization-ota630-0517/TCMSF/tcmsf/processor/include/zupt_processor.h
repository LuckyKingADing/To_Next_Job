#pragma once

#include "Eigen/Eigen"
#include "analysis.h"
#include "base_type.h"
#include "calc.h"
#include "igg3.h"
#include "local_trans.h"

#include "kalman_filter.h"

#include <fstream>
#include <iomanip>

#include <deque>
namespace MSF {

class ZuptProcessor {
public:
    ZuptProcessor() {
        constrain_gyro_bias        = parameters_sgt.get_constrain_gyro_bias();
        constrain_acc_bias         = parameters_sgt.get_constrain_acc_bias();
        constrain_P_std_min        = parameters_sgt.get_constrain_P_std_min();
        constrain_P_std_max        = parameters_sgt.get_constrain_P_std_max();
        igg3                       = IGG3(parameters_sgt.get_zupt_igg3_k0(), parameters_sgt.get_zupt_igg3_k1());
        continous_zero_speed_count = 0;
    }

private:
    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

private:
    Eigen::Vector3d constrain_gyro_bias;
    Eigen::Vector3d constrain_acc_bias;

    Eigen::Matrix<double, 21, 1> constrain_P_std_min;
    Eigen::Matrix<double, 21, 1> constrain_P_std_max;

    Vector21d delta_x;

private:
    IGG3 igg3;

private:
    uint64_t continous_zero_speed_count;

public:
    bool UpdateStateByZupt(StatePtr state_ptr, KfPtr<21, 3> kf_ptr);

private:
    byd::geo::LocalTrans localtrans;

private:
    uint64_t zupt_count_ = 0;
};

} // namespace MSF