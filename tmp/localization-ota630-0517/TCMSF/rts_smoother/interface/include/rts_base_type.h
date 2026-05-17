#pragma once

#include "Eigen/Eigen"

namespace RTS {

// RTS 维度，9~18维，且为3的倍数
// 3轴 姿态、速度、位置、陀螺零偏、加计零偏、底盘信息（IMU俯仰安装角、轮速误差系数、IMU航向安装角）
// 维度越大，需要消耗的内存越大，且效果不一定100%越好，9维是比较推荐的配置
constexpr static int64_t RTS_D = 12;
static_assert(RTS_D >= 9 && RTS_D <= 18 && RTS_D % 3 == 0, "RTS dimension range 9~18 and be a multiple of 3");

using NxN_RTS_D = Eigen::Matrix<double, RTS_D, RTS_D>;
using Nx1_RTS_D = Eigen::Matrix<double, RTS_D, 1>;

using Nx1_RTS_B = Eigen::Matrix<bool, RTS_D, 1>;

enum ManeuverStatus {
    STEADY       = 0,
    DYNAMIC_LOW  = 1,
    DYNAMIC_HIGH = 2,
};

struct ForwardStepInfo {
    double         timestamp;
    double         sow;
    NxN_RTS_D      Tkk1;
    NxN_RTS_D      Pk;
    NxN_RTS_D      Pkk1;
    Nx1_RTS_D      Xk;
    Nx1_RTS_D      Xkk1;
    bool           zupt;
    ManeuverStatus m_sta;
    ForwardStepInfo() {
        timestamp = 0.0;
        sow       = 0.0;
        Tkk1      = NxN_RTS_D::Identity();
        Pk        = NxN_RTS_D::Zero();
        Pkk1      = NxN_RTS_D::Zero();
        Xk        = Nx1_RTS_D::Zero();
        Xkk1      = Nx1_RTS_D::Zero();
        zupt      = false;
        m_sta     = STEADY;
    }
};

struct ForwardStepInfoLite {
    // 相较于标准模式，P矩阵只保存对角线元素
    // 两个效果，1、节约内存 2、剔除了状态之间的关联性，稳定性会更高，理论上精度会下降（实际不一定）
    double         timestamp;
    double         sow;
    NxN_RTS_D      Tkk1;
    Nx1_RTS_D      Pk;
    Nx1_RTS_D      Pkk1;
    Nx1_RTS_D      Xk;
    Nx1_RTS_D      Xkk1;
    bool           zupt;
    ManeuverStatus m_sta;
    ForwardStepInfoLite() {
        timestamp = 0.0;
        sow       = 0.0;
        Tkk1      = NxN_RTS_D::Identity();
        Pk        = Nx1_RTS_D::Zero();
        Pkk1      = Nx1_RTS_D::Zero();
        Xk        = Nx1_RTS_D::Zero();
        Xkk1      = Nx1_RTS_D::Zero();
        zupt      = false;
        m_sta     = STEADY;
    }
};

} // namespace RTS