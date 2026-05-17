#pragma once

#include "Eigen/Dense"
#include "geo_trans.h"

#include <iostream>

namespace byd {
namespace geo {

class LocalTrans : private Trans {
private:
    double rad2deg = 180.0 / M_PI;
    double deg2rad = M_PI / 180.0;

public:
    inline Eigen::Vector3d BLH2ENU(const Eigen::Vector3d &BLH_ref, const Eigen::Vector3d &BLH) {
        CoordB          blh_ref = {BLH_ref.x() * rad2deg, BLH_ref.y() * rad2deg, BLH_ref.z()};
        CoordB          blh     = {BLH.x() * rad2deg, BLH.y() * rad2deg, BLH.z()};
        auto            enu     = blh2enu(blh_ref, blh);
        Eigen::Vector3d ENU     = {enu.e, enu.n, enu.u};
        return ENU;
    }

public:
    // lla -> latitude longitude altitude 纬经高

    // 输入参考点纬经高、目标点纬经高、自车姿态
    // 输出 自车系（右前上）位置
    inline Eigen::Vector3d LLAtoEgoRfu(const Eigen::Vector3d &lla_ref, const Eigen::Vector3d &lla_, const Eigen::Quaterniond &att_b2ref_frame, bool no_height_ = false) {
        CoordB blh_ref = {lla_ref.x() * rad2deg, lla_ref.y() * rad2deg, lla_ref.z()};
        CoordB blh_    = {lla_.x() * rad2deg, lla_.y() * rad2deg, lla_.z()};
        if (no_height_) {
            blh_ref.h = 0.0;
            blh_.h    = 0.0;
        }
        auto            enu_    = blh2enu(blh_ref, blh_);
        Eigen::Vector3d enu     = {enu_.e, enu_.n, enu_.u};
        Eigen::Vector3d ego_rfu = att_b2ref_frame.toRotationMatrix().transpose() * enu;
        return ego_rfu;
    }

    // 输入参考点、纬经高变化量、自车姿态
    // 输出 自车系（右前上）位置
    inline Eigen::Vector3d DPos_LLAtoEgo(const Eigen::Vector3d &lla_ref, const Eigen::Vector3d &delta_lla, const Eigen::Quaterniond &att_b2ref_frame) {
        Eigen::Vector3d lla_    = lla_ref + delta_lla;
        CoordB          blh_ref = {lla_ref.x() * rad2deg, lla_ref.y() * rad2deg, lla_ref.z()};
        CoordB          blh_    = {lla_.x() * rad2deg, lla_.y() * rad2deg, lla_.z()};
        auto            enu_    = blh2enu(blh_ref, blh_);
        Eigen::Vector3d enu     = {enu_.e, enu_.n, enu_.u};
        Eigen::Vector3d ego_rfu = att_b2ref_frame.toRotationMatrix().transpose() * enu;
        return ego_rfu;
    }

    // 输入参考点、右前上变化量、自车姿态
    // 输出 纬经高变化量
    inline Eigen::Vector3d DPos_Ego2LLA(const Eigen::Vector3d &lla_ref, const Eigen::Vector3d &delta_rfu, const Eigen::Quaterniond &att_b2ref_frame) {
        Eigen::Vector3d enu_    = att_b2ref_frame.toRotationMatrix() * delta_rfu;
        CoordE          enu     = {enu_.x(), enu_.y(), enu_.z()};
        CoordB          blh_ref = {lla_ref.x() * rad2deg, lla_ref.y() * rad2deg, lla_ref.z()};
        auto            blh_    = enu2blh(blh_ref, enu);
        Eigen::Vector3d lla_    = {blh_.b * deg2rad, blh_.l * deg2rad, blh_.h};
        return lla_;
    }

    // 输入 自车系（RFU）下状态 自车姿态
    // 输出 参考系（ENU）下状态
    inline Eigen::Vector3d State_Ego2ENU(const Eigen::Vector3d &vel_rfu, const Eigen::Quaterniond &att_b2ref_frame) {
        return att_b2ref_frame.toRotationMatrix() * vel_rfu;
    }

    // 输入 参考系（ENU）下状态 自车姿态
    // 输出 自车系（RFU）下状态
    inline Eigen::Vector3d State_ENU2Ego(const Eigen::Vector3d &vel_enu, const Eigen::Quaterniond &att_b2ref_frame) {
        return att_b2ref_frame.toRotationMatrix().transpose() * vel_enu;
    }

    // 只限制横向修正量
    inline Eigen::Vector3d LLAUpdateWithEgoLatConstrain(const Eigen::Vector3d &lla_ref, const Eigen::Vector3d &delta_lla, const Eigen::Quaterniond &att_b2ref_frame, double bound_, double lat_constrain) {
        Eigen::Vector3d dp_rfu = DPos_LLAtoEgo(lla_ref, delta_lla, att_b2ref_frame);
        // constrain ego lat
        constrain_if_in_bound(dp_rfu.x(), bound_, lat_constrain);
        return DPos_Ego2LLA(lla_ref, dp_rfu, att_b2ref_frame);
    }

    // 限制横向修正量，在限制横向修正量的时候，其他轴按比例缩放
    inline Eigen::Vector3d LLAUpdateWithEgoLatScaleConstrain(const Eigen::Vector3d &lla_ref, const Eigen::Vector3d &delta_lla, const Eigen::Quaterniond &att_b2ref_frame, double lat_constrain) {
        Eigen::Vector3d dp_rfu = DPos_LLAtoEgo(lla_ref, delta_lla, att_b2ref_frame);
        if (std::fabs(dp_rfu.x()) > std::fabs(lat_constrain) && std::fabs(dp_rfu.x()) > 1e-8) {
            dp_rfu = dp_rfu * std::fabs(lat_constrain) / std::fabs(dp_rfu.x());
        }
        return DPos_Ego2LLA(lla_ref, dp_rfu, att_b2ref_frame);
    }

    // 根据两个blh坐标计算航向角
    // 北偏东 0~360
    inline double HeadingFromBLH(const Eigen::Vector3d &blh_ref_, const Eigen::Vector3d &blh_) {
        CoordB blh_ref = {blh_ref_.x() * rad2deg, blh_ref_.y() * rad2deg, blh_ref_.z()};
        CoordB blh     = {blh_.x() * rad2deg, blh_.y() * rad2deg, blh_.z()};
        auto   enu     = blh2enu(blh_ref, blh);
        auto   ori     = orientation(enu);
        return ori.az;
    }

    // 根据两个DR坐标计算航向角
    // DR系FLU
    // 输出顺时针角度 0~360
    inline double HeadingFromLocalPos(const Eigen::Vector3d &pos_ref_, const Eigen::Vector3d &pos_) {
        double dx = pos_.x() - pos_ref_.x();
        double dy = pos_.y() - pos_ref_.y();
        if (dx == 0.0 || dy == 0.0) {
            return 0.0;
        }
        double arc = std::atan2(dy, dx);
        double deg = -arc * 180.0 / M_PI;
        if (deg < 0.0) {
            deg += 360.0;
        }
        return deg;
    }

public:
    //
    static inline void constrain_if_in_bound(double &state, double bound_, double constrain_) {
        double bound     = std::abs(bound_);
        double constrain = std::abs(constrain_);
        if (std::abs(state) < bound) {
            if (state > constrain) {
                state = constrain;
            } else if (state < -constrain) {
                state = -constrain;
            }
        }
    }
};
} // namespace geo
} // namespace byd