#pragma once

#include "Eigen/Core"
#include "modules/localization/src/TCMSF/tcmsf/util/rigid/include/rigid_transform.h"
// #include <GeographicLib/LocalCartesian.hpp>

namespace MSF {

constexpr static double kDegreeToRadian = M_PI / 180.;
constexpr static double kRadianToDegree = 180. / M_PI;

// inline void ConvertLLAToENU(const Eigen::Vector3d &init_lla, const Eigen::Vector3d &point_lla, Eigen::Vector3d *point_enu) {
//     static GeographicLib::LocalCartesian local_cartesian;
//     local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
//     local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2), point_enu->data()[0], point_enu->data()[1], point_enu->data()[2]);
// }

// inline void ConvertENUToLLA(const Eigen::Vector3d &init_lla, const Eigen::Vector3d &point_enu, Eigen::Vector3d *point_lla) {
//     static GeographicLib::LocalCartesian local_cartesian;
//     local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
//     local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2), point_lla->data()[0], point_lla->data()[1], point_lla->data()[2]);
// }

// inline Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d &v) {
//     Eigen::Matrix3d w;
//     w << 0., -v(2), v(1), v(2), 0., -v(0), -v(1), v(0), 0.;

//     return w;
// }

static Eigen::Vector3d MaxEulrAnglePerMeasurementUpdate = (0.25 * kDegreeToRadian) * Eigen::Vector3d::Ones();

inline double constrain(double state_, double constrain_) {
    double constrain = std::abs(constrain_);
    double state     = state_;
    if (state > constrain) {
        state = constrain;
    } else if (state < -constrain) {
        state = -constrain;
    }
    return state;
}

inline Eigen::Vector3d constrain(const Eigen::Vector3d &state_, const Eigen::Vector3d &constrain_) {
    Eigen::Vector3d constrain = constrain_.cwiseAbs();
    Eigen::Vector3d state     = state_;

    state.x() = state.x() > constrain.x() ? constrain.x() : state.x();
    state.y() = state.y() > constrain.y() ? constrain.y() : state.y();
    state.z() = state.z() > constrain.z() ? constrain.z() : state.z();

    state.x() = state.x() < -constrain.x() ? -constrain.x() : state.x();
    state.y() = state.y() < -constrain.y() ? -constrain.y() : state.y();
    state.z() = state.z() < -constrain.z() ? -constrain.z() : state.z();

    return state;
}

inline Eigen::Vector3d UpdateGravity(double lat) {
    double g_L = 9.780325 * (1 + 0.00530240 * sinf64(lat) * sinf64(lat) - 0.00000582 * sinf64(2.0 * lat) * sinf64(2.0 * lat));
    return {0, 0, -g_L};
}

inline void StateConstrain(Eigen::Vector3d &state_, const Eigen::Vector3d bound) {

    state_.x() = state_.x() > bound.x() ? bound.x() : state_.x();
    state_.y() = state_.y() > bound.y() ? bound.y() : state_.y();
    state_.z() = state_.z() > bound.z() ? bound.z() : state_.z();

    state_.x() = state_.x() < -bound.x() ? -bound.x() : state_.x();
    state_.y() = state_.y() < -bound.y() ? -bound.y() : state_.y();
    state_.z() = state_.z() < -bound.z() ? -bound.z() : state_.z();
}

inline void StateConstrain(double &state_, double bound) {
    state_ = state_ > bound ? bound : state_;
    state_ = state_ < -bound ? -bound : state_;
}

inline void StateConstrain(Eigen::Quaterniond &state_, Eigen::Vector3d bound) {
    Eigen::Vector3d euler = INS::quaternion2euler(state_);
    StateConstrain(euler, bound);
    state_ = INS::euler2quaternion(euler);
}

inline void CovarianceConstrain(Eigen::Matrix<double, 21, 21> &cov_, const Eigen::Matrix<double, 21, 1> &min_, const Eigen::Matrix<double, 21, 1> &max_) {

    for (size_t i = 0; i < 21; i++) {
        if (cov_(i, i) < std::pow(min_(i), 2)) {
            cov_(i, i) = std::pow(min_(i), 2);
        } else if (cov_(i, i) > std::pow(max_(i), 2)) {
            double scale_ = max_(i) / std::sqrt(cov_(i, i));
            cov_.row(i)   = cov_.row(i) * scale_;
            cov_.col(i)   = cov_.col(i) * scale_;
        }
    }
}

// 依据idx，使用cov_中的值更新cov中的值
// 通过reverse标志控制更新方式：
// 如果为false，则更新idx=1对应的矩阵
// 如果为true，则更新除idx=1对应矩阵之外的元素
/*
如：
idx = 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1
inverse == false
1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1
1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1
1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1
1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1
1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1
1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1
1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1
1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1
1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1
1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1
1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1

inverse == true
0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0
0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0
0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0
0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0
0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0
0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0
1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0
0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0
0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0
0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0
0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0
0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0
*/

inline void CovarianceUpdateByIdx(Eigen::Matrix<double, 21, 21> &cov, const Eigen::Matrix<double, 21, 21> &cov_, const Eigen::Matrix<bool, 21, 1> &idx, bool reverse = false) {
    if (reverse) {
        for (Eigen::Index i = 0; i < idx.rows(); i++) {
            if (!idx(i)) {
                cov.row(i) = cov_.row(i);
                cov.col(i) = cov_.col(i);
            }
        }
    } else {
        for (Eigen::Index i = 0; i < idx.rows(); i++) {
            if (idx(i)) {
                for (Eigen::Index j = 0; j < idx.size(); j++) {
                    if (idx(j)) {
                        cov(i, j) = cov_(i, j);
                    }
                }
            }
        }
    }
}

// 与上一个函数类似，但是将矩阵块更新为特定值
inline void CovarianceUpdateByIdx(Eigen::Matrix<double, 21, 21> &cov, double c_, const Eigen::Matrix<bool, 21, 1> &idx, bool reverse = false) {
    if (reverse) {
        for (Eigen::Index i = 0; i < idx.rows(); i++) {
            if (!idx(i)) {
                cov.row(i).array() = c_;
                cov.col(i).array() = c_;
            }
        }
    } else {
        for (Eigen::Index i = 0; i < idx.rows(); i++) {
            if (idx(i)) {
                for (Eigen::Index j = 0; j < idx.size(); j++) {
                    if (idx(j)) {
                        cov(i, j) = c_;
                    }
                }
            }
        }
    }
}

inline double RtkFloatStdModification(double x) {
    constexpr double px0 = 0.0;
    constexpr double px1 = 2.0;
    constexpr double px2 = 4.0;
    constexpr double px3 = 6.0;
    constexpr double px4 = 8.0;
    constexpr double px5 = 10.0;

    constexpr double py0 = 0.0;
    constexpr double py1 = 1.0;
    constexpr double py2 = 2.5;
    constexpr double py3 = 4.5;
    constexpr double py4 = 7.0;
    constexpr double py5 = 10.0;

    if (x < 0.1)
        x = 0.1;

    if (px1 >= x && x > px0)
        return py0 + (py1 - py0) / (px1 - px0) * (x - px0);

    if (px2 >= x && x > px1)
        return py1 + (py2 - py1) / (px2 - px1) * (x - px1);

    if (px3 >= x && x > px2)
        return py2 + (py3 - py2) / (px3 - px2) * (x - px2);

    if (px4 >= x && x > px3)
        return py3 + (py4 - py3) / (px4 - px3) * (x - px3);

    if (px5 >= x && x > px4)
        return py4 + (py5 - py4) / (px5 - px4) * (x - px4);

    if (x > px5)
        return x;
}

inline double linear_interpolation(double x0, double x1, double y0, double y1, double x_) {
    if (std::abs(x1 - x0) < std::numeric_limits<double>::epsilon()) {
        return x0;
    }
    return y0 + (x_ - x0) * (y1 - y0) / (x1 - x0);
}

} // namespace MSF